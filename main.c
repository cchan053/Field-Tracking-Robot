/*
** Timers:
**   Timer 0 -- SMBus (I2C) clock for VL53L0X
**   Timer 1 -- UART baud rate
**   Timer 2 -- PWM ISR at 10 kHz
**   Timer 3 -- microsecond delay utility
**   Timer 4 -- free-running 16-bit counter for IR pulse timing
**
** H-bridge:
**   P1.2 -> Left  fwd,  P1.3 -> Left  bwd
**   P1.0 -> Right fwd,  P1.1 -> Right bwd
**
** Inductors:
**   P2.1 = Left,  P2.2 = Center,  P2.3 = Right
**
** IR:
**   P0.2 = TSOP33338 output (active-low)
**
** Modes:
**   AUTO   -- follows wire, takes path at intersections
**   MANUAL -- IR remote controls motors directly
**
** Mission Logging:
**   All log data lives in XDATA.
**   When path ends ('s'), robot stops and waits for PC to connect.
**   Python app sends one byte to trigger dump, robot replies with CSV.
*/

#include <EFM8LB1.h>
#include <stdio.h>
#include "vl53l0x.h"

/*---------------------------------------------------------------------------
** IMPORTANT: do NOT redefine uint8_t / uint16_t / int8_t here.
**            EFM8LB1.h or vl53l0x.h already typedef them.
**            Using "unsigned char" / "unsigned int" / "signed char"
**            everywhere avoids duplicate-typedef errors in C51.
**---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** Mission log struct
**---------------------------------------------------------------------------*/
#define LOG_MAX 1000

struct MissionEntry
{
    unsigned int  sample_id;
    unsigned int  dist_mm;
    unsigned char l_pwm;
    unsigned char r_pwm;
    unsigned char path_idx;
    unsigned char event;
};

struct MissionEntry xdata mission_log[LOG_MAX];

static unsigned char log_count      = 0;
static unsigned char log_throttle   = 0;
static unsigned int  sample_counter = 0;

/*---------------------------------------------------------------------------
** Config
**---------------------------------------------------------------------------*/
#define SYSCLK          72000000L
#define BAUDRATE          115200L
#define SMB_FREQUENCY    400000L
#define SARCLK          18000000L
#define VDD                  3.3f

#define COLLISION_MM    150

#define PIN_LEFT    QFP32_MUX_P2_1
#define PIN_CENTER  QFP32_MUX_P2_2
#define PIN_RIGHT   QFP32_MUX_P2_3

#define DEAD_BAND       0.05f
#define INTERSECTION_V  0.2f
#define DRIVE_SPEED     100

#define INTER_COOLDOWN_MS  800
#define SPIN_TIME_MS       400

/*---------------------------------------------------------------------------
** IR timing constants
**---------------------------------------------------------------------------*/
#define TSOP_PIN_MASK   0x04

#define MODE_DRIVE      0xAA
#define MODE_SWITCH     0xAF

#define MODE_MANNUAL    0
#define MODE_AUTO1      1
#define MODE_AUTO2      2
#define MODE_AUTO3      3

#define IR_TIMEOUT      65000U

#define HDR_MIN         7055U
#define HDR_MAX         10582U
#define BIT0_HI_MIN     1029U
#define BIT0_HI_MAX     2793U
#define BIT1_HI_MIN     3969U
#define BIT1_HI_MAX     6024U
#define PKT_GAP_MIN     50000U

/*---------------------------------------------------------------------------
** H-bridge pins
**---------------------------------------------------------------------------*/
#define L_FWD  P1_2
#define L_BWD  P1_3
#define R_FWD  P1_0
#define R_BWD  P1_1

/*---------------------------------------------------------------------------
** LED pins
**---------------------------------------------------------------------------*/
#define led_left   P0_5
#define led_right  P0_6
#define led_brake  P0_7

/*---------------------------------------------------------------------------
** Global mode / path state
**---------------------------------------------------------------------------*/
volatile unsigned char current_mode = 0;
volatile int           path_pos     = 0;

/*---------------------------------------------------------------------------
** PWM state
**---------------------------------------------------------------------------*/
volatile unsigned char left_pwm  = 0;
volatile unsigned char right_pwm = 0;
volatile unsigned char left_dir  = 1;
volatile unsigned char right_dir = 1;

/*---------------------------------------------------------------------------
** IR packet storage
**---------------------------------------------------------------------------*/
unsigned char xdata ir_bits[40];
unsigned char       ir_bytes[4];

/*---------------------------------------------------------------------------
** Flash storage
**---------------------------------------------------------------------------*/
unsigned char key1, key2;
volatile int  offset;
char skip_inter    = 'n';
char end_path_flag = 'n';

/*---------------------------------------------------------------------------
** Paths
**---------------------------------------------------------------------------*/
static const char code path1[8] = {'f','l','l','f','r','l','r','s'};
static const char code path2[7] = {'l','r','l','r','f','f','s'};
static const char code path3[8] = {'r','f','r','l','r','l','f','s'};

/*===========================================================================
** Raw UART helpers
**===========================================================================*/
void uart_putc(char c)
{
    while (!TI);
    TI    = 0;
    SBUF0 = c;
}

void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

void uart_putu(unsigned int v)
{
    char          buf[6];
    unsigned char i = 0;
    if (v == 0) { uart_putc('0'); return; }
    while (v)
    {
        buf[i] = (char)('0' + (v % 10));
        i++;
        v /= 10;
    }
    while (i)
    {
        i--;
        uart_putc(buf[i]);
    }
}

/*===========================================================================
** Log one sample into XDATA
**===========================================================================*/
void log_state(unsigned char event, int dist)
{
    if (log_count >= LOG_MAX) return;

    mission_log[log_count].sample_id = sample_counter;
    sample_counter++;

    mission_log[log_count].dist_mm  = (dist > 0) ? (unsigned int)dist : 9999U;
    mission_log[log_count].l_pwm    = left_pwm;
    mission_log[log_count].r_pwm    = right_pwm;
    mission_log[log_count].path_idx = (unsigned char)path_pos;
    mission_log[log_count].event    = event;

    log_count++;
}

/*===========================================================================
** Dump mission log over UART as CSV
**===========================================================================*/
void dump_mission_log(void)
{
    unsigned char i;

    SCON0 |= 0x10;
    while (!RI);
    RI = 0;

    uart_puts("sample_id,dist_mm,left_pwm,right_pwm,path_idx,event\r\n");
    for (i = 0; i < log_count; i++)
    {
        uart_putu(mission_log[i].sample_id); uart_putc(',');
        uart_putu(mission_log[i].dist_mm);   uart_putc(',');
        uart_putu(mission_log[i].l_pwm);     uart_putc(',');
        uart_putu(mission_log[i].r_pwm);     uart_putc(',');
        uart_putu(mission_log[i].path_idx);  uart_putc(',');
        uart_putu(mission_log[i].event);
        uart_puts("\r\n");
    }
    uart_puts("END\r\n");
}

/*===========================================================================
** Startup
**===========================================================================*/
char _c51_external_startup(void)
{
    XBR0 = 0x05;
    XBR1 = 0x00;
    XBR2 = 0x40;

    CKCON0 |= 0x04;
    TMOD   &= 0xF0;
    TMOD   |= 0x02;
    TL0 = TH0 = (unsigned char)(256 - (SYSCLK / SMB_FREQUENCY / 3));
    TR0 = 1;

    SMB0CF  = 0x5C;
    SMB0CF |= 0x80;

    SFRPAGE = 0x00;
    WDTCN   = 0xDE;
    WDTCN   = 0xAD;

    VDM0CN = 0x80;
    RSTSRC = 0x06;

    SFRPAGE = 0x10;
    PFE0CN  = 0x20;
    SFRPAGE = 0x00;

    CLKSEL = 0x00; CLKSEL = 0x00; while ((CLKSEL & 0x80) == 0);
    CLKSEL = 0x03; CLKSEL = 0x03; while ((CLKSEL & 0x80) == 0);

    SCON0 = 0x10;
    TH1   = (unsigned char)(0x100 - ((SYSCLK / BAUDRATE) / 24L));
    TL1   = TH1;
    TMOD &= 0x0F;
    TMOD |= 0x20;
    TR1 = 1;
    TI  = 1;

    SFRPAGE  = 0x10;
    TMR3CN1 |= 0x60;
    SFRPAGE  = 0x00;

    SFRPAGE  = 0x10;
    TMR4CN0  = 0x00;
    TMR4H    = 0x00;
    TMR4L    = 0x00;
    CKCON1  |= 0x10;
    TMR4CN0  = 0x04;
    SFRPAGE  = 0x00;

    P0MDOUT |=  0x10;
    P0MDIN  |=  TSOP_PIN_MASK;
    P0MDOUT &= ~TSOP_PIN_MASK;
    P0      |=  TSOP_PIN_MASK;

    //P3MDIN |= 0x01;
    //P2MDIN |= 0x60;
    
    P0MDIN  |= 0xE0;   // P0.5, P0.6, P0.7 digital
    P0MDOUT |= 0xE0;   // P0.5, P0.6, P0.7 push-pull outputs

    L_FWD = 1; L_BWD = 1; R_FWD = 1; R_BWD = 1;
    //led_green = 0; led_yellow = 0; led_red = 0;

    TMR2CN0  = 0x00;
    CKCON0  |= 0x10;
    TMR2RL   = (unsigned int)(-(SYSCLK / 10000L));
    TMR2     = 0xFFFF;
    ET2      = 1;
    TR2      = 1;

    EA = 1;
    return 0;
}

/*===========================================================================
** Timer 2 ISR -- PWM at 10 kHz
**===========================================================================*/
void Timer2_ISR(void) interrupt 5
{
    static unsigned char pwm_count = 0;
    TF2H = 0;
    if (++pwm_count >= 100) pwm_count = 0;

    if (left_dir)  { L_FWD = (pwm_count < left_pwm)  ? 0 : 1; L_BWD = 1; }
    else           { L_BWD = (pwm_count < left_pwm)  ? 0 : 1; L_FWD = 1; }

    if (right_dir) { R_FWD = (pwm_count < right_pwm) ? 0 : 1; R_BWD = 1; }
    else           { R_BWD = (pwm_count < right_pwm) ? 0 : 1; R_FWD = 1; }
}

/*===========================================================================
** Motor API
**===========================================================================*/
void stop(void)
{
    left_pwm = 0; right_pwm = 0;
    L_FWD = 1; L_BWD = 1; R_FWD = 1; R_BWD = 1;
}

void motor_forward(unsigned char speed)
{
    if (speed > 100) speed = 100;
    left_dir = 1;
    right_dir = 1;
    left_pwm = speed;
    right_pwm = speed;
}

void motor_backward(unsigned char speed)
{
    if (speed > 100) speed = 100;
    left_dir = 0;
    right_dir = 0;
    left_pwm = speed;
    right_pwm = speed;
}

void turn_left(unsigned char speed)
{
    if (speed > 100) speed = 100;
    left_dir = 1;
    right_dir = 1;
    left_pwm = 0;
    right_pwm = speed;
}

void turn_right(unsigned char speed)
{
    if (speed > 100) speed = 100;
    left_dir = 1;
    right_dir = 1;
    left_pwm = speed;
    right_pwm = 0;
}

void spin_left(unsigned char speed)
{
    if (speed > 100) speed = 100;
    left_dir = 1;
    right_dir = 0;
    left_pwm = speed;
    right_pwm = speed;
}

void spin_right(unsigned char speed)
{
    if (speed > 100) speed = 100;
    left_dir = 0;
    right_dir = 1;
    left_pwm = speed;
    right_pwm = speed;
}

/*===========================================================================
** Flash save / restore
**===========================================================================*/
#define CONST_SIZE 4
#define BASE_FDATA 0xF800

#define SaveFdata(X,Y)    { FLKEY=0xA5; FLKEY=0xF1; PSCTL=0x01; \
                            *((unsigned char xdata *)(X))=(Y); PSCTL=0x00; }
#define EraseFdataPage(X) { FLKEY=0xA5; FLKEY=0xF1; PSCTL=0x03; \
                            *((unsigned char xdata *)(X))=0; PSCTL=0x00; }
#define ReadFdata(X)      (*((unsigned char code *)(X)))

void Save_Vars(void)
{
    bit           saved_EA;
    unsigned int  j;
    unsigned int  address;
    unsigned char *ptr;

    saved_EA = EA;
    EA = 0;
    EraseFdataPage(BASE_FDATA);
    key1 = 0x55;
    key2 = 0xAA;
    address = BASE_FDATA;
    ptr = &key1;
    for (j = 0; j < CONST_SIZE; j++)
    {
        SaveFdata(address, *ptr);
        address++;
        ptr++;
    }
    EA = saved_EA;
}

void Restore_Vars(void)
{
    unsigned int  j;
    unsigned int  address;
    unsigned char *ptr;

    if ((ReadFdata(BASE_FDATA) != 0x55) || (ReadFdata(BASE_FDATA + 1) != 0xAA))
    {
        offset = 0;
    }
    else
    {
        address = BASE_FDATA;
        ptr = &key1;
        for (j = 0; j < CONST_SIZE; j++)
        {
            *ptr = ReadFdata(address);
            address++;
            ptr++;
        }
    }
}

/*===========================================================================
** I2C helpers
**===========================================================================*/
void Wait_SI(void)  { unsigned int t = 5000; while ((!SI) && t) t--; }
void Wait_STO(void) { unsigned int t = 5000; while ((STO) && t)  t--; }

void I2C_write(unsigned char d) { SMB0DAT = d; SI = 0; Wait_SI(); }
unsigned char I2C_read(bit ack) { ACK = ack; SI = 0; Wait_SI(); return SMB0DAT; }
void I2C_start(void) { ACK = 0; STO = 0; STA = 1; SI = 0; Wait_SI(); }
void I2C_stop(void)  { ACK = 0; STA = 0; STO = 1; SI = 0; Wait_STO(); STO = 0; }

bit i2c_read_addr8_data8(unsigned char addr, unsigned char *v)
{
    I2C_start(); I2C_write(0x52); I2C_write(addr); I2C_stop();
    I2C_start(); I2C_write(0x53); *v = I2C_read(1); I2C_stop();
    return 1;
}

bit i2c_read_addr8_data16(unsigned char addr, unsigned int *v)
{
    I2C_start(); I2C_write(0x52); I2C_write(addr); I2C_stop();
    I2C_start(); I2C_write(0x53);
    *v  = (unsigned int)I2C_read(0) << 8;
    *v |= (unsigned int)I2C_read(1);
    I2C_stop();
    return 1;
}

bit i2c_write_addr8_data8(unsigned char addr, unsigned char v)
{
    I2C_start(); I2C_write(0x52); I2C_write(addr); I2C_write(v); I2C_stop();
    return 1;
}

/*===========================================================================
** ADC
**===========================================================================*/
void InitPinADC(unsigned char portno, unsigned char pin_num)
{
    unsigned char mask = (unsigned char)(1u << pin_num);
    SFRPAGE = 0x20;
    switch (portno)
    {
        case 0: P0MDIN &= ~mask; P0SKIP |= mask; break;
        case 1: P1MDIN &= ~mask; P1SKIP |= mask; break;
        case 2: P2MDIN &= ~mask; P2SKIP |= mask; break;
        default: break;
    }
    SFRPAGE = 0x00;
}

void InitADC(void)
{
    SFRPAGE = 0x00;
    ADEN    = 0;
    ADC0CN1 = (0x2 << 6) | (0x0 << 3) | 0x0;
    ADC0CF0 = (unsigned char)((SYSCLK / SARCLK) << 3);
    ADC0CF1 = 0x1E;
    ADC0CN0 = 0x00;
    ADC0CF2 = (0x1 << 5) | 0x1F;
    ADC0CN2 = 0x00;
    ADEN    = 1;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
    ADC0MX = pin;
    ADINT  = 0;
    ADBUSY = 1;
    while (!ADINT);
    return ADC0;
}

float Volts_at_Pin(unsigned char pin)
{
    return ((float)ADC_at_Pin(pin) * VDD) / 16383.0f;
}

/*===========================================================================
** Timer 3 delay utility
**===========================================================================*/
void Timer3us(unsigned char us)
{
    unsigned char i;
    CKCON0  |= 0x40;
    TMR3RL   = (unsigned int)(-(SYSCLK / 1000000L));
    TMR3     = TMR3RL;
    TMR3CN0  = 0x04;
    for (i = 0; i < us; i++)
    {
        while (!(TMR3CN0 & 0x80));
        TMR3CN0 &= (unsigned char)(~0x80);
    }
    TMR3CN0 = 0x00;
}

void waitms(unsigned int ms)
{
    unsigned int  j;
    unsigned char k;
    for (j = 0; j < ms; j++)
        for (k = 0; k < 4; k++)
            Timer3us(250);
}

/*===========================================================================
** IR receive
**===========================================================================*/
unsigned int Timer4_Read(void)
{
    unsigned char lo1, hi, lo2;
    SFRPAGE = 0x10;
    do { lo1 = TMR4L; hi = TMR4H; lo2 = TMR4L; } while (lo2 < lo1);
    SFRPAGE = 0x00;
    return ((unsigned int)hi << 8) | (unsigned int)lo2;
}

unsigned int IR_elapsed(unsigned int t0, unsigned int t1)
{
    return (unsigned int)(t1 - t0);
}

bit tsop_is_low(void)  { return (bit)((P0 & TSOP_PIN_MASK) == 0); }
bit tsop_is_high(void) { return (bit)((P0 & TSOP_PIN_MASK) != 0); }

bit ir_wait_level(bit want_low, unsigned int timeout)
{
    unsigned int t0 = Timer4_Read();
    while (1)
    {
        if ( want_low && tsop_is_low())  return 1;
        if (!want_low && tsop_is_high()) return 1;
        if (IR_elapsed(t0, Timer4_Read()) > timeout) return 0;
    }
}

unsigned int ir_measure_level(bit measure_low, unsigned int timeout)
{
    unsigned int t0 = Timer4_Read();
    while (1)
    {
        if ( measure_low && tsop_is_high()) break;
        if (!measure_low && tsop_is_low())  break;
        if (IR_elapsed(t0, Timer4_Read()) > timeout) break;
    }
    return IR_elapsed(t0, Timer4_Read());
}

bit ir_read_symbol(unsigned int *lo, unsigned int *hi)
{
    if (!ir_wait_level(1, IR_TIMEOUT))  return 0;
    *lo = ir_measure_level(1, IR_TIMEOUT);
    if (!ir_wait_level(0, IR_TIMEOUT))  return 0;
    *hi = ir_measure_level(0, IR_TIMEOUT);
    return 1;
}

unsigned char ir_classify(unsigned int lo, unsigned int hi)
{
    if (hi >= PKT_GAP_MIN)                               return 3;
    if (lo >= HDR_MIN && lo <= HDR_MAX &&
        hi >= HDR_MIN && hi <= HDR_MAX)                  return 2;
    if (hi >= BIT0_HI_MIN && hi <= BIT0_HI_MAX)         return 0;
    if (hi >= BIT1_HI_MIN && hi <= BIT1_HI_MAX)         return 1;
    return 255;
}

bit ir_receive_packet(unsigned char *start,
                      signed char   *left_cmd,
                      signed char   *right_cmd,
                      unsigned char *check)
{
    unsigned int  lo, hi;
    unsigned char symbol;
    unsigned char bit_count = 0;
    unsigned char i, j;

    while (1)
    {
        if (!ir_read_symbol(&lo, &hi)) return 0;
        symbol = ir_classify(lo, hi);
        if (symbol == 2) break;
        if (symbol != 3) continue;
    }

    while (bit_count < 40)
    {
        if (!ir_read_symbol(&lo, &hi)) return 0;
        symbol = ir_classify(lo, hi);
        if (symbol == 3 || symbol == 2) break;
        if (symbol == 0 || symbol == 1)
        {
            ir_bits[bit_count] = symbol;
            bit_count++;
        }
    }

    if (bit_count != 32) return 0;

    for (i = 0; i < 4; i++)
    {
        ir_bytes[i] = 0;
        for (j = 0; j < 8; j++)
        {
            ir_bytes[i] = (unsigned char)(ir_bytes[i] << 1);
            ir_bytes[i] |= ir_bits[(unsigned char)(i * 8u + j)];
        }
    }

    *start     =              ir_bytes[0];
    *left_cmd  = (signed char)ir_bytes[1];
    *right_cmd = (signed char)ir_bytes[2];
    *check     =              ir_bytes[3];
    return 1;
}

void ir_apply_command(signed char left_cmd, signed char right_cmd)
{
    if      (left_cmd > 0) { left_dir = 1; left_pwm = (unsigned char) left_cmd;  }
    else if (left_cmd < 0) { left_dir = 0; left_pwm = (unsigned char)(-left_cmd); }
    else                   { left_pwm = 0; }

    if      (right_cmd > 0) { right_dir = 1; right_pwm = (unsigned char) right_cmd;  }
    else if (right_cmd < 0) { right_dir = 0; right_pwm = (unsigned char)(-right_cmd); }
    else                    { right_pwm = 0; }
}

bit ir_poll(void)
{
    unsigned char start, check, calc;
    signed char   left_cmd, right_cmd;

    if (!tsop_is_low()) return 0;
    if (!ir_receive_packet(&start, &left_cmd, &right_cmd, &check)) return 0;

    calc = (unsigned char)(start ^ (unsigned char)left_cmd ^ (unsigned char)right_cmd);
    if ((start != MODE_SWITCH && start != MODE_DRIVE) || check != calc) return 0;

    if (start == MODE_SWITCH)
    {
        if ((unsigned char)left_cmd  == 0x00 &&
            (unsigned char)right_cmd == 0x00 && check == 0xAF)
        {
            current_mode = MODE_MANNUAL;
        }
        else if ((unsigned char)left_cmd  == 0x00 &&
                 (unsigned char)right_cmd == 0xFF && check == 0x50)
        {
            if (current_mode != MODE_AUTO1)
            {
                current_mode  = MODE_AUTO1;
                path_pos      = 0;
                skip_inter    = 'n';
                end_path_flag = 'n';
                log_count     = 0;
                log_throttle  = 0;
                sample_counter = 0;
            }
        }
        else if ((unsigned char)left_cmd  == 0xFF &&
                 (unsigned char)right_cmd == 0xFF && check == 0xAF)
        {
            if (current_mode != MODE_AUTO2)
            {
                current_mode  = MODE_AUTO2;
                path_pos      = 0;
                skip_inter    = 'n';
                end_path_flag = 'n';
                log_count     = 0;
                log_throttle  = 0;
                sample_counter = 0;
            }
        }
        else if ((unsigned char)left_cmd  == 0xFF &&
                 (unsigned char)right_cmd == 0x00 && check == 0x50)
        {
            if (current_mode != MODE_AUTO3)
            {
                current_mode  = MODE_AUTO3;
                path_pos      = 0;
                skip_inter    = 'n';
                end_path_flag = 'n';
                log_count     = 0;
                log_throttle  = 0;
                sample_counter = 0;
            }
        }
        return 1;
    }

    if (start == MODE_DRIVE && current_mode == MODE_MANNUAL)
    {
        ir_apply_command(left_cmd, right_cmd);
        return 1;
    }

    return 0;
}

/*===========================================================================
** Wire following
**===========================================================================*/
void follow_wire(float v_left, float v_right)
{
    float diff = v_left - v_right;
    if (diff < 0.0f) diff = -diff;
    if (diff > DEAD_BAND)
    {
        if (v_right > v_left) turn_left(DRIVE_SPEED);
        else                  turn_right(DRIVE_SPEED);
    }
    else
    {
        motor_forward(DRIVE_SPEED);
    }
}

/*===========================================================================
** Path following
**===========================================================================*/
void path_follow(const char code *array)
{
    char c = array[path_pos];

    if (c == 'f')
    {
        motor_forward(DRIVE_SPEED);
        path_pos++;
    }
    else if (c == 'r')
    {
        spin_right(DRIVE_SPEED);
        path_pos++;
        waitms(500);
    }
    else if (c == 'l')
    {
        spin_left(DRIVE_SPEED);
        path_pos++;
        waitms(500);
    }
    else if (c == 's')
    {
        stop();
        log_state(3, 9999);
        end_path_flag = 'y';
        dump_mission_log();
    }
}

/*===========================================================================
** Auto mode
**===========================================================================*/
void auto_mode(char path, unsigned char *success, int *wait)
{
    int   range = 0;
    int   dist  = 9999;
    float v_left, v_center, v_right;

    v_left   = Volts_at_Pin(PIN_LEFT);
    v_center = Volts_at_Pin(PIN_CENTER);
    v_right  = Volts_at_Pin(PIN_RIGHT);

    if (vl53l0x_measurement_ready())
    {
        *success = vl53l0x_read_range_continuous(&range);
        dist = (*success) ? (range - offset) : 9999;
    }

    if (*success && dist < COLLISION_MM)
    {
        stop();
        log_state(2, dist);
        while (1)
        {
            ir_poll();
            if (current_mode == MODE_MANNUAL) break;
            if (vl53l0x_measurement_ready())
            {
                *success = vl53l0x_read_range_continuous(&range);
                if (*success)
                {
                    dist = range - offset;
                    log_state(2, dist);
                    if (dist >= COLLISION_MM) break;
                }
            }
        }
        return;
    }

    if ((v_center > INTERSECTION_V) && (skip_inter == 'n'))
    {
        log_state(1, dist);

        if      (path == '1') path_follow(path1);
        else if (path == '2') path_follow(path2);
        else if (path == '3') path_follow(path3);

        skip_inter = 'y';
    }
    else if (end_path_flag == 'n')
    {
        follow_wire(v_left, v_right);

        log_throttle++;
        if (log_throttle >= 3)
        {
            log_throttle = 0;
            log_state(0, dist);
        }

        if (skip_inter == 'y') (*wait)++;
        if (*wait > 10000) { skip_inter = 'n'; *wait = 0; }
    }
}

/*===========================================================================
** Main
**===========================================================================*/
void main(void)
{
    unsigned char success = 0;
    char          path    = '1';
    int           wait    = 0;

    waitms(500);

    InitPinADC(2, 1);
    InitPinADC(2, 2);
    InitPinADC(2, 3);
    InitADC();

    Restore_Vars();
    success = vl53l0x_init();
    if (success) success = vl53l0x_start_continuous();
    
    led_brake = 1;
    led_right = 1;
    led_left = 1;

    while (1)
    {
        ir_poll();

        if      (current_mode == MODE_AUTO1) { path = '1'; auto_mode(path, &success, &wait); }
        else if (current_mode == MODE_AUTO2) { path = '2'; auto_mode(path, &success, &wait); }
        else if (current_mode == MODE_AUTO3) { path = '3'; auto_mode(path, &success, &wait); }
        /* MODE_MANNUAL: motors already set by ir_apply_command inside ir_poll */
    }
}
