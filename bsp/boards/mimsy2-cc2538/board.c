/**
 * Author: Xavier Vilajosana (xvilajosana@eecs.berkeley.edu)
 *         Pere Tuset (peretuset@openmote.com)
 * Date:   July 2013
 * Description: CC2538-specific definition of the "board" bsp module.
 */

#include <headers/hw_ioc.h>
#include <headers/hw_memmap.h>
#include <headers/hw_ssi.h>
#include <headers/hw_sys_ctrl.h>
#include <headers/hw_types.h>
#include <headers/hw_ints.h>

#include <source/flash.h>
#include <source/interrupt.h>
#include <source/ioc.h>
#include <source/gpio.h>
#include <source/gptimer.h>
#include <source/sys_ctrl.h>

#include "board.h"
#include "debugpins.h"
#include "i2c.h"
#include "leds.h"
#include "radio.h"
#include "sensors.h"
#include "sctimer.h"
#include "uart.h"
#include "cryptoengine.h"
#include "uart_mimsy.h"



//=========================== variables =======================================

#define BSP_BUTTON_BASE                 ( GPIO_C_BASE )
#define BSP_BUTTON_USER                 ( GPIO_PIN_3 )

#ifdef REVA1 //Rev.A1 uses SF23 cc2538 which start at diffferent location
    #define CC2538_FLASH_ADDRESS            ( 0x0023F800 )
#else
    #define CC2538_FLASH_ADDRESS            ( 0x0027F800 )
#endif
//=========================== prototypes ======================================

void board_timer_init(void);
uint32_t board_timer_get(void);
bool board_timer_expired(uint32_t future);

static void clock_init(void);
static void gpio_init(void);
static void button_init(void);

void precision_timers_init(void);
void input_edge_timers_init(void);
static void mattress_init(void);

/* Hardware constants. */
static const uint32_t gptmEdgeTimerBase = GPTIMER3_BASE;
static const uint32_t gptmFallingEdgeInt = INT_TIMER3B;
static const uint32_t gptmFallingEdgeEvent = GPTIMER_CAPB_EVENT;

static const uint32_t gptmTimer3AReg = 0x40033048;
static const uint32_t gptmTimer3BReg = 0x4003304C;

static const uint32_t timer_cnt_32 = 0xFFFFFFFF;
static const uint32_t timer_cnt_16 = 0xFFFF;
static const uint32_t timer_cnt_24 = 0xFFFFFF;

/* Definition of global variables for mattress calibration. */
float azimuth = -1;
float elevation = -1;
float roll_calib = -1;
float pitch_calib = -1;

static const float sweep_velocity = PI / SWEEP_PERIOD_US;

volatile pulse_t pulses[PULSE_TRACK_COUNT];
volatile uint8_t modular_ptr;
volatile uint8_t pulse_count;

uint32_t test_count;

static void SysCtrlDeepSleepSetting(void);
static void SysCtrlSleepSetting(void);
static void SysCtrlRunSetting(void);
static void SysCtrlWakeupSetting(void);

static void GPIO_C_Handler(void);
void mimsy_GPIO_falling_edge_handler(void);

bool user_button_initialized;

//=========================== main ============================================

extern int mote_main(void);

int main(void) {
   return mote_main();
}

//=========================== public ==========================================

void board_init(void) {
   user_button_initialized = FALSE;
   
   gpio_init();
   clock_init();
   board_timer_init(); //changed for inchwrom code
   leds_init(); //changed for incworm code
   debugpins_init(); //changed for inchworm code
   //button_init();
   sctimer_init(); //changed for inchworm code
   uart_init(); //changed for inchwrom code

   mattress_init();

   radio_init(); //changed for incwhorm code
   i2c_init(); //changed for inchworm code
  // sensors_init();
  // cryptoengine_init();
}

/**
 * Puts the board to sleep
 */
void board_sleep(void) {
    SysCtrlPowerModeSet(SYS_CTRL_PM_NOACTION);
    SysCtrlSleep();
}

void board_deep_sleep(void) {
  SysCtrlPowerModeSet(SYS_CTRL_PM_2);
  SysCtrlDeepSleep();
}

/**
 * Timer runs at 32 MHz and is 32-bit wide
 * The timer is divided by 32, whichs gives a 1 microsecond ticks
 */
void board_timer_init(void) {
    // Configure the timer
    TimerConfigure(GPTIMER2_BASE, GPTIMER_CFG_PERIODIC_UP);
    
    // Enable the timer
    TimerEnable(GPTIMER2_BASE, GPTIMER_BOTH);
}

/**
 * Returns the current value of the timer
 * The timer is divided by 32, whichs gives a 1 microsecond ticks
 */
uint32_t board_timer_get(void) {
    uint32_t current;
    
    current = TimerValueGet(GPTIMER2_BASE, GPTIMER_A) >> 5;
    
    return current;
}

/**
 * Returns true if the timer has expired
 * The timer is divided by 32, whichs gives a 1 microsecond ticks
 */
bool board_timer_expired(uint32_t future) {
    uint32_t current;
    int32_t remaining;

    current = TimerValueGet(GPTIMER2_BASE, GPTIMER_A) >> 5;

    remaining = (int32_t) (future - current);
    
    if (remaining > 0) {
        return false;
    } else {
        return true;
    }
}

/**
 * Resets the board
 */
void board_reset(void) {
    SysCtrlReset();
}

//=========================== private =========================================

static void gpio_init(void) {
    /* Set GPIOs as output */
    GPIOPinTypeGPIOOutput(GPIO_A_BASE, 0xFF);
    GPIOPinTypeGPIOOutput(GPIO_B_BASE, 0xFF);
    GPIOPinTypeGPIOOutput(GPIO_C_BASE, 0xFF);
    GPIOPinTypeGPIOOutput(GPIO_D_BASE, 0xFF);

    /* Initialize GPIOs to low */
    GPIOPinWrite(GPIO_A_BASE, 0xFF, 0x00);
    GPIOPinWrite(GPIO_B_BASE, 0xFF, 0x00);
    GPIOPinWrite(GPIO_C_BASE, 0xFF, 0x00);
    GPIOPinWrite(GPIO_D_BASE, 0xFF, 0x00);
}

static void clock_init(void) {
    /* Disable global interrupts */
    bool bIntDisabled = IntMasterDisable();

    /* Configure the 32 kHz pins, PD6 and PD7, for crystal operation */
    /* By default they are configured as GPIOs */
    GPIODirModeSet(GPIO_D_BASE, 0x40, GPIO_DIR_MODE_IN);
    GPIODirModeSet(GPIO_D_BASE, 0x80, GPIO_DIR_MODE_IN);
    IOCPadConfigSet(GPIO_D_BASE, 0x40, IOC_OVERRIDE_ANA);
    IOCPadConfigSet(GPIO_D_BASE, 0x80, IOC_OVERRIDE_ANA);

    /* Set the real-time clock to use the 32 kHz external crystal */
    /* Set the system clock to use the external 32 MHz crystal */
    /* Set the system clock to 32 MHz */
    SysCtrlClockSet(true, false, SYS_CTRL_SYSDIV_32MHZ);

    /* Set the IO clock to operate at 16 MHz */
    /* This way peripherals can run while the system clock is gated */
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_16MHZ);

    /* Wait until the selected clock configuration is stable */
    while (!((HWREG(SYS_CTRL_CLOCK_STA)) & (SYS_CTRL_CLOCK_STA_XOSC_STB)));

    /* Define what peripherals run in each mode */
    SysCtrlRunSetting();
    SysCtrlSleepSetting();
    SysCtrlDeepSleepSetting();
    SysCtrlWakeupSetting();
    /* Re-enable interrupt if initially enabled */
    if (!bIntDisabled) {
        IntMasterEnable();
    }
}

/**
 * Configures the user button as input source
 */
static void button_init(void) {
    volatile uint32_t i;

    /* Delay to avoid pin floating problems */
    for (i = 0xFFFF; i != 0; i--);

    GPIOPinIntDisable(BSP_BUTTON_BASE, BSP_BUTTON_USER);
    GPIOPinIntClear(BSP_BUTTON_BASE, BSP_BUTTON_USER);

    /* The button is an input GPIO on falling edge */
    GPIOPinTypeGPIOInput(BSP_BUTTON_BASE, BSP_BUTTON_USER);
    GPIOIntTypeSet(BSP_BUTTON_BASE, BSP_BUTTON_USER, GPIO_FALLING_EDGE);

    /* Register the interrupt */
    GPIOPortIntRegister(BSP_BUTTON_BASE, GPIO_C_Handler);

    /* Clear and enable the interrupt */
    GPIOPinIntClear(BSP_BUTTON_BASE, BSP_BUTTON_USER);
    GPIOPinIntEnable(BSP_BUTTON_BASE, BSP_BUTTON_USER);
    user_button_initialized = TRUE;
}

static void SysCtrlRunSetting(void) {
  /* Disable General Purpose Timers 0, 1, 2, 3 when running */
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_GPT0);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_GPT1);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_GPT3);

  /* Disable SSI 0, 1 when running */
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_SSI0);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_SSI1);

  /* Disable UART1 when running */
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_UART1);

  /* Disable I2C, AES and PKA when running */
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_I2C);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_PKA);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_AES);

  /* Enable UART0 and RFC when running */
  SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT2);
  SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_UART0);
  SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_RFC);
}

static void SysCtrlSleepSetting(void) {
  /* Disable General Purpose Timers 0, 1, 2, 3 during sleep */
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_GPT0);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_GPT1);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_GPT3);

  /* Disable SSI 0, 1 during sleep */
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_SSI0);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_SSI1);

  /* Disable UART 0, 1 during sleep */
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_UART1);

  /* Disable I2C, PKA, AES during sleep */
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_I2C);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_PKA);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_AES);

  /* Enable UART and RFC during sleep */
  SysCtrlPeripheralSleepEnable(SYS_CTRL_PERIPH_GPT2);
  SysCtrlPeripheralSleepEnable(SYS_CTRL_PERIPH_UART0);
  SysCtrlPeripheralSleepEnable(SYS_CTRL_PERIPH_RFC);
}

static void SysCtrlDeepSleepSetting(void) {
  /* Disable General Purpose Timers 0, 1, 2, 3 during deep sleep */
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_GPT0);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_GPT1);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_GPT2);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_GPT3);

  /* Disable SSI 0, 1 during deep sleep */
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_SSI0);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_SSI1);

  /* Disable UART 0, 1 during deep sleep */
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_UART0);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_UART1);

  /* Disable I2C, PKA, AES during deep sleep */
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_I2C);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_PKA);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_AES);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_RFC);
}

static void SysCtrlWakeupSetting(void) {
  /* Allow the SMTimer to wake up the processor */
  GPIOIntWakeupEnable(GPIO_IWE_SM_TIMER);
}

//========================== mattress calibration =============================

void precision_timers_init(void){
    // SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT0); // enables timer0 module
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT3); // enables timer3 module

    input_edge_timers_init();

    // TimerConfigure(gptmPeriodTimerBase, GPTIMER_CFG_PERIODIC_UP);
    // TimerLoadSet(gptmPeriodTimerBase,GPTIMER_A,timer_cnt_32);
    // TimerEnable(gptmPeriodTimerBase,GPTIMER_A);
}

void input_edge_timers_init(void) {
    GPIOPinTypeTimer(GPIO_A_BASE,GPIO_PIN_2); // enables hw muxing of pin inputs
    GPIOPinTypeTimer(GPIO_A_BASE,GPIO_PIN_5); // enables hw muxing of pin inputs

    TimerConfigure(gptmEdgeTimerBase, GPTIMER_CFG_SPLIT_PAIR |
          GPTIMER_CFG_A_CAP_TIME_UP | GPTIMER_CFG_B_CAP_TIME_UP); // configures timer3a/b as 16-bit edge timers

    TimerPrescaleSet(gptmEdgeTimerBase,GPTIMER_A,0); // add prescaler to timer3a (24-bit)
    TimerPrescaleSet(gptmEdgeTimerBase,GPTIMER_B,0); // add prescaler to timer3b (24-bit)

    TimerLoadSet(gptmEdgeTimerBase,GPTIMER_A,timer_cnt_16);
    TimerLoadSet(gptmEdgeTimerBase,GPTIMER_B,timer_cnt_16);

    // FIXME: can we use the same gpio pin for both??
    IOCPinConfigPeriphInput(GPIO_A_BASE, GPIO_PIN_2, IOC_GPT3OCP1); // map gpio pin output to timer3a
    IOCPinConfigPeriphInput(GPIO_A_BASE, GPIO_PIN_5, IOC_GPT3OCP2); // map gpio pin output to timer3b

    // NOTE: the TS3633-CM1 Prototyping Module inverts rising and falling edges when light pulses are received,
    // so negative edges correspond to rising edges, and positive edges correspond to falling edges
    TimerControlEvent(gptmEdgeTimerBase, GPTIMER_A, GPTIMER_EVENT_NEG_EDGE); // set timer3a to capture rising edges (inverted by PCB)
    TimerControlEvent(gptmEdgeTimerBase, GPTIMER_B, GPTIMER_EVENT_POS_EDGE); // set timer3b to capture falling edges (inverted by PCB)

    TimerIntDisable(gptmEdgeTimerBase, gptmFallingEdgeEvent);
    TimerIntClear(gptmEdgeTimerBase, gptmFallingEdgeEvent);

    // set up interrupt for falling edge timer
    TimerIntRegister(gptmEdgeTimerBase, GPTIMER_B, mimsy_GPIO_falling_edge_handler);
    TimerIntEnable(gptmEdgeTimerBase, gptmFallingEdgeEvent);
    IntPrioritySet(gptmFallingEdgeInt, 0<<5);
    IntEnable(gptmFallingEdgeInt);

    IntMasterEnable();

    TimerEnable(gptmEdgeTimerBase,GPTIMER_BOTH);

    TimerSynchronize(gptmEdgeTimerBase, GPTIMER_3A_SYNC | GPTIMER_3B_SYNC);
}

float get_period_us_32(uint32_t start, uint32_t end) {
    if (start > end) {
        // do overflow arithmetic
        return ((float) (end + (timer_cnt_32 - start))) / CLOCK_SPEED_MHZ;
    } else {
        return ((float) (end - start)) / CLOCK_SPEED_MHZ;
    }
}

float get_period_us(uint32_t start, uint32_t end) {
    if (start > end) {
        // do overflow arithmetic
        return ((float) (end + (timer_cnt_24 - start))) / CLOCK_SPEED_MHZ;
    } else {
        return ((float) (end - start)) / CLOCK_SPEED_MHZ;
    }
}

/** Returns a number defining our 3 information bits: skip, data, axis.
  Given by our pulse length in microseconds (us). */
unsigned short int sync_bits(float duration) {
  return (unsigned short int) (48*duration - 2501) / 500;
}

float distance_fit_horiz(float time_us) {
  float E = 0.2218; float c0 = -0.3024; float c1 = 18.2991;
  return E + c1 / (time_us - c0 * sweep_velocity);
}

float distance_fit_vert(float time_us) {
  float E = 0.3074; float c0 = 0.9001; float c1 = 16.1908;
  return E + c1 / (time_us - c0 * sweep_velocity);
}

location_t localize_mimsy(pulse_t *pulses_local) {
    location_t loc = (location_t){.phi = 0, .theta = 0,
											.r_vert = 0, .r_horiz = 0,
											.asn =  {0, 0, 0, 0, 0}, .valid = 0};

    uint8_t init_sync_index = PULSE_TRACK_COUNT;

    // loop through and classify our pulses
    Pulses valid_seq_a[4] = { Sync, Horiz, Sync, Vert };
    Pulses valid_seq_b[4] = { Sync, Vert, Sync, Horiz };
    uint8_t sweep_axes_check = 0; uint8_t i;
    for (i = 0; i < PULSE_TRACK_COUNT; i++) {
        float period = get_period_us(pulses_local[i].rise, pulses_local[i].fall);
        if (period < MIN_SYNC_PERIOD_US) { // sweep pulse
            if (init_sync_index != PULSE_TRACK_COUNT) {
                float parent_period = get_period_us(pulses_local[i-1].rise, pulses_local[i-1].fall);
                int axis = (sync_bits(parent_period) & 0b001) + 1;
                pulses_local[i].type = axis; // 2 if horizontal, 1 if vertical

                int ind = i - init_sync_index;
                if (axis == ((int) valid_seq_a[ind]) || axis == ((int) valid_seq_b[ind])) {
                    sweep_axes_check += axis; // check for 1 horizontal, 1 vertical sweep
                } else {
                    return loc;
                }
            }
        } else if (period < MAX_SYNC_PERIOD_US) { // sync pulse
            if (init_sync_index == PULSE_TRACK_COUNT) {
            	init_sync_index = i; // set initial valid sync pulse index
            }
            pulses_local[i].type = (int) Sync;
        } else { // neither
            pulses_local[i].type = -1;
            return loc;
        }
    }

    if (init_sync_index == PULSE_TRACK_COUNT || sweep_axes_check != 3) return loc;

    for (i = init_sync_index; i < PULSE_TRACK_COUNT-1; i++) {
        pulse_t curr_pulse = pulses_local[i];
        pulse_t next_pulse = pulses_local[i+1];

        switch(next_pulse.type) {
            case ((int) Sync):
                break;
            case ((int) Horiz):
                loc.phi = get_period_us(curr_pulse.fall, next_pulse.rise) * sweep_velocity;
                loc.r_horiz = distance_fit_horiz(get_period_us(next_pulse.rise, next_pulse.fall));
                break;
            case ((int) Vert):
                loc.theta = get_period_us(curr_pulse.fall, next_pulse.rise) * sweep_velocity;
                loc.r_vert = distance_fit_vert(get_period_us(next_pulse.rise, next_pulse.fall));
                break;
            default:
                return loc;
                break;
        }
    }

    loc.valid = true;
    return loc;
}

static void mattress_init(void) {
    // localize and store
    modular_ptr = 0; pulse_count = 0; test_count = 0;
    // initialize edges
    unsigned short int i;
    for (i = 0; i < PULSE_TRACK_COUNT; i++) {
        pulses[i] = (pulse_t){.rise = 0, .fall = 0, .type = -1};
    }

    volatile uint32_t _i;
    //Delay to avoid pin floating problems
    for (_i = 0xFFFF; _i != 0; _i--);
    
    precision_timers_init();

    float samples = 0;
    float horiz_t = 0; float vert_t = 0; float roll_c = 0; float pitch_c = 0;

    while (true) {
        if (pulse_count >= 5) {
            IntMasterDisable(); // temporarily disable interrupts

            // get pose
            pulse_t pulses_local[PULSE_TRACK_COUNT];
            uint8_t ptr = modular_ptr;

            unsigned short int i;
            for (i = ptr; i < ptr + PULSE_TRACK_COUNT; i++) {
                pulses_local[i-ptr].rise = pulses[i%PULSE_TRACK_COUNT].rise;
                pulses_local[i-ptr].fall = pulses[i%PULSE_TRACK_COUNT].fall;
                pulses_local[i-ptr].type = pulses[i%PULSE_TRACK_COUNT].type;
            }

            pulse_count = 0;
            IntMasterEnable(); // re-enable interrupts

            // recover azimuth and elevation
            location_t loc = localize_mimsy(pulses_local);
            if (!loc.valid) { continue; }

            samples += 1;
            horiz_t += loc.phi; vert_t += loc.theta; // TODO: IMU data

            if (samples >= 1) {
                break;
            }
        }
        // wait for localization data
        // whenever new localization data is ready, get IMU data (not necessary for now)
        // after k samples, take average time difference, break

        // get it to work, then make it perfect

        test_count += 1;
    }

    azimuth = horiz_t / samples; elevation = vert_t / samples; // TODO: IMU data
    TimerIntDisable(gptmEdgeTimerBase, gptmFallingEdgeEvent);
    TimerIntClear(gptmEdgeTimerBase, gptmFallingEdgeEvent);
    IntDisable(gptmFallingEdgeInt);

    TimerDisable(gptmEdgeTimerBase,GPTIMER_BOTH);

    // set extern variables, return, complete board init
    
}

//=========================== interrupt handlers ==============================

/**
 * GPIO_C interrupt handler. User button is GPIO_C_3
 * Erases a Flash sector to trigger the bootloader backdoor
 */
static void GPIO_C_Handler(void) {
    if (!user_button_initialized) return;
    /* Disable the interrupts */
    IntMasterDisable();
    leds_all_off();

    /* Eras the CCA flash page */
    FlashMainPageErase(CC2538_FLASH_ADDRESS);

    leds_circular_shift();
    
    /* Reset the board */
    SysCtrlReset();
}

void mimsy_GPIO_falling_edge_handler(void) {
    TimerIntClear(gptmEdgeTimerBase, gptmFallingEdgeEvent);

    pulses[modular_ptr].rise = (uint32_t)(HWREG(gptmTimer3AReg));
    pulses[modular_ptr].fall = (uint32_t)(HWREG(gptmTimer3BReg));
    modular_ptr++; if (modular_ptr == 5) modular_ptr = 0;

    pulse_count += 1;
}
