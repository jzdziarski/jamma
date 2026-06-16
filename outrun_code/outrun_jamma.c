/*
 * @file outrun_jamma.c
 * @brief Sega Outrun Rev. B to JAMMA Adapter Firmware
 * 
 * This firmware controls the ATMEGA328P microcontroller to interface
 * between JAMMA inputs and analog voltage outputs via MCP4902 DAC.
 * 
 * Pin Assignments:
 * Inputs (active-low with internal pull-ups):
 *   PD2 (Pin 4)  - B1 (Accelerate), enabled adjustable VOUTA
 *   PD3 (Pin 5)  - B2 (Shift toggle)
 *   PD4 (Pin 6)  - B3 (Brake), digital H/L output
 *   PD5 (Pin 11) - UP, Increases accelerator voltage
 *   PD6 (Pin 12) - DOWN, Decreases accelerator voltage
 *   PD7 (Pin 13) - LEFT, Decreases steering voltage
 *   PB0 (Pin 14) - RIGHT, Increases steering voltage
 * 
 * Outputs:
 *   PB1 (Pin 15) - B3_OUT (Brake, HIGH or LOW)
 *   PB2 (Pin 16) - B2_OUT (Shift output, HIGH or LOW, Latched)
 *   PB3 (Pin 17) - MCP4902 CS
 *   PB4 (Pin 18) - MCP4902 SDI
 *   PB5 (Pin 19) - MCP4902 SCK
 * 
 * DAC Outputs:
 *   VOUTA (Pin 14 on MCP4902), Accelerator voltage
 *   VOUTB (Pin 10 on MCP4902), Steering voltage
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "outrun_jamma.h"

#define DEBOUNCE_DELAY_MS       20
#define BUTTON_REPEAT_DELAY_MS  300
#define BUTTON_REPEAT_RATE_MS   31      /* ~2s full sweep: 300ms delay + 54 steps * 31ms */

/* Steering-specific button repeat timing for fast response */
#define STEERING_BUTTON_REPEAT_DELAY_MS   0     /* Initial Steering Response */
#define STEERING_BUTTON_REPEAT_RATE_MS    0     /* Continuous Adjustment */

/*
 * Accelerator Pedal DAC Constants
 * Voltage step size (~19.6mV per step)
 * From manual: Valid pedal ranges 32-144
 */

#define ACCEL_ADJUST_STEP         2    /* ~98mV per step */
#define ACCEL_MIN_VALUE           32
#define ACCEL_MAX_VALUE           144

/*
 * Steering DAC Constants - Game-recognized range (51-203), Zeroed @ 128
 * Voltage step size (~19.6mV per step)
 */

#define STEERING_ADJUST_STEP      2     /* ~39mV per step for smooth steering */
#define STEERING_MIN_VALUE        51
#define STEERING_MAX_VALUE        203
#define STEERING_MID_VALUE  	  128
#define STEERING_RETURN_DIVISOR   16    /* Proportional return: step = distance/16
                                         * (4 steps at full lock, 1 step near center) */

/* Loop Timing; MCP4902 settling time is 4.5us, 20us safe for smooth updates */
#define LOOP_TIME_MS    0.05
#define MS_TO_LOOPS(ms) ((uint32_t)((ms)/LOOP_TIME_MS))

/*
 * Pin Definitions
 */

/* Input pins, Active-Low */
#define B1_PIN_BIT              PD2     // Accelerate     - Pin 4  (INT0)
#define B2_PIN_BIT              PD3     // Shift toggle   - Pin 5  (INT1)
#define B3_PIN_BIT              PD4     // Brake          - Pin 6  (XCK/T0)
#define UP_PIN_BIT              PD5     // Increase accel - Pin 11 (T1)
#define DOWN_PIN_BIT            PD6     // Decrease accel - Pin 12 (AIN0)
#define LEFT_PIN_BIT            PD7     // Decrease steer - Pin 13 (AIN1)
#define RIGHT_PIN_BIT           PB0     // Increase steer - Pin 14 (ICP1)

/* Output pins */
#define B3_OUT_PIN_BIT          PB1     // Brake output - Pin 15 (OC1A)
#define B2_OUT_PIN_BIT          PB2     // Shift output - Pin 16 (SS/OC1B)
#define MCP_CS_PIN_BIT          PB3     // DAC CS  - Pin 17 to MCP4902 Pin 3
#define MCP_SD_PIN_BIT          PB4     // DAC SDI - Pin 18 to MCP4902 Pin 5
#define MCP_SCK_PIN_BIT         PB5     // DAC SCK - Pin 19 to MCP4902 Pin 4

/* Globals */
static uint8_t accel_voltage_value    = ACCEL_MAX_VALUE;
static uint8_t steering_voltage_value = STEERING_MID_VALUE;
static uint8_t b2_toggle_state        = 1;                  // Shift latch (L)
static uint8_t brake_pressed_state    = 0;                  // Brake (L)

/* Globals for button repeat timing */
static uint32_t up_last_press_time    = 0;
static uint32_t down_last_press_time  = 0;
static uint32_t left_last_press_time  = 0;
static uint32_t right_last_press_time = 0;

/* Accelerator ramp target: full throttle on B1 press, idle on release.
 * Adjustable via UP/DOWN while B1 is held to set a partial-throttle setpoint. */
static uint8_t accel_ramp_target      = ACCEL_MIN_VALUE;

/* Steering button state tracking for separate repeat timing */
static uint8_t left_steering_pressed  = 0;
static uint8_t right_steering_pressed = 0;

static volatile uint32_t loop_counter = 0;

int main(void)
{
    /* Power-on delay, ensure JAMMA power rails are stabile */
    _delay_ms(500);
    
    system_init();
    
    /* Initialize DAC outputs */
    update_accel_voltage(accel_voltage_value);
    update_steering_voltage(steering_voltage_value);
    
    /* Main event loop */
    while (1) {
        update_outputs();
        _delay_us(50);
        loop_counter++;
    }
    
    return 0;
}

void system_init(void)
{
    gpio_init();
    spi_init();
    
    /* Disable global interrupts (Unnecessary) */
    cli();
}

/*
 * Initialize GPIO Pins
 * Configures all input and output pins.
 * Inputs use internal pull-ups for active-low logic.
 * Outputs are configured as push-pull digital.
 * SPI uses PB3=CS, PB4=SDI, PB5=SCK
 */

void gpio_init(void)
{
    /* Configure input pins with internal pull-ups for active-low */
    DDRD &= ~(1 << PD2);  // B1 (Accelerate)   - Pin 4
    DDRD &= ~(1 << PD3);  // B2 (Shift toggle) - Pin 5
    DDRD &= ~(1 << PD4);  // B3 (Brake)        - Pin 6
    DDRD &= ~(1 << PD5);  // UP                - Pin 11
    DDRD &= ~(1 << PD6);  // DOWN              - Pin 12
    DDRD &= ~(1 << PD7);  // LEFT              - Pin 13
    DDRB &= ~(1 << PB0);  // RIGHT             - Pin 14
    
    /* Enable pull-ups */
    PORTD |= (1 << PD2);  // B1
    PORTD |= (1 << PD3);  // B2
    PORTD |= (1 << PD4);  // B3
    PORTD |= (1 << PD5);  // UP
    PORTD |= (1 << PD6);  // DOWN
    PORTD |= (1 << PD7);  // LEFT
    PORTB |= (1 << PB0);  // RIGHT
    
    /* Configure output pins as push-pull digital */
    DDRB |= (1 << PB1);   // B3_OUT (Brake) - Pin 15
    PORTB &= ~(1 << PB1); // LOW (Default State)
    
    DDRB |= (1 << PB2);   // B2_OUT (Shift) - Pin 16
    PORTB &= ~(1 << PB2); // LOW (Default State)
    
    /* Configure SPI pins for MCP4902 */
    DDRB |= (1 << PB3);   // CS
    PORTB |= (1 << PB3);  // CS HIGH (Initially deselect DAC)
    
    DDRB |= (1 << PB4);   // SDI, Output for SPI Data
    PORTB &= ~(1 << PB4); // LOW (Default State)
    
    DDRB |= (1 << PB5);   // SCK, Output for SPI Clock
    PORTB &= ~(1 << PB5); // LOW (Default State)
}

/* Disables hardware SPI in favor of PB3/PB4/PB5 */

void spi_init(void)
{
    SPCR = 0;
    SPSR = 0;
}

/*
 * MCP4902 DAC Write Function
 * Sends command and 8-bit data to MCP4902 via SW SPI.
 * PB3=CS, PB4=SDI, PB5=SCK
 * 
 * @param CH 0 for VOUTA, CH 1 for VOUTB
 * @param data 8-bit DAC value (0-255)
 */

void mcp4902_write(uint8_t channel, uint8_t data)
{
    uint16_t command_word;
    uint8_t i;
    
    if (channel == 0) // VOUTA
    {
        command_word = 0x3000 | (data << 4);
    } else            // VOUTB
    {
        command_word = 0xB000 | (data << 4);
    }
    
    /* Assert CS */
    PORTB &= ~(1 << MCP_CS_PIN_BIT);
    
    /* Send 16 bits MSB first */
    for (i = 0; i < 16; i++)
    {
        /* Set SDI to current bit */
        if (command_word & 0x8000)
        {
            PORTB |= (1 << MCP_SD_PIN_BIT);   // HIGH
        } else
        {
            PORTB &= ~(1 << MCP_SD_PIN_BIT);  // LOW
        }
        
        /* Pulse SCK on rising edge */
        PORTB |= (1 << MCP_SCK_PIN_BIT);
        
        /* Stabilize signal */
        _delay_us(1);
        
        /* Falling edge of SCK */
        PORTB &= ~(1 << MCP_SCK_PIN_BIT);
        
        /* Shift to next bit */
        command_word <<= 1;
    }
    
    /* Deassert CS */
    PORTB |= (1 << MCP_CS_PIN_BIT);
}

/*
 * Update Accelerator Pedal Voltage (VOUTA)
 * Writes to the DAC CH A output with the specified value (0-5v) (0-255).
 * When B1 is pressed, current accelerator value is written
 * When B1 not pressed, ACCEL_MIN_VALUE is written
 *
 * @param value 8-bit DAC value (0-255)
 */

void update_accel_voltage(uint8_t value)
{
    if (value > ACCEL_MAX_VALUE)
    {
        value = ACCEL_MAX_VALUE;
    }
    
    accel_voltage_value = value;
    
    /* Write to MCP4902 CH 0 (VOUTA) */
    mcp4902_write(0, value);
}

/*
 * Update Steering Voltage (VOUTB)
 * Writes to the DAC CH B output with the specified value, 0-5v (0-255).
 * When neither L/R are pressed, STEERING_MID_VALUE is written 
 *
 * @param value 8-bit DAC value (0-255)
 */

void update_steering_voltage(uint8_t value) {
    if (value > STEERING_MAX_VALUE)
    {
        value = STEERING_MAX_VALUE;
    }
    
    steering_voltage_value = value;
    
    /* Write to MCP4902 CH 1 (VOUTB) */
    mcp4902_write(1, value);
}

/*
 * Read Button State
 * Reads a button input and returns 1 if pressed (active-low), 0 otherwise.
 * 
 * @param port Pointer to port register (PIND or PINB)
 * @param bit  Bit position within the port
 * @return 1 if button pressed, 0 if not pressed
 */

uint8_t read_button(volatile uint8_t *port, uint8_t bit)
{
    return !(*port & (1 << bit));
}

/*
 * Set Brake Output State
 * Sets the brake output pin (PB1) to HIGH or LOW.
 * This is a PUSH-PULL digital output to drive 5v and 0v directly.
 * 
 * NOTE: This would normally be DAC values 32 or 208,8
 *       but we don't have a third DAC channel.
 *
 * @param state 1 for HIGH (5V), 0 for LOW (0V)
 */

void set_brake_output(uint8_t state)
{
    if (state)
    {
        PORTB |= (1 << B3_OUT_PIN_BIT);   // HIGH (5v)
    } else
    {
        PORTB &= ~(1 << B3_OUT_PIN_BIT);  // LOW  (0v)
    }
}

/*
 * Set Shift Output State
 * Sets the shift output pin (PB2) to HIGH or LOW.
 * This is a PUSH-PULL digital output to drive 5v and 0v directly.
 * 
 * @param state 1 for HIGH (5V), 0 for LOW (0V)
 */

void set_shift_output(uint8_t state)
{
    if (state)
    {
        PORTB |= (1 << B2_OUT_PIN_BIT);   // HIGH (5v)
    } else
    {
        PORTB &= ~(1 << B2_OUT_PIN_BIT);  // LOW  (0v)
    }
}

/*
 * Update All Outputs
 * Main output update function called in the main loop. Handles:
 * - B1 (Accelerate): Enables VOUTA, UP/DOWN adjust voltage when pressed
 * - B2 (Shift): Single digital output PB2, HIGH or LOW based on toggle
 * - B3 (Brake): Digital output PB1, HIGH when pressed, LOW when not
 * - LEFT/RIGHT: Adjusts steering voltage (VOUTB)
 * 
 * Implements debouncing and button repeat functionality
 */

void update_outputs(void)
{
    static uint8_t b1_state = 0;
    
    /* B1: Accelerate Button, enabled VOUTA voltage */

    if (read_button(&PIND, B1_PIN_BIT))
    { 
        if (! b1_state)
        {
            b1_state = 1;

            /* Ramp target is full throttle when pedal is pressed */
            accel_ramp_target = ACCEL_MAX_VALUE;
        }
    } else {
        if (b1_state)
        {
            b1_state = 0;

            /* Ramp down to minimum when pedal released */
            accel_ramp_target = ACCEL_MIN_VALUE;
        } else {
            update_accel_voltage(ACCEL_MIN_VALUE);
        }
    }

    /* Accelerator ramp smoothing: tracks accel_ramp_target at 2 steps/loop
     * (~196mV/ms) in either direction, so UP/DOWN setpoint changes while
     * B1 is held are followed smoothly rather than overridden. */
    if (accel_voltage_value < accel_ramp_target)
    {
        accel_voltage_value += 2;
        if (accel_voltage_value > accel_ramp_target)
        {
            accel_voltage_value = accel_ramp_target;
        }

        update_accel_voltage(accel_voltage_value);

    } else if (accel_voltage_value > accel_ramp_target)
    {
        accel_voltage_value -= 2;
        if (accel_voltage_value < accel_ramp_target) {
            accel_voltage_value = accel_ramp_target;
        }
        update_accel_voltage(accel_voltage_value);
    }

    /* B2: Shift Toggle, Digital Output, Maintains state internally */

    static uint8_t b2_confirmed_state = 0;
    static uint8_t b2_raw_last = 0;
    static uint32_t b2_debounce_counter = 0;
    uint8_t b2_current = read_button(&PIND, B2_PIN_BIT);
    
    /* Debounce Detection */
    if (b2_current != b2_raw_last) {
        b2_debounce_counter = 0;
        b2_raw_last = b2_current;
    } else {
        b2_debounce_counter++;
    }

    if (b2_debounce_counter == MS_TO_LOOPS(DEBOUNCE_DELAY_MS)) {
        if (b2_current != b2_confirmed_state) {
            b2_confirmed_state = b2_current;
            if (b2_current == 1) {  // Only toggle on press, not release
                b2_toggle_state = !b2_toggle_state;
                set_shift_output(b2_toggle_state);
            }
        }
    }
    
    /* B3: Brake, Digital Output */

    if (read_button(&PIND, B3_PIN_BIT))
    {
        if (! brake_pressed_state)
        {
            brake_pressed_state = 1;
            set_brake_output(1);  // HIGH (5v)
        }
    } else
    {
        if (brake_pressed_state) {
            brake_pressed_state = 0;
            set_brake_output(0);  // LOW (0v)
        }
    }
    
    /* UP/DOWN: Adjust Accelerator Voltage when B1 is Pressed */
    
    static uint8_t up_pressed = 0;
    static uint8_t down_pressed = 0;
    static uint8_t up_repeating = 0;
    static uint8_t down_repeating = 0;
    
    /* UP: Increase accelerator voltage */

    if (read_button(&PIND, UP_PIN_BIT))
    {
        if (! up_pressed)
        {
            if (b1_state)
            {
                accel_ramp_target += ACCEL_ADJUST_STEP;
                if (accel_ramp_target > ACCEL_MAX_VALUE)
                {
                    accel_ramp_target = ACCEL_MAX_VALUE;
                }
            }
            up_pressed = 1;
            up_repeating = 0;
            up_last_press_time = loop_counter;
        } else if (b1_state)
        {
            uint32_t up_threshold = up_repeating ? MS_TO_LOOPS(BUTTON_REPEAT_RATE_MS)
                                                  : MS_TO_LOOPS(BUTTON_REPEAT_DELAY_MS);

            if (loop_counter - up_last_press_time >= up_threshold)
            {
                accel_ramp_target += ACCEL_ADJUST_STEP;
                up_last_press_time = loop_counter;
                up_repeating = 1;

                if (accel_ramp_target > ACCEL_MAX_VALUE)
                {
                    accel_ramp_target = ACCEL_MAX_VALUE;
                }
            }
        }
    } else {
        up_pressed = 0;
        up_repeating = 0;
    }
    
    /* DOWN: Decrease accelerator voltage */

    if (read_button(&PIND, DOWN_PIN_BIT))
    {
        if (! down_pressed)
        {
            if (b1_state)
            {
                if (accel_ramp_target >= ACCEL_MIN_VALUE + ACCEL_ADJUST_STEP)
                {
                    accel_ramp_target -= ACCEL_ADJUST_STEP;
                } else {
                    accel_ramp_target = ACCEL_MIN_VALUE;
                }
            }
            down_pressed = 1;
            down_repeating = 0;
            down_last_press_time = loop_counter;
        } else if (b1_state)
        {
            uint32_t down_threshold = down_repeating ? MS_TO_LOOPS(BUTTON_REPEAT_RATE_MS)
                                                       : MS_TO_LOOPS(BUTTON_REPEAT_DELAY_MS);

            if (loop_counter - down_last_press_time >= down_threshold)
            {
                down_last_press_time = loop_counter;
                down_repeating = 1;

                if (accel_ramp_target >= ACCEL_MIN_VALUE + ACCEL_ADJUST_STEP)
                {
                    accel_ramp_target -= ACCEL_ADJUST_STEP;
                } else {
                    accel_ramp_target = ACCEL_MIN_VALUE;
                }
            }
        }
    } else {
        down_pressed = 0;
        down_repeating = 0;
    }
    
    /* Steering Voltage (VOUTB) */
    
    static uint8_t left_pressed = 0;
    static uint8_t right_pressed = 0;
    
    /* RIGHT: Increase steering voltage toward STEERING_MAX_VALUE */
    if (read_button(&PINB, RIGHT_PIN_BIT))
    {
        right_steering_pressed = 1;

        if (! right_pressed) {
            steering_voltage_value += STEERING_ADJUST_STEP;
            if (steering_voltage_value > STEERING_MAX_VALUE)
            {
                steering_voltage_value = STEERING_MAX_VALUE;
            }
            update_steering_voltage(steering_voltage_value);
            right_pressed = 1;
            right_last_press_time = loop_counter;
        } else if ((loop_counter - right_last_press_time
                    >= MS_TO_LOOPS(STEERING_BUTTON_REPEAT_DELAY_MS)))
        {
            if (loop_counter - right_last_press_time >= MS_TO_LOOPS(STEERING_BUTTON_REPEAT_RATE_MS))
            {
                steering_voltage_value += STEERING_ADJUST_STEP;
                right_last_press_time = loop_counter;
                if (steering_voltage_value > STEERING_MAX_VALUE)
                {
                    steering_voltage_value = STEERING_MAX_VALUE;
                }
                update_steering_voltage(steering_voltage_value);
            }
        }
    } else {
        right_pressed = 0;
        right_steering_pressed = 0;
    }

    /* LEFT: Decrease steering voltage toward STEERING_MIN_VALUE */
    if (read_button(&PIND, LEFT_PIN_BIT))
    {
        left_steering_pressed = 1;

        if (! left_pressed)
        {
            if (steering_voltage_value >= 
                STEERING_MIN_VALUE + STEERING_ADJUST_STEP)
            {
                steering_voltage_value -= STEERING_ADJUST_STEP;
            } else {
                steering_voltage_value = STEERING_MIN_VALUE;
            }

            update_steering_voltage(steering_voltage_value);
            left_pressed = 1;
            left_last_press_time = loop_counter;
        } else if (loop_counter - left_last_press_time >= MS_TO_LOOPS(STEERING_BUTTON_REPEAT_DELAY_MS))
        {
            if (loop_counter - left_last_press_time >= MS_TO_LOOPS(STEERING_BUTTON_REPEAT_RATE_MS))
            {
                left_last_press_time = loop_counter;
                if (steering_voltage_value >= 
                    STEERING_MIN_VALUE + STEERING_ADJUST_STEP)
                {
                    steering_voltage_value -= STEERING_ADJUST_STEP;
                } else {
                    steering_voltage_value = STEERING_MIN_VALUE;
                }

                update_steering_voltage(steering_voltage_value);
            }
        }
    } else {
        left_pressed = 0;
        left_steering_pressed = 0;
    }

    /* Smoothly center steering (2.5v) when neither L/R is pressed */
    if (! left_steering_pressed && ! right_steering_pressed)
    {
        /* Proportional return to neutral: step = distance/DIVISOR, min 1.
         * Mimics spring-force return — faster far from center, slower near it. */
        if (steering_voltage_value > STEERING_MID_VALUE)
        {
            uint8_t distance = steering_voltage_value - STEERING_MID_VALUE;
            uint8_t return_step = distance / STEERING_RETURN_DIVISOR;
            if (return_step < 1) return_step = 1;

            if (distance <= return_step)
            {
                steering_voltage_value = STEERING_MID_VALUE;
            } else {
                steering_voltage_value -= return_step;
            }

            /* Only update DAC if value changed */
            mcp4902_write(1, steering_voltage_value);
        } else if (steering_voltage_value < STEERING_MID_VALUE)
        {
            uint8_t distance = STEERING_MID_VALUE - steering_voltage_value;
            uint8_t return_step = distance / STEERING_RETURN_DIVISOR;
            if (return_step < 1) return_step = 1;

            if (distance <= return_step)
            {
                steering_voltage_value = STEERING_MID_VALUE;
            } else {
                steering_voltage_value += return_step;
            }

            /* Only update DAC if value changed */
            mcp4902_write(1, steering_voltage_value);
        }
    }
}
