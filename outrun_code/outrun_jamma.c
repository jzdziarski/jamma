/**
 * @file outrun_jamma.c
 * @brief Sega Outrun Rev. B to JAMMA Adapter Firmware (8MHz Internal Clock)
 * 
 * This firmware controls the ATMEGA328P microcontroller to interface
 * between JAMMA arcade inputs and analog voltage outputs via MCP4902 DAC.
 * 
 * - B1 (Accelerate): VOUTA adjustable 0-5V with UP/DOWN when pressed
 * - B2 (Shift): Single digital output PB2, HIGH or LOW depending on state
 * - B3 (Brake): Digital output PB1, HIGH or LOW depending on button state
 * - LEFT/RIGHT: Adjust VOUTB (steering voltage 0-5V)
 * 
 * Hardware:
 * - ATMEGA328P @ 8MHz INTERNAL CLOCK (no external crystal required)
 * - MCP4902 Dual 8-bit DAC via SPI (bit-banged on PB3/PB4/PB5)
 * 
 * Pin Assignments:
 * - Inputs (active-low with internal pull-ups):
 *   PD2 (Pin 4)  - B1 (Accelerate) - enables adjustable VOUTA
 *   PD3 (Pin 5)  - B2 (Shift toggle)
 *   PD4 (Pin 6)  - B3 (Brake) - digital HIGH/LOW output
 *   PD5 (Pin 11) - UP (increase accelerator voltage)
 *   PD6 (Pin 12) - DOWN (decrease accelerator voltage)
 *   PD7 (Pin 13) - LEFT (increase steering voltage)
 *   PB0 (Pin 14) - RIGHT (decrease steering voltage)
 * 
 * - Outputs:
 *   PB1 (Pin 15) - B3_OUT (Brake digital output, HIGH or LOW)
 *   PB2 (Pin 16) - B2_OUT (Shift output, HIGH or LOW depending on toggle)
 *   PB3 (Pin 17) - MCP4902 CS (chip select, active low)
 *   PB4 (Pin 18) - MCP4902 SDI (SPI data output to DAC)
 *   PB5 (Pin 19) - MCP4902 SCK (SPI clock) 
 * 
 * - DAC Outputs:
 *   VOUTA (Pin 14 on MCP4902) - Accelerator voltage (0-5V, adjustable with UP/DOWN when B1 pressed)
 *   VOUTB (Pin 10 on MCP4902) - Steering voltage (0-5V, adjustable with LEFT/RIGHT)
 */

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/*============================================================================
 * Configuration Constants (8-BIT DAC)
 *===========================================================================*/

// Button debounce timing (milliseconds)
#define DEBOUNCE_DELAY_MS       20
#define BUTTON_REPEAT_DELAY_MS  300
#define BUTTON_REPEAT_RATE_MS   50

// DAC value ranges (8-bit: 0-255)
#define DAC_MIN_VALUE           0
#define DAC_MAX_VALUE           255     // MCP4902 is 8-bit
#define DAC_MID_VALUE           127     // ~2.5V (half of 255)

// Voltage adjustment step size (~19.6mV per step for 8-bit DAC)
#define ACCEL_ADJUST_STEP       5       // ~98mV per step for responsive control
#define STEERING_ADJUST_STEP    2       // ~39mV per step for smooth steering

// Timing conversion: loop runs at ~100us intervals (from _delay_us(100))
#define LOOP_TIME_MS            0.1     // Each loop iteration is ~0.1ms
#define MS_TO_LOOPS(ms)         ((uint32_t)((ms) / LOOP_TIME_MS))

/*============================================================================
 * Pin Definitions 
 *===========================================================================*/

// Input pins (active-low) - accessed directly via PIND/PINB
#define B1_PIN_BIT              PD2     // Accelerate - Pin 4 (INT0)
#define B2_PIN_BIT              PD3     // Shift toggle - Pin 5 (INT1)
#define B3_PIN_BIT              PD4     // Brake - Pin 6 (XCK/T0)
#define UP_PIN_BIT              PD5     // Increase accel - Pin 11 (T1)
#define DOWN_PIN_BIT            PD6     // Decrease accel - Pin 12 (AIN0)
#define LEFT_PIN_BIT            PD7     // Increase steer - Pin 13 (AIN1)
#define RIGHT_PIN_BIT           PB0     // Decrease steer - Pin 14 (ICP1)

// Output pins
#define B3_OUT_PIN_BIT          PB1     // Brake output - Pin 15 (OC1A)
#define B2_OUT_PIN_BIT          PB2     // Shift output - Pin 16 (SS/OC1B)
#define MCP_CS_PIN_BIT          PB3     // DAC CS - Pin 17 → MCP4902 Pin 3
#define MCP_SD_PIN_BIT          PB4     // DAC SDI - Pin 18 → MCP4902 Pin 5 
#define MCP_SCK_PIN_BIT         PB5     // DAC SCK - Pin 19 → MCP4902 Pin 4 

/*============================================================================
 * Global Variables (8-BIT DAC VALUES)
 *===========================================================================*/

// DAC output values (8-bit, 0-255)
static uint8_t accel_voltage_value = 0;        // VOUTA, starts at 0V
static uint8_t steering_voltage_value = DAC_MID_VALUE;  // VOUTB, starts at ~2.5V

// Button state tracking
static uint8_t b2_toggle_state = 0;             // Shift toggle latch (0=LOW, 1=HIGH)
static uint8_t brake_pressed_state = 0;         // Brake button state (0 or 1)

// Timing variables for button repeat
static uint32_t up_last_press_time = 0;
static uint32_t down_last_press_time = 0;
static uint32_t left_last_press_time = 0;
static uint32_t right_last_press_time = 0;

// Loop iteration counter (each iteration is ~100us)
static uint32_t loop_counter = 0;

/*============================================================================
 * Function Prototypes
 *===========================================================================*/

void system_init(void);
void gpio_init(void);
void spi_init(void);
void mcp4902_write(uint8_t channel, uint8_t data);
void update_accel_voltage(uint8_t value);
void update_steering_voltage(uint8_t value);
uint8_t read_button(volatile uint8_t *port, uint8_t bit);
void set_brake_output(uint8_t state);
void set_shift_output(uint8_t state);
void update_outputs(void);

/*============================================================================
 * Main Function
 *===========================================================================*/

int main(void) {
    // Power-on delay to ensure JAMMA power rails are stable
    _delay_ms(500);
    
    // Initialize system
    system_init();
    
    // Initial DAC output update
    update_accel_voltage(accel_voltage_value);
    update_steering_voltage(steering_voltage_value);
    
    // Main loop
    while (1) {
        update_outputs();
        
        // Small delay to prevent CPU from running too fast (~100us per loop)
        _delay_us(100);
        
        // Increment loop counter for timing calculations
        loop_counter++;
    }
    
    return 0;
}

/*============================================================================
 * System Initialization
 *===========================================================================*/

void system_init(void) {
    gpio_init();
    spi_init();
    
    // Disable global interrupts - not needed for this application
    cli();
}

/*============================================================================
 * GPIO Initialization
 * 
 * Configures all input and output pins according to specification.
 * Inputs use internal pull-ups (active-low logic).
 * Outputs are configured as push-pull digital.
 * SPI uses bit-banged on PB3=CS, PB4=SDI, PB5=SCK.
 *===========================================================================*/

void gpio_init(void) {
    // Configure input pins with internal pull-ups (active-low)
    DDRD &= ~(1 << PD2);  // B1 input (Accelerate) - Pin 4
    DDRD &= ~(1 << PD3);  // B2 input (Shift toggle) - Pin 5
    DDRD &= ~(1 << PD4);  // B3 input (Brake) - Pin 6
    DDRD &= ~(1 << PD5);  // UP input - Pin 11
    DDRD &= ~(1 << PD6);  // DOWN input - Pin 12
    DDRD &= ~(1 << PD7);  // LEFT input - Pin 13
    
    DDRB &= ~(1 << PB0);  // RIGHT input - Pin 14
    
    PORTD |= (1 << PD2);  // Enable pull-up on B1
    PORTD |= (1 << PD3);  // Enable pull-up on B2
    PORTD |= (1 << PD4);  // Enable pull-up on B3
    PORTD |= (1 << PD5);  // Enable pull-up on UP
    PORTD |= (1 << PD6);  // Enable pull-up on DOWN
    PORTD |= (1 << PD7);  // Enable pull-up on LEFT
    
    PORTB |= (1 << PB0);  // Enable pull-up on RIGHT
    
    // Configure output pins as push-pull digital
    DDRB |= (1 << PB1);   // B3_OUT (Brake) - Pin 15
    PORTB &= ~(1 << PB1); // Start LOW (brake not pressed)
    
    DDRB |= (1 << PB2);   // B2_OUT (Shift) - Pin 16
    PORTB &= ~(1 << PB2); // Start LOW (default state)
    
    // SPI pins for MCP4902:
    DDRB |= (1 << PB3);   // CS - output, active low (Pin 17 → MCP4902 Pin 3)
    PORTB |= (1 << PB3);  // CS high (deselect DAC initially)
    
    DDRB |= (1 << PB4);   // SDI - output, SPI data (Pin 18 → MCP4902 Pin 5)
    PORTB &= ~(1 << PB4); // Default low
    
    DDRB |= (1 << PB5);   // SCK - output, SPI clock (Pin 19 → MCP4902 Pin 4)
    PORTB &= ~(1 << PB5); // SCK low
}

/*============================================================================
 * SPI Initialization
 * 
 * Disables hardware SPI - we use bit-banged on PB3/PB4/PB5 instead.
 *===========================================================================*/

void spi_init(void) {
    // Disable hardware SPI - using bit-banged on PB3/PB4/PB5
    SPCR = 0;
    SPSR = 0;
}

/*============================================================================
 * MCP4902 DAC Write Function (8-BIT, BIT-BANGED SPI)
 * 
 * Sends command and 8-bit data to the MCP4902 via software SPI.
 * Uses PB3 for CS, PB4 for SDI (data), PB5 for SCK.
 * 
 * @param channel 0 for VOUTA, 1 for VOUTB
 * @param data 8-bit DAC value (0-255)
 *===========================================================================*/

void mcp4902_write(uint8_t channel, uint8_t data) {
    uint8_t i;
    uint16_t command_word;
    
    // Build 16-bit command word for MCP4902 (8-bit DAC):
    // High byte: [A/B][BUF][GA][SHDN][D7:D4]
    // Low byte: [D3:D0][XXXX] (padding)
    if (channel == 0) {
        // Channel A (VOUTA): A/B=0, GD=0, SHDN=1, BUFFER=0
        command_word = 0x3000 | (data << 4);  // Shift data to high nibble position
    } else {
        // Channel B (VOUTB): A/B=1, GD=0, SHDN=1, BUFFER=0
        command_word = 0xB000 | (data << 4);  // Shift data to high nibble position
        // Was: 0x7000
    }
    
    // Assert chip select (active low) - PB3
    PORTB &= ~(1 << MCP_CS_PIN_BIT);
    
    // Send 16 bits MSB first (bit-banged SPI on PB4)
    for (i = 0; i < 16; i++) {
        // Set SDI (PB4) based on current bit
        if (command_word & 0x8000) {
            PORTB |= (1 << MCP_SD_PIN_BIT);   // Set HIGH
        } else {
            PORTB &= ~(1 << MCP_SD_PIN_BIT);  // Set LOW
        }
        
        // Pulse SCK (PB5) - rising edge
        PORTB |= (1 << MCP_SCK_PIN_BIT);
        
        // Small delay for signal stability
        _delay_us(1);
        
        // Falling edge of SCK
        PORTB &= ~(1 << MCP_SCK_PIN_BIT);
        
        // Shift to next bit
        command_word <<= 1;
    }
    
    // Deassert chip select (active low) - PB3
    PORTB |= (1 << MCP_CS_PIN_BIT);
}

/*============================================================================
 * Update Accelerator Voltage (VOUTA) - 8-BIT DAC
 * 
 * Updates the DAC channel A output with the specified voltage value.
 * When B1 is not pressed, this should be 0V (value = 0).
 * When B1 is pressed, value ranges from 0-255 (0V to 5V).
 * 
 * @param value 8-bit DAC value (0-255)
 *===========================================================================*/

void update_accel_voltage(uint8_t value) {
    // Clamp value to valid range
    if (value > DAC_MAX_VALUE) {
        value = DAC_MAX_VALUE;
    }
    
    accel_voltage_value = value;
    
    // Write to MCP4902 Channel A (VOUTA - Accelerator)
    mcp4902_write(0, value);  // Channel 0 = VOUTA
}

/*============================================================================
 * Update Steering Voltage (VOUTB) - 8-BIT DAC
 * 
 * Updates the DAC channel B output with the specified voltage value.
 * Default is ~2.5V (value = 127), adjustable from 0-5V (0-255).
 * 
 * @param value 8-bit DAC value (0-255)
 *===========================================================================*/

void update_steering_voltage(uint8_t value) {
    // Clamp value to valid range
    if (value > DAC_MAX_VALUE) {
        value = DAC_MAX_VALUE;
    }
    
    steering_voltage_value = value;
    
    // Write to MCP4902 Channel B (VOUTB - Steering)
    mcp4902_write(1, value);  // Channel 1 = VOUTB
}

/*============================================================================
 * Read Button State
 * 
 * Reads a button input and returns 1 if pressed (active-low), 0 otherwise.
 * 
 * @param port Pointer to port register (PIND or PINB)
 * @param bit Bit position within the port
 * @return 1 if button pressed, 0 if not pressed
 *===========================================================================*/

uint8_t read_button(volatile uint8_t *port, uint8_t bit) {
    // Button is active-low: 0 = pressed, 1 = not pressed
    return !(*port & (1 << bit));
}

/*============================================================================
 * Set Brake Output State
 * 
 * Sets the brake output pin (PB1) to HIGH or LOW.
 * This is a PUSH-PULL digital output - can drive both 5V and 0V directly.
 * 
 * @param state 1 for HIGH (5V), 0 for LOW (0V)
 *===========================================================================*/

void set_brake_output(uint8_t state) {
    if (state) {
        PORTB |= (1 << B3_OUT_PIN_BIT);   // Set HIGH (5V)
    } else {
        PORTB &= ~(1 << B3_OUT_PIN_BIT);  // Set LOW (0V)
    }
}

/*============================================================================
 * Set Shift Output State
 * 
 * Sets the shift output pin (PB2) to HIGH or LOW.
 * This is a PUSH-PULL digital output - can drive both 5V and 0V directly.
 * 
 * @param state 1 for HIGH (5V), 0 for LOW (0V)
 *===========================================================================*/

void set_shift_output(uint8_t state) {
    if (state) {
        PORTB |= (1 << B2_OUT_PIN_BIT);   // Set HIGH (5V)
    } else {
        PORTB &= ~(1 << B2_OUT_PIN_BIT);  // Set LOW (0V)
    }
}

/*============================================================================
 * Update All Outputs
 * 
 * Main output update function called in the main loop. Handles:
 * - B1 (Accelerate): Enables VOUTA, UP/DOWN adjust voltage when pressed
 * - B2 (Shift): Single digital output PB2, HIGH or LOW based on toggle
 * - B3 (Brake): Digital output PB1, HIGH when pressed, LOW when not
 * - LEFT/RIGHT: Adjusts steering voltage (VOUTB)
 * 
 * Implements debouncing and button repeat functionality.
 *===========================================================================*/

void update_outputs(void) {
    static uint8_t b1_state = 0;
    
    // =========================================================================
    // B1 (Accelerate) - Enables adjustable VOUTA voltage
    // When pressed: VOUTA is enabled and can be adjusted with UP/DOWN
    // When not pressed: VOUTA = 0V
    // =========================================================================
    
    if (read_button(&PIND, B1_PIN_BIT)) {  // B1 pressed
        if (!b1_state) {
            b1_state = 1;
            // Default to 0V when first pressed (safe)
            if (accel_voltage_value == 0) {
                accel_voltage_value = DAC_MIN_VALUE;  // 0V default (safe)
            }
            update_accel_voltage(accel_voltage_value);
        }
    } else {
        if (b1_state) {
            b1_state = 0;
            update_accel_voltage(0);  // 0V when not pressed
        }
    }
    
    // =========================================================================
    // B2 (Shift) - Single Digital Output (PB2, PUSH-PULL)
    // Toggles state on button press, maintains state internally
    // =========================================================================
    
    static uint8_t b2_last_state = 1;  // Start as not pressed (pull-up)
    static uint32_t b2_debounce_counter = 0;
    
    uint8_t b2_current = read_button(&PIND, B2_PIN_BIT);
    
    // Debounce detection
    if (b2_current != b2_last_state) {
        b2_debounce_counter++;
        if (b2_debounce_counter >= MS_TO_LOOPS(DEBOUNCE_DELAY_MS)) {
            // State changed, toggle latch
            if (b2_current == 0) {  // Button just pressed
                b2_toggle_state = !b2_toggle_state;
                
                // Update shift output based on new state
                set_shift_output(b2_toggle_state);
            }
            b2_debounce_counter = 0;
        }
    } else {
        b2_debounce_counter = 0;
    }
    
    b2_last_state = b2_current;
    
    // =========================================================================
    // B3 (Brake) - Digital Output (PB1, PUSH-PULL)
    // When pressed: PB1 = HIGH (5V)
    // When not pressed: PB1 = LOW (0V)
    // =========================================================================
    
    if (read_button(&PIND, B3_PIN_BIT)) {  // B3 pressed
        if (!brake_pressed_state) {
            brake_pressed_state = 1;
            set_brake_output(1);  // Drive HIGH (5V)
        }
    } else {
        if (brake_pressed_state) {
            brake_pressed_state = 0;
            set_brake_output(0);  // Drive LOW (0V)
        }
    }
    
    // =========================================================================
    // UP/DOWN - Adjust accelerator voltage (only when B1 is pressed)
    // =========================================================================
    
    static uint8_t up_pressed = 0;
    static uint8_t down_pressed = 0;
    
    // UP button - increase accelerator voltage
    if (read_button(&PIND, UP_PIN_BIT)) {  // UP pressed
        if (!up_pressed) {
            // First press - immediate adjustment
            accel_voltage_value += ACCEL_ADJUST_STEP;
            if (accel_voltage_value > DAC_MAX_VALUE) {
                accel_voltage_value = DAC_MAX_VALUE;
            }
            if (b1_state) {
                update_accel_voltage(accel_voltage_value);
            }
            up_pressed = 1;
            up_last_press_time = loop_counter;
        } else if (b1_state && 
                   (loop_counter - up_last_press_time >= MS_TO_LOOPS(BUTTON_REPEAT_DELAY_MS))) {
            // Repeat adjustment while held
            if ((loop_counter - up_last_press_time) % MS_TO_LOOPS(BUTTON_REPEAT_RATE_MS) < 2) {
                accel_voltage_value += ACCEL_ADJUST_STEP;
                if (accel_voltage_value > DAC_MAX_VALUE) {
                    accel_voltage_value = DAC_MAX_VALUE;
                }
                update_accel_voltage(accel_voltage_value);
            }
        }
    } else {
        up_pressed = 0;
    }
    
    // DOWN button - decrease accelerator voltage
    if (read_button(&PIND, DOWN_PIN_BIT)) {  // DOWN pressed
        if (!down_pressed) {
            // First press - immediate adjustment
            if (accel_voltage_value >= ACCEL_ADJUST_STEP) {
                accel_voltage_value -= ACCEL_ADJUST_STEP;
            } else {
                accel_voltage_value = 0;
            }
            if (b1_state) {
                update_accel_voltage(accel_voltage_value);
            }
            down_pressed = 1;
            down_last_press_time = loop_counter;
        } else if (b1_state && 
                   (loop_counter - down_last_press_time >= MS_TO_LOOPS(BUTTON_REPEAT_DELAY_MS))) {
            // Repeat adjustment while held
            if ((loop_counter - down_last_press_time) % MS_TO_LOOPS(BUTTON_REPEAT_RATE_MS) < 2) {
                if (accel_voltage_value >= ACCEL_ADJUST_STEP) {
                    accel_voltage_value -= ACCEL_ADJUST_STEP;
                } else {
                    accel_voltage_value = 0;
                }
                update_accel_voltage(accel_voltage_value);
            }
        }
    } else {
        down_pressed = 0;
    }
    
    // =========================================================================
    // LEFT/RIGHT - Adjust steering voltage (VOUTB)
    // =========================================================================
    
    static uint8_t left_pressed = 0;
    static uint8_t right_pressed = 0;
    
    // LEFT button - increase steering voltage toward 5V
    if (read_button(&PIND, LEFT_PIN_BIT)) {  // LEFT pressed
        if (!left_pressed) {
            // First press - immediate adjustment
            steering_voltage_value += STEERING_ADJUST_STEP;
            if (steering_voltage_value > DAC_MAX_VALUE) {
                steering_voltage_value = DAC_MAX_VALUE;
            }
            update_steering_voltage(steering_voltage_value);
            left_pressed = 1;
            left_last_press_time = loop_counter;
        } else if ((loop_counter - left_last_press_time >= MS_TO_LOOPS(BUTTON_REPEAT_DELAY_MS))) {
            // Repeat adjustment while held
            if ((loop_counter - left_last_press_time) % MS_TO_LOOPS(BUTTON_REPEAT_RATE_MS) < 2) {
                steering_voltage_value += STEERING_ADJUST_STEP;
                if (steering_voltage_value > DAC_MAX_VALUE) {
                    steering_voltage_value = DAC_MAX_VALUE;
                }
                update_steering_voltage(steering_voltage_value);
            }
        }
    } else {
        left_pressed = 0;
    }
    
    // RIGHT button - decrease steering voltage toward 0V
    if (read_button(&PINB, RIGHT_PIN_BIT)) {  // RIGHT pressed
        if (!right_pressed) {
            // First press - immediate adjustment
            if (steering_voltage_value >= STEERING_ADJUST_STEP) {
                steering_voltage_value -= STEERING_ADJUST_STEP;
            } else {
                steering_voltage_value = 0;
            }
            update_steering_voltage(steering_voltage_value);
            right_pressed = 1;
            right_last_press_time = loop_counter;
        } else if ((loop_counter - right_last_press_time >= MS_TO_LOOPS(BUTTON_REPEAT_DELAY_MS))) {
            // Repeat adjustment while held
            if ((loop_counter - right_last_press_time) % MS_TO_LOOPS(BUTTON_REPEAT_RATE_MS) < 2) {
                if (steering_voltage_value >= STEERING_ADJUST_STEP) {
                    steering_voltage_value -= STEERING_ADJUST_STEP;
                } else {
                    steering_voltage_value = 0;
                }
                update_steering_voltage(steering_voltage_value);
            }
        }
    } else {
        right_pressed = 0;
    }
    
    // Return to neutral (2.5V) when neither LEFT nor RIGHT is pressed
    if (!left_pressed && !right_pressed) {
        // Check if we're close to neutral (within one step)
        uint16_t diff = (steering_voltage_value > DAC_MID_VALUE) ? 
                        (steering_voltage_value - DAC_MID_VALUE) : 
                        (DAC_MID_VALUE - steering_voltage_value);
        
        if (diff <= STEERING_ADJUST_STEP) {
            // Snap to exact neutral
            steering_voltage_value = DAC_MID_VALUE;
            update_steering_voltage(steering_voltage_value);
        } else {
            // Move toward neutral gradually (smooth return)
            if (steering_voltage_value > DAC_MID_VALUE) {
                steering_voltage_value -= STEERING_ADJUST_STEP / 2;
                if (steering_voltage_value < DAC_MID_VALUE) {
                    steering_voltage_value = DAC_MID_VALUE;
                }
            } else if (steering_voltage_value < DAC_MID_VALUE) {
                steering_voltage_value += STEERING_ADJUST_STEP / 2;
                if (steering_voltage_value > DAC_MID_VALUE) {
                    steering_voltage_value = DAC_MID_VALUE;
                }
            }
            update_steering_voltage(steering_voltage_value);
        }
    }
}

