/*
 * @file sega_jamma.h
 * @brief Headers for Sega Outrun Rev. B to JAMMA Adapter Firmware
 */

#ifndef _SEGA_JAMMA_H_
#define _SEGA_JAMMA_H_

#define GAME_MODE_OUTRUN 1
#define GAME_MODE_HANGON 2

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

#endif /* _SEGA_JAMMA_H_ */
