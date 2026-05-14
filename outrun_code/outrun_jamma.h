/*
 * @file outrun_jamma.c
 * @brief Headers for Sega Outrun Rev. B to JAMMA Adapter Firmware
 */

#ifndef _OUTRUN_JAMMA_H_
#define _OUTRUN_JAMMA_H_

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

#endif /* _OUTRUN_JAMMA_H_ */
