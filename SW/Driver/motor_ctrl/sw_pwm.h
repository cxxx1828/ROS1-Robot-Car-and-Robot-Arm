
#ifndef SW_PWM_H
#define SW_PWM_H


#include <linux/types.h>

#define SW_PWM__N_CH 4

typedef enum {
	SW_PWM__CH_0, // GPIO16, pin 36
	SW_PWM__CH_1, // GPIO19, pin 35
	SW_PWM__CH_2, // GPIO20, pin 38
	SW_PWM__CH_3  // GPIO26, pin 37
} sw_pwm__ch_t;

/**
 * Setup PWM to 100kHz i.e. 10us.
 */
int sw_pwm__init(void);
void sw_pwm__exit(void);

void sw_pwm__set_moduo(sw_pwm__ch_t ch, uint32_t moduo);
/**
 * @a threshold How much of moduo PWM is ON.
 */
void sw_pwm__set_threshold(sw_pwm__ch_t ch, uint32_t threshold);


#endif // SW_PWM_H
