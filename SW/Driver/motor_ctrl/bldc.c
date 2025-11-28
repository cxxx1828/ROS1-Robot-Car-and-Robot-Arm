
#include "bldc.h"

#include "gpio.h"

#include <linux/printk.h>
#include <linux/atomic.h>

#include <linux/gpio.h> // gpio stuff
#include <linux/interrupt.h> // irq stuff


typedef struct {
	uint8_t dir_gpio_no;
	uint8_t pg_gpio_no;
	const char* pg_label;
	int pg_irq;
	volatile dir_t dir;
	atomic64_t pulse_cnt;
} bldc_t;
static bldc_t bldc[] = {
	{
		6, // GPIO6, pin 31
		5, // GPIO5, pin 29
		"irq_gpio5",
		CW,
		0
	}
};


static irqreturn_t pg_isr(int irq, void* data) {
	bldc_t* p = (bldc_t*)data;
	atomic64_add(p->dir, &p->pulse_cnt);
	return IRQ_HANDLED;
}

int bldc__init(void) {
	int r;
	bldc__ch_t ch;
	
	for(ch = 0; ch < BLDC__N_CH; ch++){
		gpio__steer_pinmux(bldc[ch].dir_gpio_no, GPIO__OUT);
		bldc__set_dir(ch, CW);
		
		
		gpio__steer_pinmux(bldc[ch].pg_gpio_no, GPIO__IN);
		// Initialize GPIO ISR.
		r = gpio_request_one(
			bldc[ch].pg_gpio_no,
			GPIOF_IN,
			bldc[ch].pg_label
		);
		if(r){
			printk(
				KERN_ERR DEV_NAME": %s(): gpio_request_one() failed!\n",
				__func__
			);
			goto exit;
		}
		
		atomic64_set(&bldc[ch].pulse_cnt, 0);
		
		bldc[ch].pg_irq = gpio_to_irq(bldc[ch].pg_gpio_no);
		r = request_irq(
			bldc[ch].pg_irq,
			pg_isr,
			IRQF_TRIGGER_FALLING,
			bldc[ch].pg_label,
			&bldc[ch]
		);
		if(r){
			printk(
				KERN_ERR DEV_NAME": %s(): request_irq() failed!\n",
				__func__
			);
			goto exit;
		}
	}
	
exit:
	if(r){
		printk(KERN_ERR DEV_NAME": %s() failed with %d!\n", __func__, r);
		bldc__exit();
	}
	
	return r;
}

void bldc__exit(void) {
	bldc__ch_t ch;
	
	for(ch = 0; ch < BLDC__N_CH; ch++){
		disable_irq(bldc[ch].pg_irq);
		free_irq(bldc[ch].pg_irq, &bldc[ch]);
		gpio_free(bldc[ch].pg_gpio_no);
	}
}


void bldc__set_dir(bldc__ch_t ch, dir_t dir) {
	if(ch >= BLDC__N_CH){
		return;
	}
	bldc[ch].dir = dir;
	if(dir == CW){
		gpio__set(bldc[ch].dir_gpio_no);
	}else{
		gpio__clear(bldc[ch].dir_gpio_no);
	}
}

void bldc__set_duty(bldc__ch_t ch, u16 duty_permille) {
	if(ch >= BLDC__N_CH){
		return;
	}
	// TODO For SW PWM @ GPIO21, pin 40
}

void bldc__get_pulse_cnt(bldc__ch_t ch, int64_t* pulse_cnt) {
	if(ch >= BLDC__N_CH){
		return;
	}
	*pulse_cnt = atomic64_read(&bldc[ch].pulse_cnt);
}
