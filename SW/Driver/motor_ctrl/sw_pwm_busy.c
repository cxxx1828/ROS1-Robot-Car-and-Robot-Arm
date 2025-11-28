
#include "sw_pwm.h"

#include "gpio.h"
#include "include/motor_ctrl.h" // DEV_NAME

#include <asm/io.h> // ioremap(), iounmap()
#include <linux/delay.h> // mdelay()
#include <linux/errno.h> // ENOMEM
#include <linux/kthread.h> // kthread stuff

static const uint8_t pins[SW_PWM__N_CH] = {
	16,
	19,
	20,
	26
};

typedef u64 ns_t;

typedef struct {
	uint8_t pin;
	bool on;
	uint32_t moduo;
	uint32_t threshold;
	// TODO Maybe this kill performance and break PREEMPT_RT
	spinlock_t d_pending_lock;
	ns_t d_on_pending;
	ns_t d_off_pending;
	ns_t d_on;
	ns_t d_off;
	ns_t t_event;
} sw_pwm_t;
static sw_pwm_t sw_pwms[SW_PWM__N_CH];

//TODO ns_to_ticks();

static struct task_struct* thread;
int busy_pwm_loop(void* data) {
	uint8_t ch;
	sw_pwm_t* ps;
	unsigned long flags;
	ns_t t_now;
	ns_t t_next;
	ns_t d_sleep;
	(void) data;
int d = 0;

	
	while(!kthread_should_stop()){
		t_now = ktime_get_ns();
		
		//FIXME If we late, we need recovery.
		//FIXME cannot change duty.
if(d < 10){ printk(KERN_WARNING DEV_NAME": d = %d\n", ++d); }
if(d < 10){ printk(KERN_WARNING DEV_NAME": ps->t_event = %lld\n", ps->t_event); }
		t_next = ~(ns_t)0; // type max.
		for(ch = 0; ch < SW_PWM__N_CH; ch++){
			ps = &sw_pwms[ch];
			if(ps->t_event >= t_now){
				ps->on = !ps->on;
				if(ps->on){
					gpio__set(ps->pin);
					
					// Changing interval at the end of period.
					spin_lock_irqsave(&ps->d_pending_lock, flags);
					ps->d_on = ps->d_on_pending;
					ps->d_off = ps->d_off_pending;
					spin_unlock_irqrestore(&ps->d_pending_lock, flags);

					ps->t_event += ps->d_on;
				}else{
					gpio__clear(ps->pin);
					
					ps->t_event += ps->d_off;
				}
			}
			
			if(ps->t_event < t_next){
				t_next = ps->t_event;
			}
		}
		
if(d < 10){ printk(KERN_WARNING DEV_NAME": t_next = %lld\n", t_next); }
		if(t_next > (t_now + 1000)){
			d_sleep = t_next - (t_now + 1000); // 1us safety.
			if(d_sleep > 1000){
				ndelay(d_sleep);
			}
		}
	}
	
	do_exit(0);
	
	return 0;
}

static void set_intervals(sw_pwm_t* ps) {
	unsigned long flags;

	// 10000 stands for 10 us.
	ns_t on  = (ns_t)10000*ps->threshold;
	ns_t off = (ns_t)10000*(ps->moduo - ps->threshold);

	spin_lock_irqsave(&ps->d_pending_lock, flags);
	ps->d_on_pending = on;
	ps->d_off_pending = off;
	spin_unlock_irqrestore(&ps->d_pending_lock, flags);
}

int sw_pwm__init(void) {
	int r = 0;
	ns_t t_now;
	uint8_t ch;
	sw_pwm_t* ps;

	t_now = ktime_get_ns();
printk(KERN_WARNING DEV_NAME": t_now = %lld\n", t_now);
	for(ch = 0; ch < SW_PWM__N_CH; ch++){
		ps = &sw_pwms[ch];

		ps->pin = pins[ch];
		gpio__clear(ps->pin);
		gpio__steer_pinmux(ps->pin, GPIO__OUT);

		ps->on = false;

		spin_lock_init(&ps->d_pending_lock);

		ps->moduo = 1000;
		ps->threshold = 0;
ps->moduo = 1000<<1;
ps->threshold = 75<<1;
		set_intervals(ps);
		ps->d_on = ps->d_on_pending;
		ps->d_off = ps->d_off_pending;
		
		// First cycle will be 0% of PWM.
		ps->t_event = t_now + ps->d_on + ps->d_off;
printk(KERN_WARNING DEV_NAME": ps->t_event = %lld\n", ps->t_event);
	}

	thread = kthread_create(busy_pwm_loop, 0, "busy_pwm_loop");
	if(thread){
		kthread_bind(thread, 0);
		wake_up_process(thread);
	}else{
		r = -EFAULT;
		goto exit;
	}
	
exit:
	if(r){
		printk(KERN_ERR DEV_NAME": %s() failed with %d!\n", __func__, r);
		sw_pwm__exit();
	}
	return r;
}

void sw_pwm__exit(void) {
	uint8_t ch;
	sw_pwm_t* ps;
	
	if(thread){
		// Stop thread.
		kthread_stop(thread);
	}

	for(ch = 0; ch < SW_PWM__N_CH; ch++){
		ps = &sw_pwms[ch];

		gpio__clear(ps->pin);
		gpio__steer_pinmux(ps->pin, GPIO__IN);
	}
	
	//TODO Log lating and stuff.
}


void sw_pwm__set_moduo(sw_pwm__ch_t ch, uint32_t moduo) {
	if(ch >= SW_PWM__N_CH){
		return;
	}
	sw_pwms[ch].moduo = moduo;
	set_intervals(&sw_pwms[ch]);
}

void sw_pwm__set_threshold(sw_pwm__ch_t ch, uint32_t threshold) {
	if(ch >= SW_PWM__N_CH){
		return;
	}
	sw_pwms[ch].threshold = threshold;
	set_intervals(&sw_pwms[ch]);
}
