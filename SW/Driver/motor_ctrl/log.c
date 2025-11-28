
#ifndef LOG_H
#define LOG_H


#endif


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>         // kmalloc()
#include <linux/uaccess.h>      // copy_to/from_user()
#include <linux/proc_fs.h>


#define N_ENTRIES 1000



typedef struct {
	ns_t t;
	// flags.
	u8 state;
	u8 late;
} log_entry_t;
	
static u32 last_entry = 0;
static log_entry_t log_entries[N_ENTRIES];


static struct proc_dir_entry *parent;

static ssize_t read_proc(struct file *filp, char __user *buffer, size_t length, loff_t * offset)
{
    pr_info("proc file read.....\n");

//
    if (copy_to_user(buffer, etx_array, len))
        pr_err("Data Send : Err!\n");
 
    return length;
}


static struct proc_ops proc_fops = {
        .proc_read = read_proc,
};



/*
size_t print_ns_t(const char* buf, ns_t t) {
	//TODO %ds%3dm%3du%3dn
	//snprintf
}

#define TICKS_NS 100000
u32 ns_t_2_ticks(ns_t t) {
	return t/TICKS_NS
}

size_t print_ns_t_and_ticks(const char* buf, ns_t t) {
	u32 ticks = ns_t_2_ticks(t);
	print_ns_t(buf,
}

void log_print() {
	// ns_t and ticks
	// rel abs from start
	//	rel to prev
	// state
	// late
}

*/


void log_init(ns_t start_for_relative) {
   proc_create("motor_ctrl_log", 0555, parent, &proc_fops);
}

void log_exit() {
   proc_remove(parent);
}

void log_add(
	ns_t t,
	u8 state,
	u8 late
) {
	log_entries[last_entry].t = t;
	log_entries[last_entry].state = state;
	log_entries[last_entry].late = late;
	last_entry++;
}
