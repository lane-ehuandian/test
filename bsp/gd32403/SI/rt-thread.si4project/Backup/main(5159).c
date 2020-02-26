/*
 * File      : main.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-12-16     lane      first implementation
 */
#include <rtthread.h>
#include <board.h>
#include "led.h"
#include "fm17522_process.h"
#include "fm175xx.h"

#include "debug.h"
#include "IO_config.h"
#include "battery_process.h"

#include "common.h"

volatile unsigned int system_ms_tick = 0;
//存储BMS HALL 检测脚的状态bit0 -> BMS0;  bit1 -> BMS1
unsigned char gl_hall_gpio_state = 0;
typedef struct
{
    /*led*/
    struct rt_thread thread_led;
    unsigned char thread_led_stack[1024]; 
    /*shell*/
    struct rt_thread thread_shell;
	unsigned char thread_shell_stack[1024]; 
	/*fm17522*/
    struct rt_thread thread_fm17522;
    unsigned char thread_fm17522_stack[1024];
	/*battery*/
    struct rt_thread thread_battery;
    unsigned char thread_battery_stack[1024];
	/*rs485*/
    struct rt_thread thread_rs485;
    unsigned char thread_rs485_stack[1024];
	/*ms timer*/
    rt_timer_t timer_ms_handler;

}st_main_param;

st_main_param sl_main_param;

//rt_mutex_t mutex_nfc = RT_NULL;

static void main_ms_timer_cb(void* parameter);

static void thread_iic0_cleanup(struct rt_thread *tid);

static void main_check_hall0_switch(void *args)
{
	//低电平是电池在位
    if(rt_pin_read(HALL_SWITCH_BMS0_PIN))
	{
		if(gl_hall_gpio_state & 1)
		{
		    rt_kprintf("\r\nHall0 switch off!\r\n");
		}
		gl_hall_gpio_state &= 0xFE;	    
	}
	else
	{
		if(0 == (gl_hall_gpio_state & 1))
		{
		    rt_kprintf("\r\nHall0 switch on!\r\n");
		}
		gl_hall_gpio_state |= 1;		
	}                
}
static void main_check_hall1_switch(void *args)
{
	//低电平是电池在位
	if(rt_pin_read(HALL_SWITCH_BMS1_PIN))
	{
		if(gl_hall_gpio_state & 2)
		{
		    rt_kprintf("\r\nHall1 switch off!\r\n");
		}
		gl_hall_gpio_state &= 0xFD;	    
	}
	else
	{
        if(0 == (gl_hall_gpio_state & 2))
        {
             rt_kprintf("\r\nHall1 switch on!\r\n");
        }
		gl_hall_gpio_state |= 2;		
	}                
}
static void main_rcu_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_AF);
}
static void main_gpio_init(void)
{
    //LED_MCU_RUN
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(LED_PIN, PIN_HIGH);
	//HALL_SWITCH
	rt_pin_mode(HALL_SWITCH_BMS0_PIN, PIN_MODE_INPUT_PULLUP);
	rt_pin_mode(HALL_SWITCH_BMS1_PIN, PIN_MODE_INPUT_PULLUP);	
	rt_pin_attach_irq(HALL_SWITCH_BMS0_PIN, PIN_IRQ_MODE_RISING_FALLING, main_check_hall0_switch, RT_NULL);
	rt_pin_attach_irq(HALL_SWITCH_BMS1_PIN, PIN_IRQ_MODE_RISING_FALLING, main_check_hall1_switch, RT_NULL);
	rt_pin_irq_enable(HALL_SWITCH_BMS0_PIN, PIN_IRQ_ENABLE);
	rt_pin_irq_enable(HALL_SWITCH_BMS1_PIN, PIN_IRQ_ENABLE);
	main_check_hall0_switch(RT_NULL);
	main_check_hall1_switch(RT_NULL);
	
}
//unsigned char array[2][3]={{234,255,145},{17,213,167}};
int main(void)
{	
	rt_err_t res;
	main_rcu_init();
	main_gpio_init();
	//run led
//	rt_thread_init(&sl_main_param.thread_led,
//    "led",
//    thread_led_entry,
//    RT_NULL,
//    &sl_main_param.thread_led_stack[0],
//    sizeof(sl_main_param.thread_led_stack),
//    10, 10);
//    rt_thread_startup(&sl_main_param.thread_led);
		
	//debug
	res=rt_thread_init(&sl_main_param.thread_shell,
    "shell",
    thread_shell_entry,
    RT_NULL,
    &sl_main_param.thread_shell_stack[0],
    sizeof(sl_main_param.thread_shell_stack),
    8, 10);	
    if (res == RT_EOK) /* 如果获得线程控制块，启动这个线程 */
    {
        rt_thread_startup(&sl_main_param.thread_shell);
    }
	else
	{
		rt_kprintf("create thread debug failed");
	}
	//fm17522
	res=rt_thread_init(&sl_main_param.thread_fm17522,
    "fm17522",
    thread_fm17522_entry,
    RT_NULL,
    &sl_main_param.thread_fm17522_stack[0],
    sizeof(sl_main_param.thread_fm17522_stack),
    2, 10);	
    if (res == RT_EOK) /* 如果获得线程控制块，启动这个线程 */
    {
        sl_main_param.thread_fm17522.cleanup = thread_iic0_cleanup;
        rt_thread_startup(&sl_main_param.thread_fm17522);
    }
	else
	{
		rt_kprintf("create thread fm17522 failed");
	}
	//battery	
	rt_thread_init(&sl_main_param.thread_battery,
    "battery",
    thread_battery_entry,
    RT_NULL,
    &sl_main_param.thread_battery_stack[0],
    sizeof(sl_main_param.thread_battery_stack),
    3, 10);
    if (res == RT_EOK) /* 如果获得线程控制块，启动这个线程 */
    {
        rt_thread_startup(&sl_main_param.thread_battery);
    }
	else
	{
		rt_kprintf("create thread battery failed");
	}	
    //rs485	
	rt_thread_init(&sl_main_param.thread_rs485,
    "rs485",
    thread_rs485_entry,
    RT_NULL,
    &sl_main_param.thread_rs485_stack[0],
    sizeof(sl_main_param.thread_rs485_stack),
    4, 10);
    if (res == RT_EOK) /* 如果获得线程控制块，启动这个线程 */
    {
        rt_thread_startup(&sl_main_param.thread_rs485);
    }
	else
	{
		rt_kprintf("create thread 485 failed");
	}
    /*ms timer*/
    sl_main_param.timer_ms_handler = rt_timer_create("ms_timer", main_ms_timer_cb, RT_NULL, 1, RT_TIMER_FLAG_PERIODIC);
    if (sl_main_param.timer_ms_handler != RT_NULL) 
    {
        rt_timer_start(sl_main_param.timer_ms_handler);
    }
//	array[0][0]=sizeof(unsigned int);
//	array[0][1]=sizeof(uint16_t);
//	array[0][2]=sizeof(unsigned char);
//	rt_kprintf("array[1] address=%x\r\n",array[1]);
//	rt_kprintf("array[1]+2 address=%x\r\n",array[1]+2);
	while(1)
	{
	
	}

}


static void thread_iic0_cleanup(struct rt_thread *tid)
{
    if (tid != &sl_main_param.thread_fm17522)
    {
        return ;
    }
    rt_kprintf("fm17522 end\n");
    
}

static void main_ms_timer_cb(void* parameter)
{
	static unsigned int CLK_MS = (1000/RT_TICK_PER_SECOND);

	system_ms_tick += CLK_MS;
	
	Pcd_Comm_timer_cb();//NFC timeout
	Battery_timer_count_cb();
	master_rs485_timer_cb();
//	thread_iic0_timer_cb(&sl_main_param.thread_fm17522);//NFC线程超时
	if(system_ms_tick>10000)
	{
//		rt_thread_delete(tid1);
	}
//	&sl_main_param.thread_fm17522 = RT_NULL;
}



