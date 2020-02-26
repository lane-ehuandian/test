/*
 * File      : drv_i2c.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2018, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-06-05     tanek        first implementation.
 * 2018-04-19     misonyo      Porting for gd32f30x
 * 2019-12-16     lane         Porting for gd32f403,centrol
 */

#include "drv_i2c.h"
#include <rtthread.h>
#include "gd32f403.h"
#include "IO_config.h"

#ifdef RT_USING_I2C

#include <rtdevice.h>

#ifdef RT_USING_I2C_BITOPS

/*user can change this*/
#define I2C_BUS_NAME  "i2c2"

/*user should change this to adapt specific board*/
#define I2C_SCL_PIN          GPIO_PIN_4
#define I2C_SCL_PORT         GPIOE
#define I2C_SCL_CLK          RCU_GPIOE
#define I2C_SDA_PIN          GPIO_PIN_5
#define I2C_SDA_PORT         GPIOE
#define I2C_SDA_CLK          RCU_GPIOE

struct gd32_i2c_bit_data
{
    struct
    {
        rcu_periph_enum clk;
        rt_uint32_t port;
        rt_uint32_t pin;
    }scl, sda;
};

static void gpio_set_sda(void *data, rt_int32_t state)
{
    struct gd32_i2c_bit_data* bd = data;

    if (state)
    {
        gpio_bit_set(bd->sda.port, bd->sda.pin);
    }
    else
    {
        gpio_bit_reset(bd->sda.port, bd->sda.pin);
    }
}

static void gpio_set_scl(void *data, rt_int32_t state)
{
    struct gd32_i2c_bit_data* bd = data;
    if (state)
    {
        gpio_bit_set(bd->scl.port, bd->scl.pin);
    }
    else
    {
        gpio_bit_reset(bd->scl.port, bd->scl.pin);
    }
}

static rt_int32_t gpio_get_sda(void *data)
{
    struct gd32_i2c_bit_data* bd = data;

    return gpio_input_bit_get(bd->sda.port, bd->sda.pin);
}

static rt_int32_t gpio_get_scl(void *data)
{
    struct gd32_i2c_bit_data* bd = data;

    return gpio_input_bit_get(bd->scl.port, bd->scl.pin);
}

static void gpio_udelay(rt_uint32_t us)
{
    int i = ( rcu_clock_freq_get(CK_SYS) / 4000000 * us);
    while(i)
    {
        i--;
    }
}

static void drv_i2c_gpio_init(const struct gd32_i2c_bit_data* bd)
{
    rcu_periph_clock_enable(bd->sda.clk);
    rcu_periph_clock_enable(bd->scl.clk);
    gpio_init(bd->sda.port, GPIO_MODE_OUT_OD, GPIO_OSPEED_10MHZ, bd->sda.pin);
    gpio_init(bd->scl.port, GPIO_MODE_OUT_OD, GPIO_OSPEED_10MHZ, bd->scl.pin);

    gpio_bit_set(bd->sda.port, bd->sda.pin);
    gpio_bit_set(bd->scl.port, bd->scl.pin);
}

#else /* use hardware i2c */

struct gd32_i2c_bus
{
    struct rt_i2c_bus_device parent;
    rt_uint32_t i2c_periph;
////////////////////////////////////////////////////////////////////////////////////////////////////
    // added by Kim; 阻塞模式并中断收发数据

    //信号量
    rt_sem_t i2c_sem;
    //当前是发送或者接收模式; I2C transfer direction: I2C_RECEIVER, I2C_TRANSMITTER
    rt_uint32_t i2c_tran_dir; 
    //i2c地址，是左移一位的值
    rt_uint16_t i2c_slave_addr;
    // 如果是接收模式，存放接收数据；如果是发送模式，存放发送数据；
    rt_uint8_t *i2c_data_buf;
    rt_uint8_t i2c_data_buf_len;
    //目前已经接收或者发送的数据计数
    rt_uint8_t i2c_data_buf_cnt;
////////////////////////////////////////////////////////////////////////////////////////////////////
	
};

////////////////////////////////////////////////////////////////////////////////////////////////////
//静态变量
static struct gd32_i2c_bus *sl_gd32_i2c_bus[2] = {RT_NULL, RT_NULL};

/*Test*/
static unsigned int sl_sem_release_cnt = 0;
static unsigned int sl_sem_take_cnt = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////
//静态函数
static void gd32_i2c_save_bus_param(struct gd32_i2c_bus *gd32_bus);
static struct gd32_i2c_bus *gd32_i2c_get_bus_param(rt_uint32_t bus_select);
////////////////////////////////////////////////////////////////////////////////////////////////////


/*!
    \brief      handle I2C0 event interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void I2C1_EventIRQ_Handler(void)
{
    struct gd32_i2c_bus *vl_gd32_bus;  
    rt_uint32_t vl_state0_reg, vl_state1_reg;
    rt_uint32_t bus_select = I2C1;
    
    rt_interrupt_enter();
    
    vl_state0_reg = I2C_STAT0(bus_select);
    vl_state1_reg = I2C_STAT1(bus_select);
    vl_gd32_bus = gd32_i2c_get_bus_param(bus_select);
    if(i2c_flag_get_via_reg_value(vl_state0_reg, vl_state1_reg, I2C_FLAG_SBSEND))
    {
        //vl_gd32_bus = gd32_i2c_get_bus_param(bus_select);
        /* send slave address to I2C bus */
        i2c_master_addressing(bus_select, vl_gd32_bus->i2c_slave_addr, vl_gd32_bus->i2c_tran_dir);
    }
    else if(i2c_flag_get_via_reg_value(vl_state0_reg, vl_state1_reg, I2C_FLAG_ADDSEND))
    {
        /* clear the ADDSEND bit */
        i2c_flag_clear(bus_select,I2C_FLAG_ADDSEND);

        //vl_gd32_bus = gd32_i2c_get_bus_param(bus_select);

        if(I2C_RECEIVER == vl_gd32_bus->i2c_tran_dir)
        {
            if(1 == vl_gd32_bus->i2c_data_buf_len)
            {
                /* disable acknowledge */
                i2c_ack_config(bus_select,I2C_ACK_DISABLE);
                if(i2c_flag_get_via_reg_value(vl_state0_reg, vl_state1_reg, I2C_FLAG_MASTER))
                {
                    /* send a stop condition to I2C bus */
                    i2c_stop_on_bus(bus_select);
                }
            }
        }
    }
    else if((I2C_TRANSMITTER == vl_gd32_bus->i2c_tran_dir)&&(i2c_flag_get_via_reg_value(vl_state0_reg, vl_state1_reg, I2C_FLAG_TBE)))
    {
        //vl_gd32_bus = gd32_i2c_get_bus_param(bus_select);
        if(0 == vl_gd32_bus->i2c_data_buf_len)
        {
            if((0 == (I2C_CTL0(bus_select)&0x0200))/*是否已经置位stop位*/
                &&(i2c_flag_get_via_reg_value(vl_state0_reg, vl_state1_reg, I2C_FLAG_I2CBSY)))/*是否i2c通讯忙*/
            {
                if(i2c_flag_get_via_reg_value(vl_state0_reg, vl_state1_reg, I2C_FLAG_MASTER))
                {
                    /* send a stop condition to I2C bus */
                    i2c_stop_on_bus(bus_select);
                }
                //已经没有可发送的数据
                /* disable the error interrupt */
                i2c_interrupt_disable(bus_select, I2C_CTL1_BUFIE);
                
                if(RT_NULL != vl_gd32_bus->i2c_sem)
                {
                    if(RT_EOK == rt_sem_trytake(vl_gd32_bus->i2c_sem))
                    {
                        /*Breakpoit test*/
                        /*__asm volatile("BKPT #01");*/
                    }
                    
                    
                    rt_sem_release(vl_gd32_bus->i2c_sem);
                }
            }
        }
        else
        {
            i2c_data_transmit(bus_select, vl_gd32_bus->i2c_data_buf[vl_gd32_bus->i2c_data_buf_cnt]);

            /* decrement the read bytes counter */
            vl_gd32_bus->i2c_data_buf_len--;
            vl_gd32_bus->i2c_data_buf_cnt++;
        }
    }
    else if((I2C_TRANSMITTER == vl_gd32_bus->i2c_tran_dir)&&(i2c_flag_get_via_reg_value(vl_state0_reg, vl_state1_reg, I2C_FLAG_BTC)))
    {
        if(0 == vl_gd32_bus->i2c_data_buf_len)
        {
            if((0 == (I2C_CTL0(bus_select)&0x0200))/*是否已经置位stop位*/
                &&(i2c_flag_get_via_reg_value(vl_state0_reg, vl_state1_reg, I2C_FLAG_I2CBSY)))/*是否i2c通讯忙*/
            {
                if(i2c_flag_get_via_reg_value(vl_state0_reg, vl_state1_reg, I2C_FLAG_MASTER))
                {
                    /* send a stop condition to I2C bus */
                    i2c_stop_on_bus(bus_select);
                }
                //已经没有可发送的数据
                /* disable the error interrupt */
                i2c_interrupt_disable(bus_select, I2C_CTL1_BUFIE);
                
                if(RT_NULL != vl_gd32_bus->i2c_sem)
                {
                    if(RT_EOK == rt_sem_trytake(vl_gd32_bus->i2c_sem))
                    {
                        /*Breakpoit test*/
                        __asm volatile("BKPT #01");
                    }
                    
                    rt_sem_release(vl_gd32_bus->i2c_sem);
                }
            }
        }
        else
        {
            i2c_data_transmit(bus_select, vl_gd32_bus->i2c_data_buf[vl_gd32_bus->i2c_data_buf_cnt]);

            /* decrement the read bytes counter */
            vl_gd32_bus->i2c_data_buf_len--;
            vl_gd32_bus->i2c_data_buf_cnt++;
        }
    }
    else if(i2c_flag_get_via_reg_value(vl_state0_reg, vl_state1_reg, I2C_FLAG_RBNE))
    {
        //vl_gd32_bus = gd32_i2c_get_bus_param(bus_select);

        if(0 == vl_gd32_bus->i2c_data_buf_len)
        {            
            i2c_data_receive(bus_select);

            /* disable acknowledge */
            i2c_ack_config(bus_select,I2C_ACK_DISABLE);
            if(i2c_flag_get_via_reg_value(vl_state0_reg, vl_state1_reg, I2C_FLAG_MASTER))
            {
                /* send a stop condition to I2C bus */
                i2c_stop_on_bus(bus_select);
            }
            if(RT_NULL != vl_gd32_bus->i2c_sem)
            {
                if(RT_EOK == rt_sem_trytake(vl_gd32_bus->i2c_sem))
                {
                    /*Breakpoit test*/
                    /*__asm volatile("BKPT #01");*/
                }
                rt_sem_release(vl_gd32_bus->i2c_sem);
            }
        }
        else
        {
            /* read a byte from the EEPROM */
            vl_gd32_bus->i2c_data_buf[vl_gd32_bus->i2c_data_buf_cnt] = i2c_data_receive(bus_select);

            if((0 == vl_gd32_bus->i2c_data_buf_cnt)&&
                (1 == vl_gd32_bus->i2c_data_buf_len))
            {
                /*这个数据包只读一个字节*/
                /* enable the error interrupt */
                i2c_interrupt_disable(bus_select, I2C_CTL1_BUFIE);
                if(RT_NULL != vl_gd32_bus->i2c_sem)
                {
                    if(RT_EOK == rt_sem_trytake(vl_gd32_bus->i2c_sem))
                    {
                        /*Breakpoit test*/
                        __asm volatile("BKPT #01");
                    }
                    rt_sem_release(vl_gd32_bus->i2c_sem);
                }
                /* decrement the read bytes counter */
                vl_gd32_bus->i2c_data_buf_len--;
                vl_gd32_bus->i2c_data_buf_cnt++;
            }
            else if((0 == vl_gd32_bus->i2c_data_buf_cnt)&&
                (2 == vl_gd32_bus->i2c_data_buf_len))
            {
                /*这个数据包只读两个字节*/
                /* disable acknowledge */
                i2c_ack_config(bus_select,I2C_ACK_DISABLE);
                if(i2c_flag_get_via_reg_value(vl_state0_reg, vl_state1_reg, I2C_FLAG_MASTER))
                {
                    /* send a stop condition to I2C bus */
                    i2c_stop_on_bus(bus_select);
                }
                /* decrement the read bytes counter */
                vl_gd32_bus->i2c_data_buf_len--;
                vl_gd32_bus->i2c_data_buf_cnt++;
            }
            else
            {
                if(2 == vl_gd32_bus->i2c_data_buf_len)
                {
                    /* disable acknowledge */
                    i2c_ack_config(bus_select,I2C_ACK_DISABLE);
                    if(i2c_flag_get_via_reg_value(vl_state0_reg, vl_state1_reg, I2C_FLAG_MASTER))
                    {
                        /* send a stop condition to I2C bus */
                        i2c_stop_on_bus(bus_select);
                    }
                }
                else if(1 == vl_gd32_bus->i2c_data_buf_len)
                {
                    /* enable the error interrupt */
                    i2c_interrupt_disable(bus_select, I2C_CTL1_BUFIE);
                    if(RT_NULL != vl_gd32_bus->i2c_sem)
                    {
                        if(RT_EOK == rt_sem_trytake(vl_gd32_bus->i2c_sem))
                        {
                            /*Breakpoit test*/
                            __asm volatile("BKPT #01");
                        }
                        rt_sem_release(vl_gd32_bus->i2c_sem);
                    }
                }
                /* decrement the read bytes counter */
                vl_gd32_bus->i2c_data_buf_len--;
                vl_gd32_bus->i2c_data_buf_cnt++;
            }
            
        }
    }
    else
    {
        sl_sem_release_cnt = sl_sem_release_cnt;
    }

    rt_interrupt_leave();

}
/*!
    \brief      handle I2C1 error interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void I2C1_ErrorIRQ_Handler(void)
{
    /* no acknowledge received */
    if(i2c_flag_get(I2C1, I2C_FLAG_AERR))
    {
        i2c_flag_clear(I2C1, I2C_FLAG_AERR);
    }

    /* SMBus alert */
    if(i2c_flag_get(I2C1, I2C_FLAG_SMBALT))
    {
        i2c_flag_clear(I2C1, I2C_FLAG_SMBALT);
    }

    /* bus timeout in SMBus mode */
    if(i2c_flag_get(I2C1, I2C_FLAG_SMBTO))
    {
        i2c_flag_clear(I2C1, I2C_FLAG_SMBTO);
    }

    /* over-run or under-run when SCL stretch is disabled */
    if(i2c_flag_get(I2C1, I2C_FLAG_OUERR))
    {
        i2c_flag_clear(I2C1, I2C_FLAG_OUERR);
    }

    /* arbitration lost */
    if(i2c_flag_get(I2C1, I2C_FLAG_LOSTARB))
    {
        i2c_flag_clear(I2C1, I2C_FLAG_LOSTARB);
    }

    /* bus error */
    if(i2c_flag_get(I2C1, I2C_FLAG_BERR))
    {
        i2c_flag_clear(I2C1, I2C_FLAG_BERR);
    }

    /* CRC value doesn't match */
    if(i2c_flag_get(I2C1, I2C_FLAG_PECERR))
    {
        i2c_flag_clear(I2C1, I2C_FLAG_PECERR);
    }

    /* disable the error interrupt */
    i2c_interrupt_disable(I2C1,I2C_CTL1_ERRIE | I2C_CTL1_BUFIE | I2C_CTL1_EVIE);
}
//初始化IIC0的信号量
static void gd32_i2c_sem_init(struct gd32_i2c_bus *gd32_bus, char* sem_name)
{    
    gd32_bus->i2c_sem = rt_sem_create(sem_name, 0, RT_IPC_FLAG_PRIO);
    if (gd32_bus->i2c_sem == RT_NULL)
    {
        i2c_dbg("create dynamic semaphore failed.\n");
    }
    else
    {
        i2c_dbg("create done. dynamic semaphore value = 0.\n");
    }
}

static void gd32_i2c_save_bus_param(struct gd32_i2c_bus *gd32_bus)
{
    if(I2C0 == gd32_bus->i2c_periph)
    {
        sl_gd32_i2c_bus[0] = gd32_bus;
    }
    else if(I2C1 == gd32_bus->i2c_periph)
    {
        sl_gd32_i2c_bus[1] = gd32_bus;
    }
}
static struct gd32_i2c_bus *gd32_i2c_get_bus_param(rt_uint32_t bus_select)
{
    if(I2C0 == bus_select)
    {
        return sl_gd32_i2c_bus[0];
    }
    else if(I2C1 == bus_select)
    {
        return sl_gd32_i2c_bus[1];
    }
    else
    {
        return RT_NULL;
    }
}
/*!
    \brief      cofigure the NVIC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void gd32_i2c_nvic_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
	#ifdef RT_USING_I2C0
    nvic_irq_enable(I2C0_EV_IRQn, 0, 3);
    nvic_irq_enable(I2C0_ER_IRQn, 0, 2);
	#endif
	#ifdef RT_USING_I2C1
    nvic_irq_enable(I2C1_EV_IRQn, 0, 3);
    nvic_irq_enable(I2C1_ER_IRQn, 0, 2);
	#endif
}

static int gd32_i2c_read(struct rt_i2c_bus_device *bus, rt_uint16_t slave_address, rt_uint8_t* p_buffer, rt_uint16_t data_byte)//(rt_uint32_t i2c_periph, rt_uint16_t slave_address, rt_uint8_t* p_buffer, rt_uint16_t data_byte)
{
    rt_err_t vl_result;
    rt_uint32_t vl_delay_cnt;

    struct gd32_i2c_bus *vl_gd32_i2c = (struct gd32_i2c_bus *)bus;

    if(0 == data_byte)
    {
        return 0;
    }

    //设置读取个数
    vl_gd32_i2c->i2c_data_buf_len = data_byte;
    vl_gd32_i2c->i2c_data_buf_cnt = 0;//目前计数为0
    //设置transfer direction
    vl_gd32_i2c->i2c_tran_dir = I2C_RECEIVER;
    //设置地址
    vl_gd32_i2c->i2c_slave_addr = slave_address<<1;
    //申请内存
    vl_gd32_i2c->i2c_data_buf = rt_malloc(vl_gd32_i2c->i2c_data_buf_len);
    if(RT_NULL == vl_gd32_i2c->i2c_data_buf)
    {
        /*Breakpoit test*/
        //__asm volatile("BKPT #01");
        //错误处理
        return 0;
    }

    /*如果I2C总线是繁忙状态，就发送停止位*/
    if(i2c_flag_get(vl_gd32_i2c->i2c_periph, I2C_FLAG_I2CBSY))
    {
        /* send a stop condition to I2C bus */
        i2c_stop_on_bus(vl_gd32_i2c->i2c_periph);
    }
    /* wait until I2C bus is idle */
    vl_delay_cnt = 100; //等待500ms;
    while((vl_delay_cnt)&&(
        (i2c_flag_get(vl_gd32_i2c->i2c_periph, I2C_FLAG_I2CBSY))))
    {
        vl_delay_cnt--;
        rt_thread_delay(1);
    }

    /*清除stop产生位*/
    if(I2C_CTL0(vl_gd32_i2c->i2c_periph)&0x0200)
    {
        I2C_CTL0(vl_gd32_i2c->i2c_periph) &= (~(0x0200));
    }


    /* wait until the stop condition is finished */
    vl_delay_cnt = 1000; //等待500ms;
    while((vl_delay_cnt)&&(I2C_CTL0(vl_gd32_i2c->i2c_periph)&0x0200))
    {
        vl_delay_cnt--;
        rt_thread_mdelay(1);
    }
    
    /*清空信号量。在这里，其实不应该有信号量*/
    {
        rt_uint8_t vl_i;
        for(vl_i = 0; vl_i < 5; vl_i++)
        {
            if(RT_EOK != rt_sem_trytake(vl_gd32_i2c->i2c_sem))
            {
                break;
            }
            else
            {
                vl_i = vl_i;
            }
        }
    }

    
    /* send a start condition to I2C bus */
    i2c_start_on_bus(vl_gd32_i2c->i2c_periph);
    
    //等待信号量
    if(RT_NULL == vl_gd32_i2c->i2c_sem)
    {
        //错误处理
        /*Breakpoit test*/
        __asm volatile("BKPT #01");
        goto result_out;
    }

    vl_result = rt_sem_take(vl_gd32_i2c->i2c_sem, RT_WAITING_FOREVER); //RT_WAITING_FOREVER);//100+(data_byte/40));// 400K速率时候，40个字节时间是1mS
    if (vl_result != RT_EOK)
    {
        rt_kprintf("i2c take a dynamic semaphore, failed.\n");
        //rt_sem_delete(sl_gd32_i2c0_sem);

        goto result_out;
    }
    
    /* wait until the stop condition is finished */
    while((i2c_flag_get(vl_gd32_i2c->i2c_periph, I2C_FLAG_I2CBSY)) && 
        (I2C_CTL0(vl_gd32_i2c->i2c_periph)&0x0200))
    {
        //rt_thread_mdelay(1);
    }

    /* enable acknowledge */
    i2c_ack_config(vl_gd32_i2c->i2c_periph,I2C_ACK_ENABLE);

    i2c_ackpos_config(vl_gd32_i2c->i2c_periph,I2C_ACKPOS_CURRENT);

    //if((0xff == vl_gd32_i2c->i2c_data_buf[0])||(0 == vl_gd32_i2c->i2c_data_buf_cnt))
    //{
        /*Breakpoit test*/
    //    __asm volatile("BKPT #01");
    //}

    //copy数据
    rt_memcpy(p_buffer, vl_gd32_i2c->i2c_data_buf, vl_gd32_i2c->i2c_data_buf_cnt);

    //恢复中断中关闭的缓存中断
    /* enable the error interrupt */
    i2c_interrupt_enable(vl_gd32_i2c->i2c_periph, I2C_CTL1_BUFIE);
    
result_out:
    //释放内存
    if(RT_NULL != vl_gd32_i2c->i2c_data_buf)
    {
        rt_free(vl_gd32_i2c->i2c_data_buf);
    }
    vl_gd32_i2c->i2c_data_buf = RT_NULL;
    return 0;
}

static int gd32_i2c_write(struct rt_i2c_bus_device *bus, uint16_t slave_address, uint8_t* p_buffer, uint16_t data_byte)
{
    rt_err_t vl_result;
    rt_uint32_t vl_delay_cnt;
    struct gd32_i2c_bus *vl_gd32_i2c = (struct gd32_i2c_bus *)bus;

    //设置写个数
    vl_gd32_i2c->i2c_data_buf_len = data_byte;
    vl_gd32_i2c->i2c_data_buf_cnt = 0;//目前计数为0
    //设置transfer direction
    vl_gd32_i2c->i2c_tran_dir = I2C_TRANSMITTER;
    //设置地址
    vl_gd32_i2c->i2c_slave_addr = slave_address<<1;
    //申请内存
    vl_gd32_i2c->i2c_data_buf = rt_malloc(vl_gd32_i2c->i2c_data_buf_len);
    if(RT_NULL == vl_gd32_i2c->i2c_data_buf)
    {
        //错误处理
        return 0;
    }
    //copy数据
    rt_memcpy(vl_gd32_i2c->i2c_data_buf, p_buffer, vl_gd32_i2c->i2c_data_buf_len);


    /*如果I2C总线是繁忙状态，就发送停止位*/
    if(i2c_flag_get(vl_gd32_i2c->i2c_periph, I2C_FLAG_I2CBSY))
    {
        /* send a stop condition to I2C bus */
        i2c_stop_on_bus(vl_gd32_i2c->i2c_periph);
    }
    /* wait until I2C bus is idle */
    vl_delay_cnt = 100; //等待500ms;
    while((vl_delay_cnt)&&(
        (i2c_flag_get(vl_gd32_i2c->i2c_periph, I2C_FLAG_I2CBSY))))
    {
        vl_delay_cnt--;
        rt_thread_delay(1);
    }

    /*清除stop产生位*/
    if(I2C_CTL0(vl_gd32_i2c->i2c_periph)&0x0200)
    {
        I2C_CTL0(vl_gd32_i2c->i2c_periph) &= (~(0x0200));
    }


    /* wait until the stop condition is finished */
    vl_delay_cnt = 1000; //等待500ms;
    while((vl_delay_cnt)&&(I2C_CTL0(vl_gd32_i2c->i2c_periph)&0x0200))
    {
        vl_delay_cnt--;
        rt_thread_mdelay(1);
    }
    
    /*清空信号量。在这里，其实不应该有信号量*/
    {
        rt_uint8_t vl_i;
        for(vl_i = 0; vl_i < 5; vl_i++)
        {
            if(RT_EOK != rt_sem_trytake(vl_gd32_i2c->i2c_sem))
            {
                break;
            }
            else
            {
                vl_i = vl_i;
            }
        }
    }

    

    /* send a start condition to I2C bus */
    i2c_start_on_bus(vl_gd32_i2c->i2c_periph);
    
    //等待信号量
    if(RT_NULL == vl_gd32_i2c->i2c_sem)
    {
        //错误处理
        goto result_out;
    }
    vl_result = rt_sem_take(vl_gd32_i2c->i2c_sem, RT_WAITING_FOREVER);//RT_WAITING_FOREVER);//100+(data_byte/40)); // 400K速率时候，40个字节时间是1mS
    if (vl_result != RT_EOK)
    {
        i2c_dbg("i2c take a dynamic semaphore, failed.\n");
        //rt_sem_delete(sl_gd32_i2c0_sem);
        goto result_out;
    }

    /* wait until the stop condition is finished , or I2C is not busy*/ /*  */
    while((i2c_flag_get(vl_gd32_i2c->i2c_periph, I2C_FLAG_I2CBSY)) && 
        (I2C_CTL0(vl_gd32_i2c->i2c_periph)&0x0200))
    {
        //rt_thread_delay(1);
    }

    //恢复中断中关闭的缓存中断
    /* enable the error interrupt */
    i2c_interrupt_enable(vl_gd32_i2c->i2c_periph, I2C_CTL1_BUFIE);

result_out:
    //释放内存
    if(RT_NULL != vl_gd32_i2c->i2c_data_buf)
    {
        rt_free(vl_gd32_i2c->i2c_data_buf);
    }
    vl_gd32_i2c->i2c_data_buf = RT_NULL;
    return 0;
}

static rt_size_t gd32_i2c_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct rt_i2c_msg *msg;
    rt_uint32_t i;
    rt_err_t ret = RT_ERROR;

    struct gd32_i2c_bus *gd32_i2c = (struct gd32_i2c_bus *)bus;

    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];

        if (msg->flags & RT_I2C_ADDR_10BIT)
        {
            i2c_mode_addr_config(gd32_i2c->i2c_periph,I2C_I2CMODE_ENABLE,I2C_ADDFORMAT_10BITS,0);
        }
        else
        {
            i2c_mode_addr_config(gd32_i2c->i2c_periph,I2C_I2CMODE_ENABLE,I2C_ADDFORMAT_7BITS,0);
        }
        if (msg->flags & RT_I2C_RD)
        {
            //if (gd32_i2c_read(gd32_i2c->i2c_periph, msg->addr, msg->buf, msg->len) != 0)
            if (gd32_i2c_read(bus, msg->addr, msg->buf, msg->len) != 0)
            {
                i2c_dbg("i2c bus write failed,i2c bus stop!\n");
                goto out;
            }
        }
        else
        {
            //if (gd32_i2c_write(gd32_i2c->i2c_periph, msg->addr, msg->buf, msg->len) != 0)
            if (gd32_i2c_write(bus, msg->addr, msg->buf, msg->len) != 0)
            {
                i2c_dbg("i2c bus write failed,i2c bus stop!\n");
                goto out;
            }
        }
    }
    
    ret = i;

out:
    i2c_dbg("send stop condition\n");

    return ret;
}

static const struct rt_i2c_bus_device_ops i2c_ops =
{ 
    gd32_i2c_xfer,
    RT_NULL,
    RT_NULL
};

#endif /* RT_USING_I2C_BITOPS */

static struct gd32_i2c_bus gd32_i2c1;
static struct gd32_i2c_bus gd32_i2c0;

int rt_hw_i2c_init(void)
{
#ifdef RT_USING_I2C_BITOPS
    {
        static struct rt_i2c_bus_device i2c_device;
        static const struct gd32_i2c_bit_data _i2c_bdata =
        {
            /* SCL */
            {    I2C_SCL_CLK, I2C_SCL_PORT, I2C_SCL_PIN},
            /* SDA */
            {    I2C_SDA_CLK, I2C_SDA_PORT, I2C_SDA_PIN},
        };

        static const struct rt_i2c_bit_ops _i2c_bit_ops =
        {
            (void*)&_i2c_bdata,
            gpio_set_sda,
            gpio_set_scl,
            gpio_get_sda,
            gpio_get_scl,
            gpio_udelay,
            1,
            100
        };

        drv_i2c_gpio_init(&_i2c_bdata);

        i2c_device.priv = (void *)&_i2c_bit_ops;
        rt_i2c_bit_add_bus(&i2c_device, I2C_BUS_NAME);
    } 

#else   /* register hardware I2C */

    //NVIC中断配置
    gd32_i2c_nvic_config();// added by Kim

#ifdef RT_USING_I2C0
#define I2C0_SPEED  100000

    static struct gd32_i2c_bus gd32_i2c0;
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);

    /* connect PB6 to I2C0_SCL, PB7 to I2C0_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);

    /* enable I2C clock */
    rcu_periph_clock_enable(RCU_I2C0);
    /* configure I2C clock */
    i2c_clock_config(I2C0,I2C0_SPEED,I2C_DTCY_2);
	rt_thread_mdelay(100);
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0,I2C_ACK_ENABLE);
	
    //IIC0的中断允许; Added by Kim
    i2c_interrupt_enable(I2C0, I2C_CTL1_ERRIE | I2C_CTL1_BUFIE | I2C_CTL1_EVIE);
	
    rt_memset((void *)&gd32_i2c0, 0, sizeof(struct gd32_i2c_bus));
    gd32_i2c0.parent.ops = &i2c_ops;
    gd32_i2c0.i2c_periph = I2C0;
    rt_i2c_bus_device_register(&gd32_i2c0.parent, "i2c0");
	
	//初始化IIC0的信号量; Added by Kim
    gd32_i2c_sem_init(&gd32_i2c0, "i2c0_dsem");
    //保存总线变量; Added by Kim
    gd32_i2c_save_bus_param(&gd32_i2c0);
	
#endif

#ifdef RT_USING_I2C1
#define I2C1_SPEED  100000//2000000时，fm17522发送时ComIrqReg寄存器中定时器时间用尽

    static struct gd32_i2c_bus gd32_i2c1;
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);

    /* connect PB10 to I2C1_SCL, PB11 to I2C1_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_11);

    /* enable I2C clock */
    rcu_periph_clock_enable(RCU_I2C1);
    /* configure I2C clock */
    i2c_clock_config(I2C1,I2C1_SPEED,I2C_DTCY_2);

    i2c_enable(I2C1);
    /* enable acknowledge */
    i2c_ack_config(I2C1,I2C_ACK_ENABLE);

    //IIC0的中断允许; Added by Kim
    i2c_interrupt_enable(I2C1, I2C_CTL1_ERRIE | I2C_CTL1_BUFIE | I2C_CTL1_EVIE);
	
    rt_memset((void *)&gd32_i2c1, 0, sizeof(struct gd32_i2c_bus));
    gd32_i2c1.parent.ops = &i2c_ops;
    gd32_i2c1.i2c_periph = I2C1;
    rt_i2c_bus_device_register(&gd32_i2c1.parent, "i2c1");
	
	//初始化IIC0的信号量; Added by Kim
    gd32_i2c_sem_init(&gd32_i2c1, "i2c1_dsem");
    //保存总线变量; Added by Kim
    gd32_i2c_save_bus_param(&gd32_i2c1);
	
#endif

#endif /* RT_USING_I2C_BITOPS */

    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_i2c_init);

#endif
/* end of i2c driver */
