/*!
    \file  gd32f30x_it.c
    \brief the header file of the ISR
*/

/*
    Copyright (C) 2017 GigaDevice

    2017-02-10, V1.0.0, firmware for GD32F30x
*/

#include "gd32f403_it.h"
#include "drv_i2c.h"
#include "drv_usart.h"
//#include "IO_config.h"

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

#if 0
/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1);
}
#endif
/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1);
}
#if 0
/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}
#endif
/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

#if 0
/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}
#endif
/*!
    \brief      this function handles I2C0 event interrupt request exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void I2C0_EV_IRQHandler(void)
{
//    I2C0_EventIRQ_Handler();
}

/*!
    \brief      this function handles I2C0 error interrupt request exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void I2C0_ER_IRQHandler(void)
{
//    I2C0_ErrorIRQ_Handler();
}

/*!
    \brief      this function handles I2C1 event interrupt request exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void I2C1_EV_IRQHandler(void)
{
    I2C1_EventIRQ_Handler();
}

/*!
    \brief      this function handles I2C1 error interrupt request exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void I2C1_ER_IRQHandler(void)
{
    I2C1_ErrorIRQ_Handler();
}


/*
*
*/
/*!
    \brief      this function handles CAN0 RX0 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void CAN0_RX0_IRQHandler(void)
{
    /*Ported by Kim*/
//    CO_CAN1InterruptHandler(CO_SRCOFINT_CAN0_RX0);
    
    /* check the receive message */
    /*
    can_message_receive(CAN0, CAN_FIFO0, &receive_message);
    if((0x321 == receive_message.rx_sfid)&&(CAN_FF_STANDARD == receive_message.rx_ff) && (2 == receive_message.rx_dlen)){
        receive_flag = SET;
    }
    */
}

void CAN0_TX_IRQHandler(void)
{
    /*Ported by Kim*/
//    CO_CAN1InterruptHandler(CO_SRCOFINT_CAN0_TX);
}

void CAN0_RX1_IRQHandler(void)
{
    /*Ported by Kim*/
//    CO_CAN1InterruptHandler(CO_SRCOFINT_CAN0_RX1);
    //can_message_receive(CAN0, CAN_FIFO1, &receive_message);
}
void DMA0_Channel1_IRQHandler(void)
{
    dam0_chl1_isr();
}

void USART0_IRQHandler(void)
{
    usart0_isr();
}
void USART1_IRQHandler(void)
{
    usart1_isr();
}
void USART2_IRQHandler(void)
{
    usart2_isr();
}
