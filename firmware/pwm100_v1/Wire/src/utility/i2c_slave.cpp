/*********************************************************************************************************//**
 * @file    i2c_slave.c
 * @version $Rev:: 502          $
 * @date    $Date:: 2022-11-21 #$
 * @brief   The I2C Slave (interrupt mode with buffer).
 *************************************************************************************************************
 * @attention
 *
 * Firmware Disclaimer Information
 *
 * 1. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, which is supplied by Holtek Semiconductor Inc., (hereinafter referred to as "HOLTEK") is the
 *    proprietary and confidential intellectual property of HOLTEK, and is protected by copyright law and
 *    other intellectual property laws.
 *
 * 2. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, is confidential information belonging to HOLTEK, and must not be disclosed to any third parties
 *    other than HOLTEK and the customer.
 *
 * 3. The program technical documentation, including the code, is provided "as is" and for customer reference
 *    only. After delivery by HOLTEK, the customer shall use the program technical documentation, including
 *    the code, at their own risk. HOLTEK disclaims any expressed, implied or statutory warranties, including
 *    the warranties of merchantability, satisfactory quality and fitness for a particular purpose.
 *
 * <h2><center>Copyright (C) Holtek Semiconductor Inc. All rights reserved</center></h2>
 ************************************************************************************************************/

/* Includes ------------------------------------------------------------------------------------------------*/
#include "i2c_slave.h"

/* Settings ------------------------------------------------------------------------------------------------*/
#define I2C_SLAVE_DEBUG_MODE      0

/* Private types -------------------------------------------------------------------------------------------*/


/* Private constants ---------------------------------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------------------------------------*/
/* Global functions ----------------------------------------------------------------------------------------*/
/*********************************************************************************************************//**
  * @brief  I2C Configuration.
  * @param address: Slave's address.
  * @retval None
  ***********************************************************************************************************/
void I2CSlave::I2C_Configuration(HT_I2C_TypeDef *pI2Cx,
               IRQn_Type I2C_IRQn,
               RingBufferN<BufferSize> *txBuffer,
               RingBufferN<BufferSize> *rxBuffer,
               u32 address)
{
  this->txBuffer = txBuffer;
  this->rxBuffer = rxBuffer;
  I2C_Configuration(pI2Cx, I2C_IRQn, address);
}

void I2CSlave::I2C_Configuration(HT_I2C_TypeDef *pI2Cx,
               IRQn_Type I2C_IRQn,
               u32 address)
{
  /* !!! NOTICE !!!
     Notice that the local variable (structure) did not have an initial value.
     Please confirm that there are no missing members in the parameter settings below in this function.
  */
  I2C_InitTypeDef  I2C_InitStructure;
  /* Enable NVIC I2C interrupt                                                                              */
  gI2CSlaveState.pI2Cx         = pI2Cx;
  gI2CSlaveState.I2C_IRQn      = I2C_IRQn;
  gI2CSlaveState.uSlaveAddress = address;
  NVIC_EnableIRQ(I2C_IRQn);
  NVIC_SetPriority(I2C_IRQn, 2);

  /* I2C Slave configuration                                                                                */
  I2C_InitStructure.I2C_GeneralCall = DISABLE;
  I2C_InitStructure.I2C_AddressingMode = I2C_ADDRESSING_7BIT;
  I2C_InitStructure.I2C_Acknowledge = ENABLE;
  I2C_InitStructure.I2C_OwnAddress = address;
  I2C_InitStructure.I2C_Speed = 0;
  I2C_InitStructure.I2C_SpeedOffset = 0;
  I2C_Init(pI2Cx, &I2C_InitStructure);
  I2C_SetTimeOutValue(pI2Cx, SystemCoreClock/2000);     /* 500us                                   */
  I2C_SetTimeOutPrescaler(pI2Cx,I2C_PRESCALER_128);     /* 500us*128 = 64 ms                       */
  I2C_TimeOutCmd(pI2Cx,ENABLE);                         /* Enable timeout function                 */
  /* Enable I2C0 & I2C1                                                                                     */
  I2C_Cmd(pI2Cx, ENABLE);

  /* Enable I2C Slave interrupts                                                                            */
  I2C_IntConfig(pI2Cx,
                I2C_INT_TOUT |
                I2C_INT_ARBLOS |
                I2C_INT_BUSERR |
                I2C_INT_ADRS |
                I2C_INT_TXDE |
                I2C_INT_RXNACK |
                I2C_INT_RXBF |
                I2C_INT_STO,
                ENABLE);
}

/*********************************************************************************************************//**
  * @brief  Tx is Finish.
  * @retval TRUE: Finish, False: Not Finish
  ***********************************************************************************************************/
bool I2CSlave::IsTxFinsh(void)
{
  return (txBuffer->available() == 0)? TRUE : FALSE;
}

/*********************************************************************************************************//**
  * @brief Preparing Tx Data.
  * @param txData: Tx data.
  * @param length: Tx data's length.
  * @retval None
  ***********************************************************************************************************/
bool I2CSlave::PrepareTxData(u8* txData, u32 length)
{
  while((txBuffer->available()<BufferSize) && (length>0))
  {
    txBuffer->store(*txData++);
    length--;
  }
  return (length==0)? TRUE : FALSE;
}

/*********************************************************************************************************//**
  * @brief This function handles I2C interrupt.
  * @retval None
  ***********************************************************************************************************/
void I2CSlave::I2C_IRQHandler(void (*ReceiveCB)(int), void (*RequestCB)(void))
{
  HT_I2C_TypeDef *pI2Cx = gI2CSlaveState.pI2Cx;
  u32 Result = I2C_ReadRegister(pI2Cx, I2C_REGISTER_SR);

  /*----------------------------- Error Process ----------------------------------------------------------  */
  if(Result & I2C_FLAG_BUSERR)
  {
    I2C_ClearFlag(pI2Cx, I2C_FLAG_BUSERR);
  }
  if(Result & I2C_FLAG_ARBLOS)
  {
    I2C_ClearFlag(pI2Cx, I2C_FLAG_ARBLOS);
  }
  if(Result & I2C_FLAG_TOUTF)
  {
    I2C_ClearFlag(pI2Cx, I2C_FLAG_TOUTF);
  }

  /*----------------------------- Stop Process ------------------------------------------------------------ */
  if( Result & I2C_FLAG_STO)
  {
    if(Result & I2C_FLAG_RXDNE)
    {
      u8 buff;
      buff = I2C_ReceiveData(pI2Cx);
      if(!rxBuffer->isFull())
      {
        rxBuffer->store(buff);
      }
    }
    if((Result == I2C_FLAG_STO) ||
       (Result ==  (I2C_FLAG_BUSBUSY|I2C_FLAG_STO|I2C_FLAG_ADRS)) ||
       (Result & I2C_FLAG_RXDNE))
    {
      I2C_AckCmd(pI2Cx, DISABLE);
      if((ReceiveCB) && (!getRequest))
      {
        ReceiveCB(rxBuffer->available());
      }
      gRestartFlag = FALSE;
      getRequest = FALSE;
      I2C_AckCmd(pI2Cx, ENABLE);
    }
  }
  else if (Result & I2C_FLAG_RXNACK)
  {
    /* Clear RXNACK flag                                                                                    */
    I2C_ClearFlag(pI2Cx, I2C_FLAG_RXNACK);
  }
  /*-----------------------------Rx Process --------------------------------------------------------------  */
  else if(Result & I2C_FLAG_RXDNE)
  {
    if((Result&I2C_FLAG_ADRS) || (Result&I2C_FLAG_TXDE)==0)
    {
      u8 buff;
      /* Restart Mode                                                                                       */
      gRestartFlag = TRUE;
      buff = I2C_ReceiveData(pI2Cx);
      if(!rxBuffer->isFull())
      {
        rxBuffer->store(buff);
      }
    }
    else if(Result&I2C_FLAG_RXBF)
    {
      /* Rx buffer full                                                                                     */
      u8 buff;
      buff = I2C_ReceiveData(pI2Cx);
      if(!rxBuffer->isFull())
      {
        rxBuffer->store(buff);
      }
    }
  }
  /*-----------------------------Tx Process --------------------------------------------------------------  */
  else if(Result & I2C_FLAG_TXDE)
  {
    u8 buff = 0;

    if(gRestartFlag == TRUE)
    {
      /* Restart Mode                                                                                       */
      if(ReceiveCB != NULL)
      {
        ReceiveCB(rxBuffer->available());
      }
      gRestartFlag = FALSE;
    }
    if(getRequest == FALSE)
    {
      txBuffer->clear();
      if(RequestCB)
      {
        RequestCB();
      }
      getRequest = TRUE;
    }
    if( txBuffer->available() )
    {
      buff = txBuffer->read();
    }
    I2C_SendData(pI2Cx, buff);
  }
  /*-----------------------------Receive Address----------------------------------------------------------- */
  else if (Result & I2C_FLAG_ADRS)
  {
  }
  /*-----------------------------Unkonw Status------------------------------------------------------------  */
  else
  {

    I2C_DeInit(pI2Cx);
    I2C_Configuration(gI2CSlaveState.pI2Cx,
                      gI2CSlaveState.I2C_IRQn,
                      gI2CSlaveState.uSlaveAddress);
  }
}
