/*********************************************************************************************************//**
 * @file    i2c_master.c
 * @version $Rev:: 1142         $
 * @date    $Date:: 2024-12-13 #$
 * @brief   The I2C Master (interrupt mode with buffer).
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
#include "i2c_master.h"


/** @addtogroup HT32_Series_Peripheral_Examples HT32 Peripheral Examples
  * @{
  */

/** @addtogroup I2C I2C
  * @{
  */

/** @addtogroup I2C_Master I2C Master
  * @{
  */

/* Settings ------------------------------------------------------------------------------------------------*/
#define I2C_INTERRUPT_ERROR_RESET_VALUE         (100) /*  100 times  */
#define I2C_IDEL_MAX_TIME                       (50)  /*  50us  */

/* Private types -------------------------------------------------------------------------------------------*/
/**
 * @brief I2C Master state variables
 */
/* Private constants ---------------------------------------------------------------------------------------*/
#define I2C_MASTER_RECEIVED_TXNACK    ((u32)0x003A0200)
#define I2C_MASTER_RECEIVED_RXNACK    ((u32)0x00180200)

/* Private variables ---------------------------------------------------------------------------------------*/
u32 gInterruptErrorCounter;

/* Global functions ----------------------------------------------------------------------------------------*/
I2CMaster::I2CMaster(void):
SendAddressState(0)
{
  memset(&gI2CMasterState, 0, sizeof(I2CMaster_StateTypeDef));
}

/*********************************************************************************************************//**
 * @brief Initial I2C Master.
 * @param CH: I2C Channel
 * @param uMasterAddress
 * @param uClockSpeed
 * @retval None
 ************************************************************************************************************/
void I2CMaster::I2C_Configuration(HT_I2C_TypeDef *pI2Cx, IRQn_Type I2C_IRQn,  u32 uMasterAddress, u32 uClockSpeed, u32 uSpeedOffset)
{
  /* !!! NOTICE !!!
     Notice that the local variable (structure) did not have an initial value.
     Please confirm that there are no missing members in the parameter settings below in this function.
  */
  I2C_InitTypeDef  I2C_InitStructure;
  I2CMaster_StateTypeDef *pI2CMasterState = &gI2CMasterState;
  pI2CMasterState->pI2Cx = pI2Cx;

  NVIC_EnableIRQ(I2C_IRQn);
  NVIC_SetPriority(I2C_IRQn, 2);


  /* I2C configuration                                                                                      */
  I2C_InitStructure.I2C_GeneralCall = DISABLE;
  I2C_InitStructure.I2C_AddressingMode = I2C_ADDRESSING_7BIT;
  I2C_InitStructure.I2C_Acknowledge = DISABLE;
  I2C_InitStructure.I2C_OwnAddress = uMasterAddress;
  I2C_InitStructure.I2C_Speed = uClockSpeed;
  I2C_InitStructure.I2C_SpeedOffset = uSpeedOffset;
  I2C_Init(pI2CMasterState->pI2Cx, &I2C_InitStructure);

  #if I2C_MASTER_HW_TIMEOUT_MODE
  /*Seting TimeOut Counter                                                                                  */
  I2C_SetTimeOutPrescaler(pI2CMasterState->pI2Cx, I2C_MASTER_HW_TIMEOUT_DIV << 16);
  I2C_SetTimeOutValue(pI2CMasterState->pI2Cx, I2C_MASTER_HW_TIMEOUT_BASE);

  /* Enable I2C                                                                                             */
  I2C_TimeOutCmd(pI2CMasterState->pI2Cx, ENABLE);
  I2C_IntConfig(pI2CMasterState->pI2Cx, I2C_INT_TOUT, ENABLE);
  #endif

  /* Enable I2C interrupts                                                                                  */
  I2C_IntConfig(pI2CMasterState->pI2Cx, I2C_INT_ARBLOS | I2C_INT_BUSERR | I2C_INT_STA | I2C_INT_ADRS | I2C_INT_RXNACK | I2C_INT_RXDNE, ENABLE);

  I2C_CombFilterCmd(pI2CMasterState->pI2Cx, ENABLE);
  I2C_SequentialFilterConfig(pI2CMasterState->pI2Cx, SEQ_FILTER_2_PCLK);
}

/*********************************************************************************************************//**
 * @brief I2C Write.
 * @param CH: I2C Channel
 * @param pI2CMaster:
 * @retval None
 ************************************************************************************************************/
void I2CMaster::I2CMaster_Write(I2CMaster_Typedef *pI2CMaster)
{
  pI2CMaster->Command = I2C_MASTER_CMD_WRITE;
  I2CMaster_Start(pI2CMaster);
}

/*********************************************************************************************************//**
 * @brief I2C Read.
 * @param CH: I2C Channel
 * @param pI2CMaster:
 * @retval None
 ************************************************************************************************************/
void I2CMaster::I2CMaster_Read(I2CMaster_Typedef *pI2CMaster)
{
  pI2CMaster->Command = I2C_MASTER_CMD_READ;
  I2CMaster_Start(pI2CMaster);
}

/*********************************************************************************************************//**
 * @brief I2C Write Read.
 * @param CH: I2C Channel
 * @param pI2CMaster:
 * @retval None
 ************************************************************************************************************/
void I2CMaster::I2CMaster_WriteRead(I2CMaster_Typedef *pI2CMaster)
{
  pI2CMaster->Command = I2C_MASTER_CMD_WRITEREAD;
  I2CMaster_Start(pI2CMaster);
}


/*********************************************************************************************************//**
 * @brief Start Read or Write of I2C buffer.
 * @param CH: I2C Channel
 * @param pI2CMaster: Pointer of I2CMaster_InitTypedef
 * @retval None
  ***********************************************************************************************************/
void I2CMaster::I2CMaster_Start(I2CMaster_Typedef *pI2CMaster)
{
  I2CMaster_StateTypeDef *pI2CMasterState = &gI2CMasterState;
  vu32 currentUs = 0;

  u32 _SR = pI2CMasterState->pI2Cx->SR;
#if I2C_MASTER_StART_TRANSFER_BUS_BUSY_CHECK
  u32 uCurrent;
#endif

/* =================ERROR PROCESS===================== */

  /* All conditions are clear by I2C Reset because of the Arbloss interrupt, then waitting for the bus idle. */
  if(pI2CMasterState->uIsBusArbLoss == TRUE)
  {
    /* Time required for detection. */
    currentUs = micros();
    /* Waiting for another master's communication. */
    while(micros()-currentUs < I2C_IDEL_MAX_TIME){
      if( pI2CMasterState->pI2Cx->SR == I2C_FLAG_BUSBUSY ){
        return ;
      }
    }
  }

  /* Check I2C's SR. if something gose wrong, reset this i2c. */
  if ((_SR & (I2C_FLAG_MASTER | I2C_FLAG_BUSBUSY)) == I2C_FLAG_BUSBUSY)
  {
      I2CMaster_ResetPeripheral();
  }

/* =================================================== */

  gInterruptErrorCounter = 0;
  pI2CMasterState->uTxIndex = 0;
  pI2CMasterState->uTxLength = pI2CMaster->Tx.uLength;
  pI2CMasterState->puTxBuffer = pI2CMaster->Tx.puBuffer;

  pI2CMasterState->uRxIndex = 0;
  pI2CMasterState->uRxLength = pI2CMaster->Rx.uLength;
  pI2CMasterState->puRxBuffer = pI2CMaster->Rx.puBuffer;

  pI2CMasterState->uSlaveAddress = pI2CMaster->uSlaveAddress;
  pI2CMasterState->Command = pI2CMaster->Command;

  pI2CMasterState->IsBusy = TRUE;
  pI2CMasterState->uIsAddrNACK = FALSE;
  pI2CMasterState->uIsDataNACK = FALSE;
  pI2CMasterState->uIsBusError = FALSE;
  pI2CMasterState->uIsBusArbLoss = FALSE;
  pI2CMasterState->uIsHardwareTimeout = FALSE;
  if (pI2CMaster->uTimeout_us)
  {
    bTimeoutStartFlag = true;
  }
  else
  {
    bTimeoutStartFlag = false;
  }
  if(bTimeoutStartFlag)
  {
    pI2CMasterState->sTimerCount = TRANSFER_TIMER(pI2CMaster->uTimeout_us);
    if (pI2CMasterState->sTimerCount <= 0)
    {
      while(1);   /* Please check uTimeout_us need more than I2C_MASTER_SW_TIMEOUT_BASE. */
    }
  }

  /* this flag avoided I2C busbusy when Pull-up was disabled.                                               */
  pI2CMasterState->sDynamicOnOffI2C = (u32)pI2CMaster->isDynamicOnOff_I2C;

  I2C_Cmd(pI2CMasterState->pI2Cx, ENABLE);

  #if (I2C_MASTER_SW_TIMEOUT_MODE == 2)
  pI2CMasterState->uStart = I2CMASTER_GETTICK();
  #endif

#if I2C_MASTER_StART_TRANSFER_BUS_BUSY_CHECK
  while (I2C_GetFlagStatus(pI2CMasterState->pI2Cx, I2C_FLAG_BUSBUSY) == SET)
  {
    #if (I2C_MASTER_SW_TIMEOUT_MODE == 2)
    if(bTimeoutStartFlag)
    {
      uCurrent = I2CMASTER_GETTICK();
      pI2CMasterState->sTimerCount -= DIFF_TICK(pI2CMasterState->uStart,uCurrent);
      pI2CMasterState->uStart = uCurrent;
    }
    #endif

    if (pI2CMasterState->sTimerCount <= 0)
    {
      return;
    }
  }
#endif
  I2C_ClearFlag(pI2CMasterState->pI2Cx, I2C_FLAG_RXNACK);
  I2C_IntConfig(pI2CMasterState->pI2Cx, I2C_INT_RXNACK, ENABLE);

  if (pI2CMaster->Command == I2C_MASTER_CMD_WRITE || pI2CMaster->Command == I2C_MASTER_CMD_WRITEREAD)
  {
    I2C_IntConfig(pI2CMasterState->pI2Cx, I2C_INT_TXDE, ENABLE);
    pI2CMasterState->uI2CReadWrite = I2C_MASTER_WRITE;
  }
  else
  {
    pI2CMasterState->uI2CReadWrite = I2C_MASTER_READ;
  }

  I2C_TargetAddressConfig(pI2CMasterState->pI2Cx, pI2CMasterState->uSlaveAddress, pI2CMasterState->uI2CReadWrite);

  return;
}

/*********************************************************************************************************//**
 * @brief Get I2C buffer statue.
 * @param CH: I2C Channel
 * @retval Result: I2C_MASTER_FINISH, I2C_MASTER_HW_TIMEOUT or I2C_MASTER_SW_TIMEOUT
 ************************************************************************************************************/
I2CMaster::I2CMaster_Status_Enum I2CMaster::I2CMaster_GetStatus()
{
  I2CMaster_StateTypeDef *pI2CMasterState = &gI2CMasterState;
  I2CMaster_Status_Enum Result = I2C_MASTER_GOING;
  #if (I2C_MASTER_NACK_AUTO_RETRY == 1)
  u32 i;
  #endif

  #if (I2C_MASTER_SW_TIMEOUT_MODE == 2)
  if(bTimeoutStartFlag)
  {
    u32 uCurrent;
    uCurrent = I2CMASTER_GETTICK();
    pI2CMasterState->sTimerCount -= DIFF_TICK(pI2CMasterState->uStart,uCurrent);
    pI2CMasterState->uStart = uCurrent;
  }
  #endif

  if (pI2CMasterState->uIsBusError == TRUE)
  {
    return I2C_MASTER_BUSERR;
  }

  if (pI2CMasterState->uIsHardwareTimeout == TRUE)
  {
    return I2C_MASTER_HW_TIMEOUT;
  }

  if (pI2CMasterState->IsBusy == FALSE)
  {
    Result = I2C_MASTER_FINISH;
  }
  if (pI2CMasterState->sTimerCount <= 0)
  {
    if(bTimeoutStartFlag)
    {
      pI2CMasterState->IsBusy = FALSE;
      Result = I2C_MASTER_SW_TIMEOUT;
      I2C_IntConfig(pI2CMasterState->pI2Cx, I2C_INT_RXNACK, DISABLE);
      I2C_ClearFlag(pI2CMasterState->pI2Cx, I2C_FLAG_RXNACK);
    }
  }
  if (pI2CMasterState->uIsDataNACK == TRUE)
  {
    pI2CMasterState->uIsDataNACK = FALSE;

    #if (I2C_MASTER_NACK_AUTO_RETRY == 0)
    Result = I2C_MASTER_DATA_NACK;
    #else
    /* Release I2C bus for other master by adding a delay                                                   */
    for (i = 0; i < 500; i++);

    /* Re-Write Slave address for start Tx or Rx (re-try when Slave not ack)                                */
    I2C_TargetAddressConfig(pI2CMasterState->pI2Cx, pI2CMasterState->uSlaveAddress, pI2CMasterState->uI2CReadWrite);
    #endif
  }
  else if (pI2CMasterState->uIsAddrNACK == TRUE)
  {
    pI2CMasterState->uIsAddrNACK = FALSE;

    #if (I2C_MASTER_NACK_AUTO_RETRY == 0)
    Result = I2C_MASTER_ADDR_NACK;
    #else
    /* Release I2C bus for other master by adding a delay                                                   */
    for (i = 0; i < 500; i++);

    /* Re-Write Slave address for start Tx or Rx (re-try when Slave not ack)                                */
    I2C_TargetAddressConfig(pI2CMasterState->pI2Cx,
                            pI2CMasterState->uSlaveAddress,
                            pI2CMasterState->uI2CReadWrite);
    #endif
  }

  return Result;
}

/*********************************************************************************************************//**
 * @brief Get I2C Rx index.
 * @param CH: I2C Channel
 * @retval None
 ************************************************************************************************************/
u32 I2CMaster::I2CMaster_GetRxIndex()
{
  I2CMaster_StateTypeDef *pI2CMasterState = &gI2CMasterState;
  return pI2CMasterState->uRxIndex;
}

/*********************************************************************************************************//**
 * @brief Update I2C Rx length.
 * @param CH: I2C Channel
 * @retval None
 ************************************************************************************************************/
void I2CMaster::I2CMaster_UpdateRxLength(u32 uLnegth)
{
  I2CMaster_StateTypeDef *pI2CMasterState = &gI2CMasterState;
  pI2CMasterState->uRxLength = uLnegth;
  return;
}

/*********************************************************************************************************//**
 * @brief   This function handles I2C interrupt.
 * @param  CH: I2C Channel
 * @retval  None
 ************************************************************************************************************/
extern volatile uint32_t orinWireCause, orinWireStatus, orinWireUnknownCount;
void I2CMaster::I2C_IRQHandler(void)
{
  I2CMaster_StateTypeDef *pI2CMasterState = &gI2CMasterState;
  u32 uResult;

  uResult = I2C_ReadRegister(pI2CMasterState->pI2Cx, I2C_REGISTER_SR);

  // Snapshot before any flag clear or peripheral reset; preserve simultaneous flags.
  if(uResult & I2C_FLAG_BUSERR){orinWireCause|=1;orinWireStatus|=uResult;}
  if(uResult & I2C_FLAG_ARBLOS){orinWireCause|=2;orinWireStatus|=uResult;}
  /*----------------------------- Error Process ----------------------------------------------------------  */
  if (uResult & I2C_FLAG_BUSERR)
  {
    pI2CMasterState->uIsBusError = TRUE;
    I2C_ClearFlag(pI2CMasterState->pI2Cx, I2C_FLAG_BUSERR);
    return ;
  }

  if (uResult & I2C_FLAG_ARBLOS)
  {
    /* RESET I2C IP */
    I2CMaster_ResetPeripheral();
    I2C_Cmd(pI2CMasterState->pI2Cx, ENABLE);

    /* Change the Error and ArbLoss Flag */
    pI2CMasterState->uIsBusError = TRUE;
    pI2CMasterState->uIsBusArbLoss = TRUE;

    return;
  }

  if(uResult & I2C_FLAG_TOUTF)
  {
    pI2CMasterState->uIsHardwareTimeout = TRUE;
    I2C_ClearFlag(pI2CMasterState->pI2Cx, I2C_FLAG_TOUTF);
    return;
  }

  if (uResult == I2C_FLAG_RXNACK)
  {
    pI2CMasterState->uIsDataNACK = TRUE;
    I2C_ClearFlag(pI2CMasterState->pI2Cx, I2C_FLAG_RXNACK);
    return;
  }

  switch (uResult)
  {
    case I2C_MASTER_SEND_START:
    {
      SendAddressState = 1;
      break;
    }
    /*-----------------------------Master Receiver ---------------------------------------------------------*/
    case I2C_MASTER_RECEIVER_MODE:
    {
      SendAddressState = 2;
      /* Enable I2C ACK for receiving data                                                                  */
      if (pI2CMasterState->uRxLength > 1)
      {
        I2C_AckCmd(pI2CMasterState->pI2Cx, ENABLE);
      }
      break;
    }
    case I2C_MASTER_RX_NOT_EMPTY_NOBUSY:
    {
      SendAddressState = 2;
      if (pI2CMasterState->uRxIndex == pI2CMasterState->uRxLength - 2)
      {
        I2C_AckCmd(pI2CMasterState->pI2Cx, DISABLE);
      }
      *(pI2CMasterState->puRxBuffer + pI2CMasterState->uRxIndex) = I2C_ReceiveData(pI2CMasterState->pI2Cx);
      pI2CMasterState->uRxIndex++;
      break;
    }
    case I2C_MASTER_RX_BUFFER_FULL:
    case I2C_MASTER_RX_NOT_EMPTY:
    {
      SendAddressState = 2;
      if (pI2CMasterState->uRxIndex == pI2CMasterState->uRxLength - 2)
      {
        I2C_AckCmd(pI2CMasterState->pI2Cx, DISABLE);
      }
      /* Receive data sent from I2C                                                                         */
      *(pI2CMasterState->puRxBuffer + pI2CMasterState->uRxIndex) = I2C_ReceiveData(pI2CMasterState->pI2Cx);
      pI2CMasterState->uRxIndex++;

      if (pI2CMasterState->uRxIndex == pI2CMasterState->uRxLength)
      {
        /* Clear IsBusy flag to terminate polling of I2CMaster_GetStatus                                    */
        pI2CMasterState->IsBusy = FALSE;

        if (pI2CMasterState->sDynamicOnOffI2C)
        {
          if (I2CMaster_DynamicTimeout() == I2C_MASTER_SW_TIMEOUT)
          {
            return;
          }
          I2C_Cmd(pI2CMasterState->pI2Cx, DISABLE);
        }
      }
      break;
    }

    /*-----------------------------Master Transmitter ------------------------------------------------------*/
    case I2C_MASTER_TRANSMITTER_MODE:
    case I2C_MASTER_TX_EMPTY:
    {
      SendAddressState = 3;
      if (pI2CMasterState->uTxLength != 0)
      {
        I2C_SendData(pI2CMasterState->pI2Cx, *(pI2CMasterState->puTxBuffer + pI2CMasterState->uTxIndex));
        pI2CMasterState->uTxIndex++;
        pI2CMasterState->uTxLength--;
      }
      else
      {
        I2C_IntConfig(pI2CMasterState->pI2Cx, I2C_INT_TXDE, DISABLE);
        if (pI2CMasterState->Command == I2C_MASTER_CMD_WRITE)
        {
          /* Clear IsBusy flag to terminate polling of I2CMaster_GetStatus                                  */
          pI2CMasterState->IsBusy = FALSE;
          if (pI2CMasterState->sDynamicOnOffI2C)
          {
            if (I2CMaster_DynamicTimeout() == I2C_MASTER_SW_TIMEOUT)
            {
              return;
            }
            I2C_Cmd(pI2CMasterState->pI2Cx, DISABLE);
          }
        }
        else if (pI2CMasterState->Command == I2C_MASTER_CMD_WRITEREAD)
        {
          pI2CMasterState->uI2CReadWrite = I2C_MASTER_READ;
          I2C_TargetAddressConfig(pI2CMasterState->pI2Cx, pI2CMasterState->uSlaveAddress, pI2CMasterState->uI2CReadWrite);
        }
      }
      break;
    }

    /* Received NACK when Read or Write mode                                                                */
    case I2C_MASTER_RECEIVED_RXNACK:
    case I2C_MASTER_RECEIVED_TXNACK:
    {
      I2C_ClearFlag(pI2CMasterState->pI2Cx, I2C_FLAG_RXNACK);

      if (pI2CMasterState->sDynamicOnOffI2C)
      {
        if (I2CMaster_DynamicTimeout() == I2C_MASTER_SW_TIMEOUT)
        {
          return;
        }
        I2C_Cmd(pI2CMasterState->pI2Cx, DISABLE);
      }
      /* Set uIsNACK flag for I2CMaster_GetStatus                                                           */
      if(SendAddressState == 1)
      {
        SendAddressState = 0;
        pI2CMasterState->uIsAddrNACK = TRUE;
      }
      else
        pI2CMasterState->uIsDataNACK = TRUE;
      break;
    }
    default:
    {
      if((uResult & (I2C_FLAG_STA | I2C_FLAG_STO | I2C_FLAG_ADRS | I2C_FLAG_GCS)) > 0)
      {
        break;
      }
      /* Unknown uResult, error gInterruptErrorCounter */
      gInterruptErrorCounter++;
      orinWireUnknownCount++;
      if(gInterruptErrorCounter >= I2C_INTERRUPT_ERROR_RESET_VALUE)
      {
        orinWireCause|=4;orinWireStatus|=uResult;
        /* The gInterruptErrorCounter exceeds the threshold, reset the I2C */
        gInterruptErrorCounter = 0;
        I2CMaster_ResetPeripheral();
        pI2CMasterState->uIsBusError = TRUE;
      }
      break;
    }
  }
}

/*********************************************************************************************************//**
 * @brief I2C timeout handler.
 * @retval None
 ************************************************************************************************************/
void I2CMaster::I2CMaster_TimeoutHandler(void)
{
  if (gI2CMasterState.sTimerCount > 0)
  {
    gI2CMasterState.sTimerCount--;
  }
}

/*********************************************************************************************************//**
 * @brief I2C Dynmaic Timeout Process.
 * @retval None
 ************************************************************************************************************/
I2CMaster::I2CMaster_Status_Enum I2CMaster::I2CMaster_DynamicTimeout()
{
  I2CMaster_StateTypeDef *pI2CMasterState = &gI2CMasterState;

  while (I2C_GetFlagStatus(pI2CMasterState->pI2Cx, I2C_FLAG_BUSBUSY) == SET)
  {
    #if (I2C_MASTER_SW_TIMEOUT_MODE == 2)
    if(bTimeoutStartFlag)
    {
      u32 uCurrent;
      uCurrent = I2CMASTER_GETTICK();
      pI2CMasterState->sTimerCount -= DIFF_TICK(pI2CMasterState->uStart,uCurrent);
      pI2CMasterState->uStart = uCurrent;
    }
    #endif

    if (pI2CMasterState->sTimerCount <= 0) // May not work if timer's interrupt priority is lower than I2C
    {
      return I2C_MASTER_SW_TIMEOUT;
    }
  }
  return I2C_MASTER_GOING;
}

/*********************************************************************************************************//**
 * @brief   This function Reset I2C peripheral.
 * @retval  None
 ************************************************************************************************************/
void I2CMaster::I2CMaster_ResetPeripheral(void)
{
  I2CMaster_StateTypeDef *pI2CMasterState = &gI2CMasterState;
  uint32_t SHPGR = pI2CMasterState->pI2Cx->SHPGR;
  uint32_t SLPGR = pI2CMasterState->pI2Cx->SLPGR;
  I2C_DeInit(pI2CMasterState->pI2Cx);
  I2C_Configuration( gI2CMasterState.pI2Cx, gI2CMasterState.I2C_IRQn, 0x01, 100000L, WIRE_MASTER_SPEED_OFFSET);
  pI2CMasterState->pI2Cx->SHPGR = SHPGR;
  pI2CMasterState->pI2Cx->SLPGR = SLPGR;
}

/* Private functions ---------------------------------------------------------------------------------------*/

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

volatile uint32_t orinWireCause=0, orinWireStatus=0, orinWireUnknownCount=0;
