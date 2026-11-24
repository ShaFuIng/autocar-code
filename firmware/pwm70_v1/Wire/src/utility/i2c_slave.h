/*********************************************************************************************************//**
 * @file    i2c_slave.h
 * @version $Rev:: 502          $
 * @date    $Date:: 2022-11-21 #$
 * @brief   The header file of I2C Slave (interrupt mode with buffer).
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
/* Define to prevent recursive inclusion -------------------------------------------------------------------*/
#ifndef __I2C_SLAVE_H
#define __I2C_SLAVE_H

/* Includes ------------------------------------------------------------------------------------------------*/
#include "Arduino.h"

/* Settings ------------------------------------------------------------------------------------------------*/
#define BufferSize             33

/* Exported typedef ----------------------------------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------------------------------------*/
class I2CSlave {
  typedef struct
  {
    HT_I2C_TypeDef *pI2Cx;
    IRQn_Type I2C_IRQn;
    u32 uSlaveAddress;
  } I2CSlave_StateTypeDef;
  public:
    /* RX Buffer */
    bool IsTxFinsh(void);
    bool PrepareTxData(u8* txData, u32 length);
    void I2C_Configuration(HT_I2C_TypeDef *pI2Cx,
                           IRQn_Type I2C_IRQn,
                           RingBufferN<BufferSize> *txBuffer,
                           RingBufferN<BufferSize> *rxBuffer,
                           u32 address);
    void I2C_Configuration(HT_I2C_TypeDef *pI2Cx,
                           IRQn_Type I2C_IRQn,
                           u32 address);
    void I2C_IRQHandler(void (*ReceiveCB)(int), void (*RequestCB)(void));
  private:

/* Private variables ---------------------------------------------------------------------------------------*/
    I2CSlave_StateTypeDef  gI2CSlaveState;
    volatile bool getRequest = FALSE;
    volatile bool gRestartFlag = FALSE;
    /* TX buffer */
    RingBufferN<BufferSize> *txBuffer;
    RingBufferN<BufferSize> *rxBuffer;
};


#endif
