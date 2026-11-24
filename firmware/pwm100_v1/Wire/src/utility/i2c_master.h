/*********************************************************************************************************//**
 * @file    i2c_master.h
 * @version $Rev:: 1142         $
 * @date    $Date:: 2024-12-13 #$
 * @brief   The header file of I2C Master (interrupt mode with buffer).
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
// <<< Use Configuration Wizard in Context Menu >>>

/* Define to prevent recursive inclusion -------------------------------------------------------------------*/
#ifndef __I2C_MASTER_H
#define __I2C_MASTER_H

/* Includes ------------------------------------------------------------------------------------------------*/

#include "Arduino.h"

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
//<q0> Enable Master NACK Auto Retry
// <i> Master NACK auto retry by I2CMaster_GetStatus().
//<q1> Enable Clock Generated
// <i> Master generates clock in the initialization phase.

//<e2> Enable I2C Hardware Time-out Mode Configuration
// <i> The hardware time-out calculator is:
// <i> F_PCLK = CK_AHB / (2 ^ I2CPCLK)    (I2CPCLK: APB Peripheral Clock Prescaler)
// <i> F_I2CTO = F_PCLK / (2 ^ PSC)
// <i> Time-out = TOUT / F_I2CTO
//    <o3> Counter Prescaler Selection (2 ^ PSC)
//       <0=> /1
//       <1=> /2
//       <2=> /4
//       <3=> /8
//       <4=> /16
//       <5=> /32
//       <6=> /64
//       <7=> /128
//     <i> F_I2CTO = F_PCLK / (2 ^ PSC)
//    <o4> Counter Preload Value (TOUT) <1-0xFFFF:1>
//     <i> Time-out = TOUT / F_I2CTO
//     <i> TOUT range of (1 ~ 0xFFFF).
//</e>

//<e5> Enable I2C Software Time-out Mode configuration
// <i> The software time-out configuration is based on Interrupt or Tick mode.
//    <o5> I2C Software Time-out Mode Selection
//       <1=> Interrupt Mode
//       <2=> Tick Mode
//       <i> 1 = Interrupt Mode
//       <i> 2 = Tick Mode
//    <o6> Time-out Timer Base (Hz) <1-0xFFFFFFFF:1>
//       <i> Please follow the external interrupt / tick source parameter.
//       <i> (Tick refer to "ht32f5xxxx_conf.h")
//</e>
#define I2C_MASTER_StART_TRANSFER_BUS_BUSY_CHECK   (0) /* Auto check whether the BUS is busy at the beginning of the transmission. */
#define I2C_MASTER_NACK_AUTO_RETRY          (0)     /* Auto retry when NACK by I2CMaster_GetStatus()        */
#define I2C_MASTER_INITIAL_BUS              (0)     /* Master generates clock in the initialization phase.  */

#define I2C_MASTER_HW_TIMEOUT_MODE          (0)     /* 0: DISABLE, 1: ENABLE                                */
#define I2C_MASTER_HW_TIMEOUT_DIV           (7)     /* Timeout Clock = TOUT / (Core Clock / (2 ^ I2C_MASTER_HW_TIMEOUT_DIV))  */
#define I2C_MASTER_HW_TIMEOUT_BASE          (SystemCoreClock/2000)  /* TOUT¡G500us*128=64ms                */
#define I2C_MASTER_SW_TIMEOUT_MODE          (2)     /* 1: Interrupt, 2: TimeTick                            */
#define I2C_MASTER_SW_TIMEOUT_BASE          (1000000)


#if (I2C_MASTER_SW_TIMEOUT_MODE == 2)
#define I2CMASTER_GETTICK   micros            /* Call from external function                          */
#endif


/* Exported typedef ----------------------------------------------------------------------------------------*/



/* Exported macro ------------------------------------------------------------------------------------------*/

#define DIFF_TICK(start,current)      ((current) >= (start) ? (u32)(current - start) : (u32)(0xFFFFFFFF - start + current + 1))

#if (I2C_MASTER_SW_TIMEOUT_BASE >= 1000000)
#define TRANSFER_TIMER(us)            (u32) (us * (I2C_MASTER_SW_TIMEOUT_BASE / 1000000))
#else
#define TRANSFER_TIMER(us)            (u32) (us / (1000000 / I2C_MASTER_SW_TIMEOUT_BASE))
#endif

class I2CMaster {
  private:
  /**
   * @brief I2C Master command
   */
  typedef enum
  {
    I2C_MASTER_CMD_READ      = 0,
    I2C_MASTER_CMD_WRITE     = 1,
    I2C_MASTER_CMD_WRITEREAD = 2,
    I2C_MASTER_CMD_READWRITE = 3,
  } I2CMaster_CMD_Enum;

  /**
   * @brief I2C Master buffer struct
   */
  typedef struct
  {
    u8 *puBuffer;
    u32 uLength;
  } I2CMaster_BufferTypedef;


  typedef struct
  {
    HT_I2C_TypeDef *pI2Cx;
    IRQn_Type I2C_IRQn;
    u32 uTxIndex;
    u32 uTxLength;
    u8 *puTxBuffer;

    u32 uRxIndex;
    u32 uRxLength;
    u8 *puRxBuffer;

    u32 uSlaveAddress;
    I2CMaster_CMD_Enum Command;
    u32 uI2CReadWrite;
    vu32 IsBusy;
    vu32 uIsDataNACK;
    vu32 uIsAddrNACK;
    vu32 uIsBusError;
    vu32 uIsBusArbLoss;
    vu32 uIsHardwareTimeout;
    vu32 uStart;
    vs32 sTimerCount;
    vs32 sDynamicOnOffI2C;
  } I2CMaster_StateTypeDef;
  I2CMaster_StateTypeDef gI2CMasterState;
  u8 SendAddressState;
  bool bTimeoutStartFlag;
  public:
  /**
   * @brief I2C Master struct
   */
  typedef struct
  {
    u32 uSlaveAddress;
    I2CMaster_BufferTypedef Tx;
    I2CMaster_BufferTypedef Rx;
    u32 uTimeout_us;
    I2CMaster_CMD_Enum Command;
    bool isDynamicOnOff_I2C;
  } I2CMaster_Typedef;
  /**
   * @brief I2C Master status
   */
  typedef enum
  {
    I2C_MASTER_GOING        = 0,
    I2C_MASTER_FINISH       = 1,
    I2C_MASTER_SW_TIMEOUT   = 2,
    I2C_MASTER_DATA_NACK    = 3,
    I2C_MASTER_BUSERR       = 4,
    I2C_MASTER_HW_TIMEOUT   = 5,
    I2C_MASTER_ADDR_NACK    = 6,
  } I2CMaster_Status_Enum;
    /* Exported functions ----------------------------------------------------------------------------------*/
    I2CMaster();
    void I2C_Configuration(HT_I2C_TypeDef *pI2Cx, IRQn_Type I2C_IRQn,  u32 uMasterAddress, u32 uClockSpeed, u32 uSpeedOffset);
    void I2CMaster_Write(I2CMaster_Typedef *pI2CMaster);
    void I2CMaster_Read(I2CMaster_Typedef *pI2CMaster);
    void I2CMaster_WriteRead(I2CMaster_Typedef *pI2CMaster);
    void I2CMaster_Start(I2CMaster_Typedef *pI2CMaster);
    I2CMaster_Status_Enum I2CMaster_GetStatus();
    u32 I2CMaster_GetRxIndex();
    void I2CMaster_UpdateRxLength(u32 uLnegth);
    void I2CMaster_ResetPeripheral(void);
    void I2CMaster_TimeoutHandler(void);
    I2CMaster_Status_Enum I2CMaster_DynamicTimeout();
    void I2C_IRQHandler(void);
};


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif
