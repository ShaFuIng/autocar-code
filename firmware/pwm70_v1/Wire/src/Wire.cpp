/*
  TwoWire.cpp - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
*/

/* Includes ------------------------------------------------------------------------------------------------*/
#include "Wire.h"
#include "utility/i2c_master.h"
#include "utility/i2c_slave.h"
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

/* Setting -------------------------------------------------------------------------------------------------*/
#define WIRE_TIMEOUT_DEFAULT     25000
#define WIRE_SCL_SPEED_DEFAULT   100000L

typedef void (*RequestTypedef)(void);
typedef void (*ReceiveTypedef)(int);
/* Private constants ---------------------------------------------------------------------------------------*/
/* Distinguish master from slave. */
/* 0x01 is a reserved value, and thus cannot be used by slave devices */
static const uint8_t gMASTER_ADDRESS = 0x01;

static RingBufferN<_I2C_TXRX_BUFFER_SIZE> gRxBuffer[I2C_PORT_NUM];
static RingBufferN<_I2C_TXRX_BUFFER_SIZE> gTxBuffer[I2C_PORT_NUM];
static TwoWire::I2CPortStatus gPortStatusTable[I2C_PORT_NUM];

void TwoWire::_errorHandling(void)
{
    {
      volatile int i = 0;
      volatile int tm = 0;
      uint32_t scl_pin = INF_GET_I2C_SCL(this->arrayIndex);
      uint32_t sda_pin = INF_GET_I2C_SDA(this->arrayIndex);
      AFIO_GPxConfig(digitalPinToID(sda_pin), digitalPinToBitMask(sda_pin), AFIO_FUN_GPIO);
      GPIO_InputConfig(digitalPinToPort(sda_pin), digitalPinToBitMask(sda_pin), ENABLE);

      AFIO_GPxConfig(digitalPinToID(scl_pin), digitalPinToBitMask(scl_pin), AFIO_FUN_GPIO);
      GPIO_SetOutBits(digitalPinToPort(scl_pin), digitalPinToBitMask(scl_pin));
      GPIO_DirectionConfig(digitalPinToPort(scl_pin), digitalPinToBitMask(scl_pin), GPIO_DIR_OUT);
      for (; i < 10; i++)
      {
        if(GPIO_ReadInBit(digitalPinToPort(sda_pin), digitalPinToBitMask(sda_pin)) != RESET)
        {
          return;
        }
        GPIO_ClearOutBits(digitalPinToPort(scl_pin), digitalPinToBitMask(scl_pin));
        for (tm = 0; tm < 2000; tm++)
        __NOP();
        GPIO_SetOutBits(digitalPinToPort(scl_pin), digitalPinToBitMask(scl_pin));
        for (tm = 0; tm < 2000; tm++)
        __NOP();
      }
    }
}
/*********************************************************************************************************//**
 * @brief  Constructor of I2C's Class.
 * @param  arrayIndex
 * @retval None
 ************************************************************************************************************/
TwoWire::TwoWire(u32 arrayIndex)
{
  this->arrayIndex = arrayIndex;
  this->pI2Cx =  INF_GET_PORT(pI2CInfo, arrayIndex);
  this->bufferIndex = INF_GET_BUFFER_INDEX(pI2CInfo, arrayIndex);
  this->twi_timeout_us = WIRE_TIMEOUT_DEFAULT;
}

/*********************************************************************************************************//**
 * @brief Initiate the Wire library and join the I2C bus as a master or slave. This should normally be called
          only once.
 * @param address: the 7-bit slave address (optional); if not specified, join the bus as a master.
 * @retval None
 ************************************************************************************************************/
void TwoWire::begin(uint8_t address)
{
  I2CPortStatus _I2cStatus = gPortStatusTable[this->bufferIndex];
  IRQn_Type I2C_IRQn = INF_GET_IRQ(pI2CInfo,this->arrayIndex);
  uint32_t scl_pin = INF_GET_I2C_SCL(this->arrayIndex);
  uint32_t sda_pin = INF_GET_I2C_SDA(this->arrayIndex);
  device_address = address;
  if(_I2cStatus == UNKNOW)
  {
    this->I2cStatus = (address == gMASTER_ADDRESS) ? MASTER_MODE : SLAVE_MODE;
    gPortStatusTable[this->bufferIndex] = this->I2cStatus;
  }
  else
  {
    if(this->I2cStatus == UNKNOW)
    {
      SetErrId(API_ERROR_HARDWARE_SRC_OCCUPIED);
      return;
    }
  }

  this->ownAddress   = address;
  this->transmitting = 0;

  /* Configure GPIO to I2C mode for Slave                                                                   */
  AFIO_GPxConfig(digitalPinToID(scl_pin), digitalPinToBitMask(scl_pin), AFIO_FUN_I2C);
  AFIO_GPxConfig(digitalPinToID(sda_pin), digitalPinToBitMask(sda_pin), AFIO_FUN_I2C);
  if(this->I2cStatus == MASTER_MODE)
  {
    GPIO_PullResistorConfig(digitalPinToPort(scl_pin), digitalPinToBitMask(scl_pin), GPIO_PR_UP);
    GPIO_PullResistorConfig(digitalPinToPort(sda_pin), digitalPinToBitMask(sda_pin), GPIO_PR_UP);
    this->master.I2C_Configuration( this->pI2Cx, I2C_IRQn, this->ownAddress, WIRE_SCL_SPEED_DEFAULT , WIRE_MASTER_SPEED_OFFSET);
    this->setClock(WIRE_SCL_SPEED_DEFAULT);
  }
  else
  {
    GPIO_PullResistorConfig(digitalPinToPort(scl_pin), digitalPinToBitMask(scl_pin), GPIO_PR_DISABLE);
    GPIO_PullResistorConfig(digitalPinToPort(sda_pin), digitalPinToBitMask(sda_pin), GPIO_PR_DISABLE);
    this->slave.I2C_Configuration( this->pI2Cx, I2C_IRQn, &gTxBuffer[this->bufferIndex], &gRxBuffer[this->bufferIndex], this->ownAddress);
  }
}

void TwoWire::begin(int address)
{
  begin((uint8_t)address);
}

void TwoWire::begin()
{
  begin(gMASTER_ADDRESS);
}

/*********************************************************************************************************//**
 * @brief Stop I2C.
 * @retval None
 ************************************************************************************************************/
void TwoWire::end(void)
{
  if(this->I2cStatus == UNKNOW)
  {
    SetErrId(API_ERROR_NO_INITALIZED);
    return;
  }
  uint32_t scl_pin = INF_GET_I2C_SCL(this->arrayIndex);
  uint32_t sda_pin = INF_GET_I2C_SDA(this->arrayIndex);

  I2C_DeInit(this->pI2Cx);
  AFIO_GPxConfig(digitalPinToID(scl_pin), digitalPinToBitMask(scl_pin), AFIO_FUN_DEFAULT);
  AFIO_GPxConfig(digitalPinToID(sda_pin), digitalPinToBitMask(sda_pin), AFIO_FUN_DEFAULT);

  gPortStatusTable[this->bufferIndex] = UNKNOW;
  this->I2cStatus = UNKNOW;
}

/*********************************************************************************************************//**
 * @brief This function modifies the clock frequency for I2C communication.
 * @param frequency: clockFrequency: the value (in Hertz) of desired communication clock.
 * @retval None
 ************************************************************************************************************/
void TwoWire::setClock(uint32_t frequency)
{
  u32 PCLK_Freq = 0;
  s32 sTmp = 0;
  s32 SHPGR = 0;
  s32 SLPGR = 0;
  u32 OffsetValue;
  bool isAPBPrescalerDiv2 = false;
  bool isAPBPrescalerDiv4 = false;
  if(this->I2cStatus != MASTER_MODE)
  {
    return;
  }
  #if defined(WIRE_MASTER_PRESCALER_4_THRSHOLD)
  if(frequency <= WIRE_MASTER_PRESCALER_4_THRSHOLD)
  {
    isAPBPrescalerDiv4 = true;
  }
  #endif
  #if defined(WIRE_MASTER_PRESCALER_2_THRSHOLD)
  if(frequency <= WIRE_MASTER_PRESCALER_2_THRSHOLD)
  {
    isAPBPrescalerDiv2 = true;
  }
  #endif
  #if defined(HT_I2C0)
  if (this->pI2Cx == HT_I2C0)
  {
    if(isAPBPrescalerDiv4)
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C0, CKCU_APBCLKPRE_DIV4);
    else if(isAPBPrescalerDiv2)
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C0, CKCU_APBCLKPRE_DIV2);
    else
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C0, CKCU_APBCLKPRE_DIV1);

    PCLK_Freq = CKCU_GetPeripFrequency(CKCU_PCLK_I2C0);
  }
  #endif
  #if defined(HT_I2C1)
  else if (this->pI2Cx == HT_I2C1)
  {
    if(isAPBPrescalerDiv4)
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C1, CKCU_APBCLKPRE_DIV4);
    else if(isAPBPrescalerDiv2)
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C1, CKCU_APBCLKPRE_DIV2);
    else
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C1, CKCU_APBCLKPRE_DIV1);

    PCLK_Freq = CKCU_GetPeripFrequency(CKCU_PCLK_I2C1);
  }
  #endif
  #if defined(HT_I2C2)
  else if (this->pI2Cx == HT_I2C2)
  {
    if(isAPBPrescalerDiv4)
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C2, CKCU_APBCLKPRE_DIV4);
    else if(isAPBPrescalerDiv2)
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C2, CKCU_APBCLKPRE_DIV2);
    else
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C2, CKCU_APBCLKPRE_DIV1);

    PCLK_Freq = CKCU_GetPeripFrequency(CKCU_PCLK_I2C2);
  }
  #endif
  #if defined(HT_I2C3)
  else if (this->pI2Cx == HT_I2C3)
  {
    if(isAPBPrescalerDiv4)
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C3, CKCU_APBCLKPRE_DIV4);
    else if(isAPBPrescalerDiv2)
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C3, CKCU_APBCLKPRE_DIV2);
    else
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C3, CKCU_APBCLKPRE_DIV1);

    PCLK_Freq = CKCU_GetPeripFrequency(CKCU_PCLK_I2C3);
  }
  #endif
  #if defined(HT_I2C4)
  else if (this->pI2Cx == HT_I2C4)
  {
    if(isAPBPrescalerDiv4)
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C4, CKCU_APBCLKPRE_DIV4);
    else if(isAPBPrescalerDiv2)
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C4, CKCU_APBCLKPRE_DIV2);
    else
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C4, CKCU_APBCLKPRE_DIV1);

    PCLK_Freq = CKCU_GetPeripFrequency(CKCU_PCLK_I2C4);
  }
  #endif
  #if defined(HT_I2C5)
  else if (this->pI2Cx == HT_I2C5)
  {
    if(isAPBPrescalerDiv4)
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C5, CKCU_APBCLKPRE_DIV4);
    else if(isAPBPrescalerDiv2)
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C5, CKCU_APBCLKPRE_DIV2);
    else
      CKCU_SetPeripPrescaler(CKCU_PCLK_I2C5, CKCU_APBCLKPRE_DIV1);

    PCLK_Freq = CKCU_GetPeripFrequency(CKCU_PCLK_I2C5);
  }
  #endif
  switch (this->pI2Cx->CR & 0xC000)
  {
    case 0:
    {
      sTmp = 6;
      break;
    }
    case 0x4000:
    {
      sTmp = 8;
      break;
    }
    case 0x8000:
    {
      sTmp = 9;
      break;
    }
  }

  if(isAPBPrescalerDiv4)
    OffsetValue = WIRE_MASTER_SPEED_OFFSET / 4;
  else if(isAPBPrescalerDiv2)
    OffsetValue = WIRE_MASTER_SPEED_OFFSET / 2;
  else
    OffsetValue = WIRE_MASTER_SPEED_OFFSET;

  SHPGR = (PCLK_Freq * 9)/(frequency * 20) - sTmp - OffsetValue;
  SLPGR = (PCLK_Freq * 11)/(frequency * 20) - sTmp - OffsetValue;

  SHPGR = (SHPGR < 0) ? 0 : SHPGR;
  SLPGR = (SLPGR < 0) ? 0 : SLPGR;

  this->pI2Cx->SHPGR = SHPGR;
  this->pI2Cx->SLPGR = SLPGR;
}

/*********************************************************************************************************//**
 * @brief Used by the master to request bytes from a slave device. The bytes may then be retrieved with the
          available() and read() functions.
 * @param address: the 7-bit address of the device to request bytes from.
 * @param quantity: the number of bytes to request.
 * @param iaddress: Devices' internal address.
 * @param isize: The internal address length of the devices.
 * @param stop : boolean. true will send a stop message after the request, releasing the bus. false will
          continually send a restart after the request, keeping the connection active.
 * @retval The number of bytes returned from the slave device
 ************************************************************************************************************/
uint8_t TwoWire::requestFrom(uint8_t address,
                             uint8_t quantity,
                             uint32_t iaddress,
                             uint8_t isize,
                             uint8_t sendStop)
{
  int8_t ret;
  uint8_t read = 0;
  uint8_t _rxBuffer[_I2C_TXRX_BUFFER_SIZE];
  if(this->I2cStatus != MASTER_MODE)
  {
    return read;
  }
  quantity = (quantity>_I2C_TXRX_BUFFER_SIZE)? _I2C_TXRX_BUFFER_SIZE : quantity;

  if (isize > 0)
  {
    /* send internal address; this mode allows sending a repeated start to access
       some devices' internal registers. This function is executed by the hardware
       TWI module on other processors */
    beginTransmission(address);

    /* the maximum size of internal address is 3 bytes */
    if (isize > 3)
    {
      isize = 3;
    }

    /* write internal register address - most significant byte first */
    while (isize-- > 0)
    {
      write((uint8_t)(iaddress >> (isize * 8)));
    }
    endTransmission(false);
  }

  ret = _i2c_master_read(address, (unsigned char*)_rxBuffer, quantity);
  if (I2C_OK == ret)
  {
    read = quantity;
  }
  if((ret != I2C_ERROR) && (sendStop == true))
  {
    I2C_GenerateSTOP(this->pI2Cx);
  }
  if(read == quantity)
  {
    for(int i=0;i<read;i++)
    {
      if(gRxBuffer[this->bufferIndex].isFull())
      {
        break;
      }
      gRxBuffer[this->bufferIndex].store(_rxBuffer[i]);
    }
  }
  return read;
}

/*********************************************************************************************************//**
 * @brief Begin a transmission to the I2C slave device with the given address.
 * @param address: the 7-bit address of the device to transmit to
 * @retval None
 ************************************************************************************************************/
void TwoWire::beginTransmission(uint8_t address)
{
  if(this->I2cStatus == UNKNOW)
  {
    SetErrId(API_ERROR_NO_INITALIZED);
    return;
  }
  /* indicate that we are transmitting */
  transmitting = 1;
  /* set address of targeted slave */
  txAddress = address;
  /* reset tx buffer iterator vars */
  gTxBuffer[this->bufferIndex].clear();
}

void TwoWire::beginTransmission(int address)
{
  beginTransmission((uint8_t)address);
}

/*********************************************************************************************************//**
 * @brief Ends a transmission to a slave device that was begun by beginTransmission() and transmits the bytes
          that were queued by write().
 * @param sendStop : boolean. true will send a stop message, releasing the bus after transmission. false will
          send a restart, keeping the connection active.
 * @retval byte, which indicates the status of the transmission:
 *   This parameter can be one of the following values:
 *     @arg I2C_OK
 *     @arg I2C_DATA_TOO_LONG
 *     @arg I2C_NACK_ADDR
 *     @arg I2C_NACK_DATA
 *     @arg I2C_ERROR
 ************************************************************************************************************/
/*  WARNING: Nothing in the library keeps track of whether
    the bus tenure has been properly ended with a STOP. It
    is very possible to leave the bus in a hung state if
    no call to endTransmission(true) is made. Some I2C
    devices will behave oddly if they do not see a STOP. */

uint8_t TwoWire::endTransmission(bool sendStop)
{
  int8_t ret = 4;
  uint8_t _txBuffer[_I2C_TXRX_BUFFER_SIZE]={0};
  u32 _txBufferLength=0;
  if(this->I2cStatus == UNKNOW)
  {
    SetErrId(API_ERROR_NO_INITALIZED);
    return ret;
  }
  if (this->I2cStatus == MASTER_MODE)
  {
    for(int i=0;i<_I2C_TXRX_BUFFER_SIZE;i++)
    {
      if(gTxBuffer[this->bufferIndex].available()==0)
      {
        break;
      }
      _txBuffer[_txBufferLength] = (uint8_t)gTxBuffer[this->bufferIndex].read();
      _txBufferLength++;
    }
    ret = _i2c_master_write(txAddress, _txBuffer, _txBufferLength);

    /* reset Tx buffer */
    gTxBuffer[this->bufferIndex].clear();
    /* indicate that we are done transmitting */
    transmitting = 0;

    /* Generate STOP process */
    if(ret != I2C_ERROR)
    {
      /* If an I2C_ERROR occurs, the system will automatically release the I2C Bus,
         so it can not generate a STOP signal */
      if((sendStop == true) || (ret != I2C_OK))
      {
        /* If SendStop=TRUE or something goes wrong(excluding I2C_ERROR) let generate a STOP. */
        I2C_GenerateSTOP(this->pI2Cx);
      }
    }
  }
  return ret;
}

/*********************************************************************************************************//**
 * @brief  This function must be called in slave Tx event callback or after
 *         beginTransmission() and before endTransmission().
 * @param  pdata: pointer to the buffer data
 * @param  quantity: number of bytes to write
 * @retval number of bytes ready to write.
 ************************************************************************************************************/
size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
  size_t ret = quantity;
  if(this->I2cStatus == UNKNOW)
  {
    SetErrId(API_ERROR_NO_INITALIZED);
    return 0;
  }
  else if(this->I2cStatus == MASTER_MODE)
  {
    if (transmitting)
    {
      /* put bytes in tx buffer */
      for(size_t i=0;i<quantity;i++)
      {
        if(gTxBuffer[this->bufferIndex].available() >= _I2C_TXRX_BUFFER_SIZE)
        {
          break;
        }
        gTxBuffer[this->bufferIndex].store(data[i]);
      }
    }
  }
  else
  {
    if(this->slave.PrepareTxData((uint8_t *)data, quantity))
    {
      ret = 0;
    }
  }
  return ret;
}

size_t TwoWire::write(uint8_t data)
{
  uint8_t *pdata = &data;
  return write(pdata, 1);
}

/*********************************************************************************************************//**
 * @brief  available() inherits from the Stream utility class.
 * @retval The number of bytes available to read.
 ************************************************************************************************************/
int TwoWire::available(void)
{
  if(this->I2cStatus == UNKNOW)
  {
    SetErrId(API_ERROR_NO_INITALIZED);
    return 0;
  }
  return gRxBuffer[this->bufferIndex].available();
}

/*********************************************************************************************************//**
 * @brief  read() inherits from the Stream utility class.
 * @retval The first byte of incoming serial data available (or -1 if no data is available).
 ************************************************************************************************************/
int TwoWire::read(void)
{
  if(this->I2cStatus == UNKNOW)
  {
    SetErrId(API_ERROR_NO_INITALIZED);
    return -1;
  }
  return gRxBuffer[this->bufferIndex].read();
}

/*********************************************************************************************************//**
 * @brief  flush() inherits from the Stream utility class.
 * @retval None.
 ************************************************************************************************************/
void TwoWire::flush(void)
{
  if(this->I2cStatus == UNKNOW)
  {
    SetErrId(API_ERROR_NO_INITALIZED);
    return;
  }
  if (this->I2cStatus == SLAVE_MODE)
  {
    while( !this->slave.IsTxFinsh());
  }
}

/*********************************************************************************************************//**
 * @brief  peek() inherits from the Stream utility class.
 * @retval The first byte of incoming serial data available (or -1 if no data is available).
 ************************************************************************************************************/
int TwoWire::peek(void)
{
  if(this->I2cStatus == UNKNOW)
  {
    SetErrId(API_ERROR_NO_INITALIZED);
    return -1;
  }
  return gRxBuffer[this->bufferIndex].peek();
}

/*********************************************************************************************************//**
 * @brief  Sets function called on slave write.
 * @param  function: Callback function
 * @retval None.
 ************************************************************************************************************/
void TwoWire::onReceive(void (*function)(int))
{
  user_onReceive = function;
}

/*********************************************************************************************************//**
 * @brief  Sets function called on slave read.
 * @param  function: Callback function
 * @retval None.
 ************************************************************************************************************/
void TwoWire::onRequest(void (*function)(void))
{
  user_onRequest = function;
}

/* Private functions ---------------------------------------------------------------------------------------*/
/*********************************************************************************************************//**
 * @brief  Write bytes at a given address
 * @param  _i2c : pointer to i2c_t structure
 * @param  dev_address: specifies the address of the device.
 * @param  data: pointer to data to be write
 * @param  size: number of bytes to be write.
 * @retval read status
 ************************************************************************************************************/
i2c_status_e TwoWire::_i2c_master_write(uint8_t dev_address,
                              uint8_t *data, uint16_t size)
{
  orinWireCause=0;orinWireStatus=0;orinWireUnknownCount=0;
  i2c_status_e ret = I2C_OK;
  I2CMaster::I2CMaster_Status_Enum I2CResult = I2CMaster::I2C_MASTER_GOING;

  I2CMaster::I2CMaster_Typedef I2CMaster;

  I2CMaster.uSlaveAddress = dev_address;
  I2CMaster.Tx.puBuffer = (u8 *)data;
  I2CMaster.Tx.uLength = size;
  I2CMaster.Rx.puBuffer = (u8 *)NULL;
  I2CMaster.Rx.uLength = 0;
  I2CMaster.uTimeout_us = twi_timeout_us;
  I2CMaster.isDynamicOnOff_I2C = FALSE;

  this->master.I2CMaster_Write(&I2CMaster);
  do {
    I2CResult = master.I2CMaster_GetStatus();
  } while (I2CResult == I2CMaster::I2C_MASTER_GOING);

  switch (I2CResult)
  {
    case I2CMaster::I2C_MASTER_FINISH:
      ret = I2C_OK;
      break;
    case I2CMaster::I2C_MASTER_ADDR_NACK:
      ret = I2C_NACK_ADDR;
      break;
    case I2CMaster::I2C_MASTER_DATA_NACK:
      ret = I2C_NACK_DATA;
      break;
    case I2CMaster::I2C_MASTER_SW_TIMEOUT:
    case I2CMaster::I2C_MASTER_HW_TIMEOUT:
      ret = I2C_TIMEOUT;
      if(this->twi_do_reset_on_timeout)
      {
        this->twi_timed_out_flag = true;
        if(twi_do_reset_on_timeout)
        {
          uint32_t SHPGR = this->pI2Cx->SHPGR;
          uint32_t SLPGR = this->pI2Cx->SLPGR;
          I2C_DeInit(this->pI2Cx);
          _errorHandling();
          begin(this->device_address);
          this->pI2Cx->SHPGR = SHPGR;
          this->pI2Cx->SLPGR = SLPGR;
        }
      }
      break;
    default:
      ret = I2C_ERROR;
      break;
  }
  return ret;
}

/*********************************************************************************************************//**
 * @brief  read bytes in master mode at a given address
 * @param  _i2c : pointer to i2c_t structure
 * @param  dev_address: specifies the address of the device.
 * @param  data: pointer to data to be read
 * @param  size: number of bytes to be read.
 * @retval read status
 ************************************************************************************************************/
i2c_status_e TwoWire::_i2c_master_read(uint8_t dev_address, uint8_t *data, uint16_t size)
{
  orinWireCause=0;orinWireStatus=0;orinWireUnknownCount=0;
  i2c_status_e ret = I2C_OK;
  uint8_t _txBuffer[_I2C_TXRX_BUFFER_SIZE]={0};
  u32 _txBufferLength=0;
  for(int i=0;i<_I2C_TXRX_BUFFER_SIZE;i++)
  {
    if(gTxBuffer[this->bufferIndex].available()==0)
    {
      break;
    }
    _txBuffer[_txBufferLength] = (uint8_t)gTxBuffer[this->bufferIndex].read();
    _txBufferLength++;
  }
  I2CMaster::I2CMaster_Status_Enum I2CResult = I2CMaster::I2C_MASTER_GOING;

  I2CMaster::I2CMaster_Typedef I2CMaster;
  I2CMaster.uSlaveAddress = dev_address;
  I2CMaster.Tx.puBuffer = (u8 *)_txBuffer;
  I2CMaster.Tx.uLength = _txBufferLength;
  I2CMaster.Rx.puBuffer = (u8 *)data;
  I2CMaster.Rx.uLength = size;
  I2CMaster.uTimeout_us = twi_timeout_us;
  I2CMaster.isDynamicOnOff_I2C = FALSE;
  if(_txBufferLength>0)
  {
    this->master.I2CMaster_WriteRead(&I2CMaster);
  }
  else
    this->master.I2CMaster_Read(&I2CMaster);

  do {
    I2CResult = this->master.I2CMaster_GetStatus();
  } while (I2CResult == I2CMaster::I2C_MASTER_GOING);

  switch (I2CResult)
  {
    case I2CMaster::I2C_MASTER_FINISH:
      ret = I2C_OK;
      break;
    case I2CMaster::I2C_MASTER_ADDR_NACK:
      ret = I2C_NACK_ADDR;
      break;
    case I2CMaster::I2C_MASTER_DATA_NACK:
      ret = I2C_NACK_DATA;
      break;
    case I2CMaster::I2C_MASTER_SW_TIMEOUT:
    case I2CMaster::I2C_MASTER_HW_TIMEOUT:
      if(this->twi_do_reset_on_timeout)
      {
        this->twi_timed_out_flag = true;
        if(twi_do_reset_on_timeout)
        {
          uint32_t SHPGR = this->pI2Cx->SHPGR;
          uint32_t SLPGR = this->pI2Cx->SLPGR;
          I2C_DeInit(this->pI2Cx);
          begin(this->device_address);
          this->pI2Cx->SHPGR = SHPGR;
          this->pI2Cx->SLPGR = SLPGR;
        }
      }
      ret = I2C_TIMEOUT;
      break;
    default:
      ret = I2C_ERROR;
      break;
  }
  return ret;
}

/*********************************************************************************************************//**
 * @brief IRQ Handler
 * @retval None
 ************************************************************************************************************/
void TwoWire::IrqHandler()
{
  if(this->I2cStatus == UNKNOW)
  {
    return;
  }
  if(this->I2cStatus == MASTER_MODE)
  {
    this->master.I2C_IRQHandler();
  }
  else
  {
    this->slave.I2C_IRQHandler(this->user_onReceive, this->user_onRequest);
  }
}
/***
 * Sets the TWI timeout.
 *
 * This limits the maximum time to wait for the TWI hardware. If more time passes, the bus is assumed
 * to have locked up (e.g. due to noise-induced glitches or faulty slaves) and the transaction is aborted.
 * Optionally, the TWI hardware is also reset, which can be required to allow subsequent transactions to
 * succeed in some cases (in particular when noise has made the TWI hardware think there is a second
 * master that has claimed the bus).
 *
 * When a timeout is triggered, a flag is set that can be queried with `getWireTimeoutFlag()` and is cleared
 * when `clearWireTimeoutFlag()` or `setWireTimeoutUs()` is called.
 *
 * Note that this timeout can also trigger while waiting for clock stretching or waiting for a second master
 * to complete its transaction. So make sure to adapt the timeout to accomodate for those cases if needed.
 * A typical timeout would be 25ms (which is the maximum clock stretching allowed by the SMBus protocol),
 * but (much) shorter values will usually also work.
 *
 * In the future, a timeout will be enabled by default, so if you require the timeout to be disabled, it is
 * recommended you disable it by default using `setWireTimeoutUs(0)`, even though that is currently
 * the default.
 *
 * @param timeout a timeout value in microseconds, if zero then timeout checking is disabled
 * @param reset_with_timeout if true then TWI interface will be automatically reset on timeout
 *                           if false then TWI interface will not be reset on timeout

 */

void TwoWire::setWireTimeout(uint32_t timeout, bool reset_with_timeout){
  twi_timed_out_flag = false;
  twi_timeout_us = timeout;
  twi_do_reset_on_timeout = reset_with_timeout;
}

/***
 * Returns the TWI timeout flag.
 *
 * @return true if timeout has occured since the flag was last cleared.
 */
bool TwoWire::getWireTimeoutFlag(void){
  return(twi_timed_out_flag);
}

/***
 * Clears the TWI timeout flag.
 */
void TwoWire::clearWireTimeoutFlag(void){
  twi_timed_out_flag = false;
}
/*
 * Function twi_handleTimeout
 * Desc     this gets called whenever a while loop here has lasted longer than
 *          twi_timeout_us microseconds. always sets twi_timed_out_flag
 * Input    reset: true causes this function to reset the twi hardware interface
 * Output   none
 */

#ifdef __cplusplus
extern "C"{
#endif

#if defined(HT_I2C0) && defined(I2C0_IRQ_HANDLER)
void I2C0_IRQHandler(void)
{
  I2C0_IRQ_HANDLER();
}
#endif
#if defined(HT_I2C1) && defined(I2C1_IRQ_HANDLER)
void I2C1_IRQHandler(void)
{
  I2C1_IRQ_HANDLER();
}
#endif
#if defined(HT_I2C2) && defined(I2C2_IRQ_HANDLER)
void I2C2_IRQHandler(void)
{
  I2C2_IRQ_HANDLER();
}
#endif
#if defined(HT_I2C3) && defined(I2C3_IRQ_HANDLER)
void I2C3_IRQHandler(void)
{
  I2C3_IRQ_HANDLER();
}
#endif
#if defined(HT_I2C4) && defined(I2C4_IRQ_HANDLER)
void I2C4_IRQHandler(void)
{
  I2C4_IRQ_HANDLER();
}
#endif
#if defined(HT_I2C5) && defined(I2C5_IRQ_HANDLER)
void I2C5_IRQHandler(void)
{
  I2C5_IRQ_HANDLER();
}
#endif
#ifdef __cplusplus
}
#endif /* cpp */

#ifdef WIRE0_PORT
TwoWire _OBJ_Wire0 = TwoWire(WIRE0_ARRAY_INDEX);
#endif
#ifdef WIRE1_PORT
TwoWire _OBJ_Wire1 = TwoWire(WIRE1_ARRAY_INDEX);
#endif
#ifdef WIRE2_PORT
TwoWire _OBJ_Wire2 = TwoWire(WIRE2_ARRAY_INDEX);
#endif
#ifdef WIRE3_PORT
TwoWire _OBJ_Wire3 = TwoWire(WIRE3_ARRAY_INDEX);
#endif
#ifdef WIRE4_PORT
TwoWire _OBJ_Wire4 = TwoWire(WIRE4_ARRAY_INDEX);
#endif
#ifdef WIRE5_PORT
TwoWire _OBJ_Wire5 = TwoWire(WIRE5_ARRAY_INDEX);
#endif
#ifdef WIRE6_PORT
TwoWire _OBJ_Wire6 = TwoWire(WIRE6_ARRAY_INDEX);
#endif
#ifdef WIRE7_PORT
TwoWire _OBJ_Wire7 = TwoWire(WIRE7_ARRAY_INDEX);
#endif
#ifdef WIRE8_PORT
TwoWire _OBJ_Wire8 = TwoWire(WIRE8_ARRAY_INDEX);
#endif
#ifdef WIRE9_PORT
TwoWire _OBJ_Wire9 = TwoWire(WIRE9_ARRAY_INDEX);
#endif
#ifdef WIRE10_PORT
TwoWire _OBJ_Wire10 = TwoWire(WIRE10_ARRAY_INDEX);
#endif
#ifdef WIRE11_PORT
TwoWire _OBJ_Wire11 = TwoWire(WIRE11_ARRAY_INDEX);
#endif
#ifdef WIRE12_PORT
TwoWire _OBJ_Wire12 = TwoWire(WIRE12_ARRAY_INDEX);
#endif
#ifdef WIRE13_PORT
TwoWire _OBJ_Wire13 = TwoWire(WIRE13_ARRAY_INDEX);
#endif
#ifdef WIRE14_PORT
TwoWire _OBJ_Wire14 = TwoWire(WIRE14_ARRAY_INDEX);
#endif
#ifdef WIRE15_PORT
TwoWire _OBJ_Wire15 = TwoWire(WIRE15_ARRAY_INDEX);
#endif
#ifdef WIRE16_PORT
TwoWire _OBJ_Wire16 = TwoWire(WIRE16_ARRAY_INDEX);
#endif
#ifdef WIRE17_PORT
TwoWire _OBJ_Wire17 = TwoWire(WIRE17_ARRAY_INDEX);
#endif
#ifdef WIRE18_PORT
TwoWire _OBJ_Wire18 = TwoWire(WIRE18_ARRAY_INDEX);
#endif
#ifdef WIRE19_PORT
TwoWire _OBJ_Wire19 = TwoWire(WIRE19_ARRAY_INDEX);
#endif
