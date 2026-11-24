/*
  TwoWire.h - TWI/I2C library for Arduino & Wiring
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

/* Define to prevent recursive inclusion -------------------------------------------------------------------*/
#ifndef TwoWire_h
#define TwoWire_h

/* Includes ------------------------------------------------------------------------------------------------*/
#include "Stream.h"
#include "Arduino.h"
#include "utility/i2c_master.h"
#include "utility/i2c_slave.h"

/* Exported typedef ----------------------------------------------------------------------------------------*/
typedef enum {
  I2C_OK = 0,
  I2C_DATA_TOO_LONG = 1,
  I2C_NACK_ADDR = 2,
  I2C_NACK_DATA = 3,
  I2C_ERROR = 4,
  I2C_TIMEOUT = 5,
  I2C_BUSY = 6
} i2c_status_e;

/* Exported constants --------------------------------------------------------------------------------------*/
/* I2C Tx/Rx buffer size */
#if !defined(_I2C_TXRX_BUFFER_SIZE)
#define _I2C_TXRX_BUFFER_SIZE    33
#endif

#define I2C_BUFFER_LENGTH    (_I2C_TXRX_BUFFER_SIZE-1)

/* Class ---------------------------------------------------------------------------------------------------*/
class TwoWire : public Stream {
  public:
    typedef enum {
      UNKNOW = 0,
      MASTER_MODE,
      SLAVE_MODE
    } I2CPortStatus;
    void (*user_onRequest)(void);
    void (*user_onReceive)(int);
    TwoWire(u32 arrayIndex);
    void begin();
    void begin(uint8_t);
    void begin(int);
    void end();
    void setClock(uint32_t);
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    uint8_t endTransmission( bool sendStop=TRUE);
    uint8_t requestFrom(uint8_t, uint8_t, uint32_t, uint8_t, uint8_t);
    size_t requestFrom(uint8_t address, size_t quantity, bool stopBit)
    {
      return requestFrom((uint8_t)address, (uint8_t)quantity, (uint32_t)0, (uint8_t)0, (uint8_t)stopBit);
    }
    size_t requestFrom(uint8_t address, size_t quantity)
    {
      return requestFrom((uint8_t)address, (uint8_t)quantity, (uint32_t)0, (uint8_t)0, (uint8_t)TRUE);
    }

    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive(void (*)(int));
    void onRequest(void (*)(void));
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n)          { return write((uint8_t)n); }
    inline size_t write(unsigned int n)  { return write((uint8_t)n); }
    inline size_t write(int n)           { return write((uint8_t)n); }
    inline size_t write(unsigned short n)  { return write((uint8_t)n); }
    inline size_t write(short n)           { return write((uint8_t)n); }

    void IrqHandler();
    using Print::write;
    operator bool() { return I2cStatus==UNKNOW? false: true; }
    void setWireTimeout(uint32_t timeout = 25000, bool reset_with_timeout = false);
    bool getWireTimeoutFlag(void);
    void clearWireTimeoutFlag(void);
  private:
    I2CMaster master;
    I2CSlave slave;
    uint8_t bufferIndex;
    uint8_t arrayIndex;
    I2CPortStatus I2cStatus;
    HT_I2C_TypeDef* pI2Cx;

    uint8_t txAddress;

    uint8_t transmitting;
    uint8_t ownAddress;

    void _errorHandling(void);
    i2c_status_e _i2c_master_write(uint8_t dev_address, uint8_t *data, uint16_t size);
    i2c_status_e _i2c_master_read(uint8_t dev_address, uint8_t *data, uint16_t size);
    bool twi_timed_out_flag;
    uint32_t twi_timeout_us;
    bool twi_do_reset_on_timeout;
    uint8_t device_address;
};

#ifdef WIRE0_PORT
extern TwoWire _OBJ_Wire0;
#endif
#ifdef WIRE1_PORT
extern TwoWire _OBJ_Wire1;
#endif
#ifdef WIRE2_PORT
extern TwoWire _OBJ_Wire2;
#endif
#ifdef WIRE3_PORT
extern TwoWire _OBJ_Wire3;
#endif
#ifdef WIRE4_PORT
extern TwoWire _OBJ_Wire4;
#endif
#ifdef WIRE5_PORT
extern TwoWire _OBJ_Wire5;
#endif
#ifdef WIRE6_PORT
extern TwoWire _OBJ_Wire6;
#endif
#ifdef WIRE7_PORT
extern TwoWire _OBJ_Wire7;
#endif
#ifdef WIRE8_PORT
extern TwoWire _OBJ_Wire8;
#endif
#ifdef WIRE9_PORT
extern TwoWire _OBJ_Wire9;
#endif
#ifdef WIRE10_PORT
extern TwoWire _OBJ_Wire10;
#endif
#ifdef WIRE11_PORT
extern TwoWire _OBJ_Wire11;
#endif
#ifdef WIRE12_PORT
extern TwoWire _OBJ_Wire12;
#endif
#ifdef WIRE13_PORT
extern TwoWire _OBJ_Wire13;
#endif
#ifdef WIRE14_PORT
extern TwoWire _OBJ_Wire14;
#endif
#ifdef WIRE15_PORT
extern TwoWire _OBJ_Wire15;
#endif
#ifdef WIRE16_PORT
extern TwoWire _OBJ_Wire16;
#endif
#ifdef WIRE17_PORT
extern TwoWire _OBJ_Wire17;
#endif
#ifdef WIRE18_PORT
extern TwoWire _OBJ_Wire18;
#endif
#ifdef WIRE19_PORT
extern TwoWire _OBJ_Wire19;
#endif

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
}
#endif
#endif

#define ORIN_WIRE_DETAIL_VERSION 2
extern volatile uint32_t orinWireCause, orinWireStatus, orinWireUnknownCount;
