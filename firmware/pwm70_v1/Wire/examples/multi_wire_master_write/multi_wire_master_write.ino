#include <Wire.h>
#define _TEST_WIRE_NUM         (6)
#if _TEST_WIRE_NUM > I2C_WIRE_NUM
  #undef _TEST_WIRE_NUM 
  #define _TEST_WIRE_NUM       I2C_WIRE_NUM
#endif

void setup()
{
  Serial.begin(115200);
  #if _TEST_WIRE_NUM>0
  Wire.begin(); // join i2c bus (address optional for master)
  Wire? Serial.print("Wire enabled success."):
    Serial.print("Wire enabled fail. The physical IP of Wirex is already occupied, please close the corresponding Wirex. Example: \"Wire.end();\"");
  Serial.print("\r\n");
  #endif
  #if _TEST_WIRE_NUM>1
  Wire1.begin(); // join i2c bus (address optional for master)
    Wire1? Serial.print("Wire1 enabled success."):
    Serial.print("Wire1 enabled fail. The physical IP of Wire1 is already occupied, please close the corresponding Wirex. Example: \"Wire.end();\"");
    Serial.print("\r\n");
  #endif
  #if _TEST_WIRE_NUM>2
  Wire2.begin(); // join i2c bus (address optional for master)
    Wire2? Serial.print("Wire2 enabled success."):
    Serial.print("Wire2 enabled fail. The physical IP of Wire2 is already occupied, please close the corresponding Wirex. Example: \"Wire.end();\"");
    Serial.print("\r\n");
  #endif
  #if _TEST_WIRE_NUM>3
  Wire3.begin(); // join i2c bus (address optional for master)
    Wire3? Serial.print("Wire3 enabled success."):
    Serial.print("Wire3 enabled fail. The physical IP of Wire3 is already occupied, please close the corresponding Wirex. Example: \"Wire.end();\"");
    Serial.print("\r\n");
  #endif
  #if _TEST_WIRE_NUM>4
  Wire4.begin(); // join i2c bus (address optional for master)
    Wire4? Serial.print("Wire4 enabled success."):
    Serial.print("Wire5 enabled fail. The physical IP of Wire4 is already occupied, please close the corresponding Wirex. Example: \"Wire.end();\"");
    Serial.print("\r\n");
  #endif
  #if _TEST_WIRE_NUM>5
  Wire5.begin(); // join i2c bus (address optional for master)
    Wire5? Serial.print("Wire5 enabled success."):
    Serial.print("Wire6 enabled fail. The physical IP of Wire5 is already occupied, please close the corresponding Wirex. Example: \"Wire.end();\"");
    Serial.print("\r\n");
  #endif
}
int x=0;
void loop()
{
  if(Wire)
  {
    Wire.beginTransmission(4); // transmit to device #4
    Wire.write("x is ");        // sends five bytes
    Wire.write(x);              // sends one byte  
    Wire.endTransmission();    // stop transmitting
  }
  if(Wire1)
  {
    Wire1.beginTransmission(5); // transmit to device #5
    Wire1.write("x is ");        // sends five bytes
    Wire1.write(x);              // sends one byte  
    Wire1.endTransmission();    // stop transmitting
  }
  if(Wire2)
  {
    Wire2.beginTransmission(6); // transmit to device #6
    Wire2.write("x is ");        // sends five bytes
    Wire2.write(x);              // sends one byte  
    Wire2.endTransmission();    // stop transmitting
  }
  if(Wire3)
  {
    Wire3.beginTransmission(7); // transmit to device #7
    Wire3.write("x is ");        // sends five bytes
    Wire3.write(x);              // sends one byte  
    Wire3.endTransmission();    // stop transmitting
  }
  if(Wire4)
  {
    Wire4.beginTransmission(8); // transmit to device #8
    Wire4.write("x is ");        // sends five bytes
    Wire4.write(x);              // sends one byte  
    Wire4.endTransmission();    // stop transmitting
  }
  if(Wire5)
  {
    Wire5.beginTransmission(9); // transmit to device #9
    Wire5.write("x is ");        // sends five bytes
    Wire5.write(x);              // sends one byte  
    Wire5.endTransmission();    // stop transmitting
  }
  x++;
  delay(500);
}
