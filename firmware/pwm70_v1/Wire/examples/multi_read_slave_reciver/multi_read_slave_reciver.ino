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
  Wire.begin(4); 
  Wire.onReceive(receiveEvent); // register event
  Wire? Serial.print("Wire enabled success."):
    Serial.print("Wire enabled fail. The physical IP of Wirex is already occupied, please close the corresponding Wirex. Example: \"Wire.end();\"");
  Serial.print("\r\n");
  #endif
  #if _TEST_WIRE_NUM>1
  Wire1.begin(5); 
  Wire1.onReceive(receiveEvent1); // register event
  Wire1.end(); 
    Wire1? Serial.print("Wire1 enabled success."):
    Serial.print("Wire1 enabled fail. The physical IP of Wire1 is already occupied, please close the corresponding Wirex. Example: \"Wire.end();\"");
    Serial.print("\r\n");
  #endif
  #if _TEST_WIRE_NUM>2
  Wire2.begin(6); 
  Wire2.onReceive(receiveEvent2); // register event
    Wire2? Serial.print("Wire2 enabled success."):
    Serial.print("Wire2 enabled fail. The physical IP of Wire2 is already occupied, please close the corresponding Wirex. Example: \"Wire.end();\"");
    Serial.print("\r\n");
  #endif
  #if _TEST_WIRE_NUM>3
  Wire3.begin(7);
  Wire3.onReceive(receiveEvent3); // register event
    Wire3? Serial.print("Wire3 enabled success."):
    Serial.print("Wire3 enabled fail. The physical IP of Wire3 is already occupied, please close the corresponding Wirex. Example: \"Wire.end();\"");
    Serial.print("\r\n");
  #endif
  #if _TEST_WIRE_NUM>4
  Wire4.onReceive(receiveEvent4); // register event // join i2c bus (address optional for master)
    Wire4? Serial.print("Wire4 enabled success."):
    Serial.print("Wire5 enabled fail. The physical IP of Wire4 is already occupied, please close the corresponding Wirex. Example: \"Wire.end();\"");
    Serial.print("\r\n");
  #endif
  #if _TEST_WIRE_NUM>5
  Wire5.begin(9);
  Wire5.onReceive(receiveEvent5); // register event
    Wire5? Serial.print("Wire5 enabled success."):
    Serial.print("Wire6 enabled fail. The physical IP of Wire5 is already occupied, please close the corresponding Wirex. Example: \"Wire.end();\"");
    Serial.print("\r\n");
  #endif
}

void loop()
{
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  Serial.print("Wire: ");         // print the character
  while(1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}
void receiveEvent1(int howMany)
{
  Serial.print("Wire1: ");         // print the character
  while(1 < Wire1.available()) // loop through all but the last
  {
    char c = Wire1.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire1.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}
void receiveEvent2(int howMany)
{
  Serial.print("Wire2: ");         // print the character
  while(1 < Wire2.available()) // loop through all but the last
  {
    char c = Wire2.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire2.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}
void receiveEvent3(int howMany)
{
  Serial.print("Wire3: ");         // print the character
  while(1 < Wire3.available()) // loop through all but the last
  {
    char c = Wire3.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire3.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}
void receiveEvent4(int howMany)
{
  Serial.print("Wire4: ");         // print the character
  while(1 < Wire4.available()) // loop through all but the last
  {
    char c = Wire4.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire4.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}
void receiveEvent5(int howMany)
{
  Serial.print("Wire5: ");         // print the character
  while(1 < Wire5.available()) // loop through all but the last
  {
    char c = Wire5.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire5.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}
