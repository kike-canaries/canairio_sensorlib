/*!
  * @file  readGasConcentration.ino
  * @brief Obtain gas concentration corresponding to the current environment, output as concentration value
  * @n Experimental mode: connect sensor communication pin to the main controller and burn
  * @n Communication mode select, DIP switch SEL: 0: I2C, 1: UART
  * @n Group serial number         Address in the group
  * @n A0 A1 DIP level 00    01    10    11
  * @n 1            0x60  0x61  0x62  0x63
  * @n 2            0x64  0x65  0x66  0x67
  * @n 3            0x68  0x69  0x6A  0x6B
  * @n 4            0x6C  0x6D  0x6E  0x6F
  * @n 5            0x70  0x71  0x72  0x73
  * @n 6 (Default address group) 0x74  0x75  0x76  0x77 (Default address)
  * @n 7            0x78  0x79  0x7A  0x7B
  * @n 8            0x7C  0x7D  0x7E  0x7F
  * @n i2c address select, default to 0x77, A1 and A0 are grouped into 4 I2C addresses.
  * @n             | A0 | A1 |
  * @n             | 0  | 0  |    0x74
  * @n             | 0  | 1  |    0x75
  * @n             | 1  | 0  |    0x76
  * @n             | 1  | 1  |    0x77   default i2c address  
  * @n Experimental phenomenon: view the gas concentration corresponding to the current environment through serial port printing
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V1.0
  * @date        2021-03-28
  * @url         https://github.com/DFRobot/DFRobot_MultiGasSensor
  */
#include "DFRobot_MultiGasSensor.h"
#include <Arduino.h>

//Turn on by default, using I2C communication at the time, switch to serial port communication after turning off
//#define I2C_COMMUNICATION

//#ifdef  I2C_COMMUNICATION
//#define I2C_ADDRESS    0x77
  DFRobot_GAS_I2C nh3(&Wire,0x7A);
  DFRobot_GAS_I2C co(&Wire,0x78);
  DFRobot_GAS_I2C no(&Wire,0x7B);

void setup() {
  //Serial port init for viewing printing output
  Serial.begin(115200);

 //Change i2c address group
  while(gas.changeI2cAddrGroup(7)==0)
  {
    Serial.println("IIC addr change fail!");
    delay(1000);
  }  
  Serial.println("IIC addr change success!");
}

  //Sensor init, used to init serial port or I2C, depending on the communication mode currently used
  while(!nh3.begin())
  {
    Serial.println("No Devices NH3 !");
    delay(1000);
  }
  //Mode of obtaining data: the main controller needs to request the sensor for data
  nh3.changeAcquireMode(nh3.PASSIVITY);
  delay(1000);

  nh3.setTempCompensation(nh3.ON);

 Serial.println("The device nh3  0x7A is connected successfully!");
 
  while(!co.begin())
  {
    Serial.println("No Devices CO !");
    delay(1000);
  }

 co.changeAcquireMode(co.PASSIVITY);
  delay(1000);

  co.setTempCompensation(co.ON);

 Serial.println("The device CO  0x78 is connected successfully!");

  while(!no2.begin())
  {
    Serial.println("No Devices NO2 !");
    delay(1000);
  }

 no2.changeAcquireMode(no2.PASSIVITY);
  delay(1000);

  no2.setTempCompensation(no2.ON);

 Serial.println("The device CO  0x7B is connected successfully!");
}

void loop() {
  String gastypeNH3 = nh3.queryGasType();
  /**
   *Fill in the parameter readGasConcentration() with the type of gas to be obtained and print
   *The current gas concentration
   *Print with 1s delay each time
   */
  Serial.print("Ambient ");
  Serial.print(gastypeNH3);
  Serial.print(" concentration is: ");
  Serial.print(nh3.readGasConcentrationPPM());
  if (gastypeNH3 == "O2")
    Serial.println(" %vol");
  else
    Serial.println(" PPM");
  Serial.println();
  delay(1000);

   String gastypeCO = co.queryGasType();
  /**
   *Fill in the parameter readGasConcentration() with the type of gas to be obtained and print
   *The current gas concentration
   *Print with 1s delay each time
   */
  Serial.print("Ambient ");
  Serial.print(gastypeCO);
  Serial.print(" concentration is: ");
  Serial.print(co.readGasConcentrationPPM());
  if (gastypeCO == "O2")
    Serial.println(" %vol");
  else
    Serial.println(" PPM");
  Serial.println();
  delay(1000);

   String gastypeNO2 = no2.queryGasType();
  /**
   *Fill in the parameter readGasConcentration() with the type of gas to be obtained and print
   *The current gas concentration
   *Print with 1s delay each time
   */
  Serial.print("Ambient ");
  Serial.print(gastypeNO2);
  Serial.print(" concentration is: ");
  Serial.print(no2.readGasConcentrationPPM());
  if (gastypeNO2 == "O2")
    Serial.println(" %vol");
  else
    Serial.println(" PPM");
  Serial.println();
  delay(1000);
}
