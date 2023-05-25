/*!
  * @file  DFRobot_MultiGasSensor.h
  * @brief This is a header file of the library for the sensor that can detect gas concentration in the air.
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V2.0.0
  * @date        2021-09-26
  * @url         https://github.com/DFRobot/DFRobot_MultiGasSensor
*/
#ifndef __DFRobot_GAS_H__
#define __DFRobot_GAS_H__

#include "Arduino.h"
#include <Wire.h>

#if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)
#include "SoftwareSerial.h"
#else
#include "HardwareSerial.h"
#endif

#define CMD_CHANGE_GET_METHOD          0X78
#define CMD_GET_GAS_CONCENTRATION      0X86
#define CMD_GET_TEMP                   0X87
#define CMD_GET_ALL_DTTA               0X88
#define CMD_SET_THRESHOLD_ALARMS       0X89
#define CMD_IIC_AVAILABLE              0X90
#define CMD_SENSOR_VOLTAGE             0X91
#define CMD_CHANGE_IIC_ADDR            0X92

// Open this macro to see the program running in detail
#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] 0x"); Serial.println(__VA_ARGS__,HEX);}
#else
#define DBG(...)
#endif

/**
 * @struct sProtocol_t
 * @brief Data protocol package for communication
 */
typedef struct
{
  uint8_t head;
  uint8_t addr;
  uint8_t data[6];
  uint8_t check;
} sProtocol_t;

/**
 * @struct sAllData_t
 * @brief The struct used when getting all the data
 * @note 
 * @n -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * @n |  byte0          | byte1    |          byte2             |        byte3                |  byte4   |  byte5         |  byte6                 |   byte7               |  byte8   
 * @n -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * @n |  Protocol Head  | Command  | Gas Concentrate High 8-bit | Gas Concentration Low 8-bit | Gas Type | Decimal Digits | Temperature High 8-bit | Temperature Low 8-bit |  CRC
 * @n -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */
typedef struct
{
  uint8_t head;
  uint8_t cmd;
  uint8_t gasconcentration_h;
  uint8_t gasconcentration_l;
  uint8_t gastype;
  uint8_t gasconcentration_decimals;
  uint8_t temp_h;
  uint8_t temp_l;
  uint8_t check;
} sAllData_t;
extern sAllData_t AllData;

/**
 * @struct sAllDataAnalysis_t
 * @brief All the parsed data 
 */
typedef struct
{
  float gasconcentration;
  String gastype;
  float temp;
} sAllDataAnalysis_t;
extern sAllDataAnalysis_t AllDataAnalysis;

class DFRobot_GAS
{
public:
  /**
   * @enum eMethod_t
   * @brief Type of the data the sensor uploads
   */
  typedef enum
  {
    INITIATIVE = 0x03,
    PASSIVITY = 0x04
  } eMethod_t;

  /**
   * @enum eType_t
   * @brief Gas Type
   */
  typedef enum
  {
    O2 = 0x05,
    CO = 0x04,
    H2S = 0x03,
    NO2 = 0x2C,
    O3 = 0x2A,
    CL2 = 0x31,
    NH3 = 0x02,
    H2 = 0x06,
    HCL = 0X2E,
    SO2 = 0X2B,
    HF = 0x33,
    _PH3 = 0x45
  } eType_t;

  /**
   * @enum eSwitch_t
   * @brief Whether to enable ALA alarm function
   */
  typedef enum
  {
    ON = 0x01,
    OFF = 0x00
  } eSwitch_t;

  /**
   * @enum eSwitch_t
   * @brief High and low ALA alarm function
   */
  typedef enum
  {
    LOW_THRESHOLD_ALA = 0x00,
    HIGH_THRESHOLD_ALA = 0x01
  } eALA_t;

  DFRobot_GAS(void){};
  ~DFRobot_GAS(void){};

  /**
   * @fn begin
   * @brief Parent class init, I2C or UART init is performed in subclass function
   * @return bool type, indicating whether init succeed
   * @retval True succeed
   * @retval False failed
   */
  virtual bool begin(void) = 0;

  /**
   * @fn changeAcquireMode
   * @brief Change the mode of acquiring sensor data
   * @param mode Mode select
   * @n     INITIATIVE The sensor proactively reports data
   * @n     PASSIVITY The main controller needs to request data from sensor
   * @return bool type, indicating whether the setting is successful
   * @retval True succeed
   * @retval False failed
   */
  bool changeAcquireMode(eMethod_t mode);

  /**
   * @fn readGasConcentrationPPM
   * @brief Get gas concentration from sensor, unit PPM
   * @return float type, indicating return gas concentration, if data is transmitted normally, return gas concentration, otherwise, return 0.0
   */
  float readGasConcentrationPPM(void);

  /**
   * @fn queryGasType
   * @brief Query gas type
   * @return String type, indicating return gas type string
   */
  String queryGasType(void);

  /**
   * @fn setThresholdAlarm
   * @brief Set sensor alarm threshold
   * @param switchof Whether to turn on threshold alarm switch
   * @n            ON turn on     
   * @n           OFF turn off
   * @param threshold The threshold for starting alarm
   * @param alamethod Set sensor high or low threshold alarm
   * @param gasType   Gas Type
   * @return bool type, indicating whether the setting is successful
   * @retval True succeed
   * @retval False failed
   */
  bool setThresholdAlarm(eSwitch_t switchof, uint16_t threshold, eALA_t alamethod, String gasType);

  /**
   * @fn readTempC
   * @brief Get sensor onboard temperature
   * @return float type, indicating return the current onboard temperature
   */
  float readTempC(void);

  /**
   * @fn setTempCompensation
   * @brief Set whether to turn on temperature compensation, values output by sensor under different temperatures are various.
   * @n     To get more accurate gas concentration, temperature compensation is necessary when calculating gas concentration.
   * @param tempswitch Whether to turn on temperature compensation
   * @n             ON Turn on temperature compensation
   * @n            OFF Turn off temperature compensation
   */
  void setTempCompensation(eSwitch_t tempswitch);

  /**
   * @fn readVolatageData
   * @brief Get sensor gas concentration output by original voltage, which is different from reading sensor register directly.
   * @n     The function is mainly for detecting whether the read gas concentration is right.
   * @param vopin Pin for receiving the original voltage output from sensor probe
   * @return Float type, indicating return the original voltage output of sensor gas concentration
   */
  float readVolatageData(uint8_t vopin);

  /**
   * @fn pack
   * @brief Pack the protocol data for easy transmission
   * @param pBuf Data to be packed
   * @param len Length of data package  
   * @return sProtocol_t type, indicating return the packed data
   */
  sProtocol_t pack(uint8_t *pBuf, uint8_t len);

  /**
   * @fn getSensorVoltage
   * @brief Get voltage output by sensor probe (for calculating the current gas concentration)
   * @return float type, indicating return voltage value
   */
  float getSensorVoltage(void);

  /**
   * @fn dataIsAvailable
   * @brief Call this function in active mode to determine the presence of data on data line
   * @return bool type, indicating whether there is data coming from the sensor 
   * @retval True Has uploaded data
   * @retval False No data uploaded
   */
  virtual bool dataIsAvailable(void) = 0;

  /**
   * @fn changeI2cAddrGroup
   * @brief Change I2C address group
   * @param group Address group select
   * @return int type, indicating return init status
   * @retval bool type
   * @retval True Change succeed
   * @retval False Change failed
   */
  bool changeI2cAddrGroup(uint8_t group);

protected:
  /**
   * @fn writeData
   * @brief Write data to the specified register of the sensor
   * @param Reg Register address to be written
   * @param Data Data to be written to register
   * @param len Length of data to be written
   */
  virtual void writeData(uint8_t Reg, void *Data, uint8_t len) = 0;

  /**
   * @fn readData
   * @brief Get the data with specified length from the specified sensor
   * @param Reg Register address to be read
   * @param Data Position of the data stored in the register to be read 
   * @param len Length of the data to be written
   */
  virtual int16_t readData(uint8_t Reg, uint8_t *Data, uint8_t len) = 0;

private:
  bool _tempswitch;
};

class DFRobot_GAS_I2C : public DFRobot_GAS
{
  public:
    DFRobot_GAS_I2C(TwoWire *pWire=&Wire,uint8_t addr=0x74);
    ~DFRobot_GAS_I2C(void){};
    bool begin(void);
    void setI2cAddr(uint8_t addr);
    bool dataIsAvailable(void);
  protected:
    void writeData(uint8_t Reg ,void *Data ,uint8_t len);
    int16_t readData(uint8_t Reg ,uint8_t *Data ,uint8_t len);
  private:
    TwoWire* _pWire;
    uint8_t _I2C_addr;
};

#if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)
class DFRobot_GAS_SoftWareUart : public DFRobot_GAS
{
  public:
    DFRobot_GAS_SoftWareUart(SoftwareSerial *psoftUart);
    ~DFRobot_GAS_SoftWareUart(void){};
    bool begin(void);
    bool dataIsAvailable(void);
  protected:
    void writeData(uint8_t Reg ,void *Data ,uint8_t len);
    int16_t readData(uint8_t Reg ,uint8_t *Data ,uint8_t len);
  private:
    SoftwareSerial *_psoftUart;
}; 
#else
class DFRobot_GAS_HardWareUart : public DFRobot_GAS
{
  public:
    DFRobot_GAS_HardWareUart(HardwareSerial *phardUart);
    ~DFRobot_GAS_HardWareUart(void){};
    bool begin(void);
  protected:
    bool dataIsAvailable(void);
    void writeData(uint8_t Reg, void *Data, uint8_t len);
    int16_t readData(uint8_t Reg, uint8_t *Data, uint8_t len);
  private:
    HardwareSerial *_pharduart;
};

#endif
#endif