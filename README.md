DFRobot_18B20_RS485
===========================
* [中文版](./README_CN.md)

This is a 6-axis MEMS sensor library for Arduino. ICG20660L combines a 3-axis gyroscope and 3-axis accelerometer.<br>
It supports two communication interfaces: IIC(100~400KHz) and SPI(7MHz)<br>
Features:<br>
* Accelerometer scale range: ±2g, ±4g, ±8g, ±16g, g = 9.80665 m/s².<br>
* Gyro scale range: ±125dps, ±250dps, ±500dps, 1dps = Π/180° rad/s, Π = 3.1415926<br>
* Support 512 bytes FIFO.<br>
* Support wake on motion for accelerometer.  Condition: If the threshold difference of measured accelerometer between the previous and the next is greater than or equal to the set threshold, an interrupt will be generated.<br>
* X, Y, Z axes of accelerometer and gyroscope can be enabled and disabled individually. Temperature can be disabled individually, but it must be used with one of the axis data. 
* Disabling one axis or temperature can reduce power consumption.<br>
* Support low-power sleep mode. In this mode, sensors like gyroscope, accelerometer, temperature will stop sampling.<br>
* Accelerometer supports low power consumption mode, in which the gyroscope will be forbidden to work.<br>
* IIC address can be changed by changing the level of SDO. If in level high, the address is 0x69 and if low, then 0x68.<br>
* Support to read data from registers or FIFO, but for reading from FIFO, you must enable all functions including accelerometer, gyroscope and temperature, and the internal sampling settings of gyroscope and accelerometer should be the same.<br>

![产品效果图](./resources/images/SEN0443.png)


## Product Link（[https://www.dfrobot.com/product-2433.html](https://www.dfrobot.com/product-2433.html)）
    SKU: SEN0443

## Table of Contents

* [Summary](#summary)
* [Connected](#connected)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary
This is a 6-axis MEMS sensor ICG-20660L library. It supports IIC and SPI communication.<br>
It has a 3-axis accelerometer, 3-axis gyroscope and onboard temperature.<br>

## Connected
Hardware connection table<br>
### Hardware connection table in SPI
 Sensor      |                      MCU                          |
------------ | :------------------------------------------------:|
FSY          | not connected, floating                           |
INT          | connected to the external interrupt IO pin of MCU |
CS           | connected to the IO pin of MCU                    |
SDO          | connected to miso of mcu'spi                      |
SDI          | connected to mosi of mcu'spi                      |
SCK          | connected to sck of mcu'spi                       |
GND          | GND                                               |
3V3/VCC      | 3V3/VCC                                           |
### Hardware connection table in IIC
 Sensor      |                      MCU                          |
------------ | :------------------------------------------------:|
FSY          | not connected, floating                           |
INT          | connected to the external interrupt IO pin of MCU |
SDA          | connected to SDA of mcu'iic                       |
SCL          | connected to scl of mcu'iic                       |
GND          | GND                                               |
3V3/VCC      | 3V3/VCC                                           |  

## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Methods

```C++
  /**
   * @fn DFRobot_ICG20660L_IIC
   * @brief The constructor of the ICG20660L sensor, using IIC communication.
   * @param addr:  7-bit IIC address, controlled by SDO pin.
   * @n     IIC_ADDR_SDO_H or 0x69:  SDO pull high.(default)
   * @n     IIC_ADDR_SDO_L or 0x68:  SDO pull down.
   * @param pWire:   TwoWire class pointer.
   */
  DFRobot_ICG20660L_IIC(uint8_t addr = IIC_ADDR_SDO_H, TwoWire *pWire = &Wire);
  
  /**
   * @fn DFRobot_ICG20660L_SPI
   * @brief The constructor of the ICG20660L sensor, using SPI communication.
   * @param csPin:  SPI chip select pin, connected to IO pin of MCU.
   * @param spi: SPIClass class pointer.
   */
  DFRobot_ICG20660L_SPI(int csPin, SPIClass *spi);
  
  /**
   * @fn begin
   * @brief Initialize the sensor, after initialization, all sensors are turned off, and the corresponding configuration
   * @n needs to be turned on through enableSensor.
   * @param mode: Enum variable,from eDataReadMode_t, configure to read sensor data from FIFO or register?
   * @n     eRegMode: Read sensor data from registers.
   * @n     eFIFOMode:Read sensor data from 512 bytes FIFO. 
   * @note Read from FIFO, accelerometer, gyroscope and temperature must all be enabled,  and the internal sampling rate must be configured to be consistent. 
   * @return status:
   * @n      0 :   Initialization success.
   * @n      -1:   Interface initialization failed(IIC or SPI).
   * @n      -2:   Failed to read the device ID, the ID is not 0x91
   */
  int begin(eDataReadMode_t  mode = eRegMode);
  
  /**
   * @fn readID
   * @brief Get device ID, ICG20660L is 0x91 (145).
   * @return  If device is ICG20660L, it will return 0x91.
   */
  uint8_t readID();
  
  /**
   * @fn enableSensor
   * @brief Enable sensor, including Accel of xyz axis, Gyro of xyz, temperature. 
   * @param bit: 8-bit byte data. Each bit represents enabling a function bit, as shown in the following table:
   * @n -------------------------------------------------------------------------------------------------------------------
   * @n |       bit7      |     bit6     |      bit5   |    bit4     |     bit3    |     bit2   |    bit1    |    bit0    |
   * @n -------------------------------------------------------------------------------------------------------------------
   * @n |     reserve     |    reserve   | eAccelAxisX | eAccelAxisY | eAccelAxisZ | eGyroAxisX | eGyroAxisY | eGyroAxisZ |
   * @n |                                |            eAccelAxisXYZ                |           eGyroAxisXYZ               |
   * @n |                                |                                eAxisAll                                        |
   * @n -------------------------------------------------------------------------------------------------------------------
   * @n   bit0:  Z-axis of gyro and temperature.
   * @n   bit1:  Y-axis of gyro and temperature.
   * @n   bit2:  X-axis of gyro and temperature.
   * @n   bit3:  Z-axis of acceleration.
   * @n   bit4:  Z-axis of acceleration.
   * @n   bit5:  Z-axis of acceleration.
   * @n   bit6:  reserve.
   * @n   bit7:  reserve.
   * @note Enabling any axis of the gyroscope will automatically enable the on-board temperature sensor.
   * @n   eGyroAxisZ: The bit0 of the bit, enable gyro's z axis and temperature.
   * @n   eGyroAxisY: The bit1 of the bit, enable gyro's y axis and temperature.
   * @n   eGyroAxisX: The bit2 of the bit, enable gyro's X axis and temperature.
   * @n   eAccelAxisZ: The bit3 of the bit, enable accel's z axis.
   * @n   eAccelAxisY: The bit4 of the bit, enable Accel's y axis.
   * @n   eAccelAxisX: The bit5 of the bit, enable Accel's X axis.
   * @n   eGyroAxisXYZ or eGyroAxisX|eGyroAxisY|eGyroAxisZ: The bit0/bit1/bit2 of the bit, enable gyro's xyz axis and temperature.
   * @n   eAccelAxisXYZ or eAccelAxisX|eAccelAxisY|eAccelAxisZ: The bit3/bit4/bit5 of the bit, enable Accel's xyz axis.
   * @n   eAxisAll or eGyroAxisX|eGyroAxisY|eGyroAxisZ|eAccelAxisX|eAccelAxisY|eAccelAxisZ: The bit0/bit1/bit2/bit3/bit4/bit5 of the bit,
   * @n enable temperature, Accel's and gyro's xyz axis. 
   */
  void enableSensor(uint8_t bit);
  
  /**
   * @fn disableSensor
   * @brief Disable sensor, including Accel of xyz axis, Gyro of xyz, temperature. 
   * @param bit: 8-bit byte data. Each bit represents enabling a function bit, as shown in the following table:
   * @n -------------------------------------------------------------------------------------------------------------------
   * @n |       bit7      |     bit6     |      bit5   |    bit4     |     bit3    |     bit2   |    bit1    |    bit0    |
   * @n -------------------------------------------------------------------------------------------------------------------
   * @n |     reserve     |    reserve   | eAccelAxisX | eAccelAxisY | eAccelAxisZ | eGyroAxisX | eGyroAxisY | eGyroAxisZ |
   * @n |                                |            eAccelAxisXYZ                |           eGyroAxisXYZ               |
   * @n |                                |                                eAxisAll                                        |
   * @n -------------------------------------------------------------------------------------------------------------------
   * @n   bit0:  Z-axis of gyro and temperature.
   * @n   bit1:  Y-axis of gyro and temperature.
   * @n   bit2:  X-axis of gyro and temperature.
   * @n   bit3:  Z-axis of acceleration.
   * @n   bit4:  Z-axis of acceleration.
   * @n   bit5:  Z-axis of acceleration.
   * @n   bit6:  reserve.
   * @n   bit7:  reserve.
   * @note Only when the X, Y, and Z axes of the gyroscope are all closed, the temperature sensor will be turned off. 
   * @n Any axis’s turning on will make the temperature sensor not be turned off.
   * @n   eGyroAxisZ: The bit0 of the bit, disable gyro's z axis.
   * @n   eGyroAxisY: The bit1 of the bit, disable gyro's y axis.
   * @n   eGyroAxisX: The bit2 of the bit, disable gyro's X axis.
   * @n   eAccelAxisZ: The bit3 of the bit, disable accel's z axis.
   * @n   eAccelAxisY: The bit4 of the bit, disable Accel's y axis.
   * @n   eAccelAxisX: The bit5 of the bit, disable Accel's X axis.
   * @n   eGyroAxisXYZ or eGyroAxisX|eGyroAxisY|eGyroAxisZ: The bit0/bit1/bit2 of the bit, disable gyro's xyz axis and temperature.
   * @n   eAccelAxisXYZ or eAccelAxisX|eAccelAxisY|eAccelAxisZ: The bit3/bit4/bit5 of the bit, disable Accel's xyz axis.
   * @n   eAxisAll or eGyroAxisX|eGyroAxisY|eGyroAxisZ|eAccelAxisX|eAccelAxisY|eAccelAxisZ: The bit0/bit1/bit2/bit3/bit4/bit5 of the bit, 
   * @n disable temperature, Accel's and gyro's xyz axis. 
   */
  void disableSensor(uint8_t bit);
  
  /**
   * @fn configGyro
   * @brief Config of gyro's full scale, dlpf bandwidth and internal sample rate. 
   * @param scale  The full scale of gyro, unit: dps(Degrees per second).
   * @n     eFSR_G_125DPS:  The full scale range is ±125 dps.
   * @n     eFSR_G_250DPS:  The full scale range is ±250 dps.
   * @n     eFSR_G_500DPS:  The full scale range is ±500 dps.
   * @param bd  Set 3-db bandwidth.
   * @n     eGyro_DLPF_8173_32KHZ: When the signal is equal to or greater than 8173Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 32KHz.
   * @n     eGyro_DLPF_3281_32KHZ: When the signal is equal to or greater than 3281Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 32KHz.
   * @n     eGyro_DLPF_250_8KHZ:   When the signal is equal to or greater than 250Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 8KHz.
   * @n     eGyro_DLPF_176_1KHZ:   When the signal is equal to or greater than 176Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
   * @n     eGyro_DLPF_92_1KHZ:    When the signal is equal to or greater than 92Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
   * @n     eGyro_DLPF_3281_8KHZ:  When the signal is equal to or greater than 3281Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 8KHz.
   * @note When the gyroscope and accelerometer are both enabled, if the sensor data is read through the FIFO,the internal sampling rate of the gyroscope and accelerometer must be the same.
   */
  void configGyro(eGyroFSR_t scale, eGyroBandwidth_t  bd);
  void configGyro(uint8_t scale, uint8_t  bd);
  
  /**
   * @fn configAccel
   * @brief Config of accel's full scale, dlpf bandwidth and internal sample rate. 
   * @param scale  The full scale of accel, unit: g(1g = 9.80665 m/s²).
   * @n     eFSR_A_2G:  The full scale range is ±2g.
   * @n     eFSR_A_4G:  The full scale range is ±4g.
   * @n     eFSR_A_8G:  The full scale range is ±8g.
   * @n     eFSR_A_16G:  The full scale range is ±16g.
   * @param bd  Set 3-db bandwidth.
   * @n     eAccel_DLPF_5_1KHZ or 0:    When the signal is less than or equal to 5Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
   * @n     eAccel_DLPF_10_1KHZ or 1:   When the signal is less than or equal to 10Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
   * @n     eAccel_DLPF_21_1KHZ or 2:   When the signal is less than or equal to 21Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
   * @n     eAccel_DLPF_44_1KHZ or 3:   When the signal is less than or equal to 44Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
   * @n     eAccel_DLPF_99_1KHZ or 4:   When the signal is less than or equal to 99Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
   * @n     eAccel_DLPF_218_1KHZ or 5:  When the signal is less than or equal to 218Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.    Support low power consumption mode
   * @n     eAccel_DLPF_420_1KHZ or 6:  When the signal is less than or equal to 420Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.   Support low power consumption mode
   * @n     eAccel_DLPF_1046_4KHZ or 7: When the signal is less than or equal to 1046Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 4KHz.   Support low power consumption mode
   * @n     eAccel_DLPF_55_1KHZ or 8:   When the signal is less than or equal to 55Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.   Only support low power consumption mode
   * @n     eAccel_DLPF_110_1KHZ or 9:  When the signal is less than or equal to 110Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.    Only support low power consumption mode
   * @note When the gyroscope and accelerometer are both enabled, if the sensor data is read through the FIFO, 
   * @n the internal sampling rate of the gyroscope and accelerometer must be the same.
   * @param odr:  Set the frequency of waking up the chip to take a sample of accel data – the low power accel Output Data Rate.
   * @n     eODR_125Hz or 9:    The low power accel Output Data Rate: 125Hz
   * @n     eODR_250Hz or 10:   The low power accel Output Data Rate: 250Hz
   * @n     eODR_500Hz or 11:   The low power accel Output Data Rate: 500Hz
   * @param lowPowerFlag:  Whether to configure the Acceleration to low power mode.
   * @n     true:          Enter low power mode.
   * @n     false:         Not configure the Acceleration to low power mode.(default)
   */
  void configAccel(eAccelFSR_t scale, eAccelBandwidth_t bd, eODR_t odr = eODR_0_24Hz, bool lowPowerFlag = false);
  void configAccel(uint8_t scale, uint8_t bd, uint8_t odr = 0, bool lowPowerFlag = false);
  
  /**
   * @fn setSampleDiv
   * @brief Set sample rate divider. 
   * @param div  Sample rate divider, the range is 0~255.
   * @n     Sampling rate = internal sampling rate/(div+1)
   * @note  If the accelerometer configuration is in low power consumption mode, that is, the formal parameter lowPowerFlag of the configAccel function is true, 
   * @n the sampling rate must match the output rate of the formal parameter odr of configAccel, as shown in the following table:
   * @n ----------------------------------------------------------------------------
   * @n |                           configAccel                    |  setSampleDiv  |
   * @n ----------------------------------------------------------------------------|
   * @n |            bd             |      odr      | lowPowerFlag |      div       |
   * @n ----------------------------------------------------------------------------|
   * @n |            X              |       X       |    false     |      0~255     |
   * @n ----------------------------------------------------------------------------|
   * @n |                           |  eODR_125Hz   |    true      |        7       |
   * @n |                           |-----------------------------------------------|
   * @n |bd of supporting low power consumption mode|  eODR_250Hz   |    true      |        3       |
   * @n |                           |-----------------------------------------------|
   * @n |                           |  eODR_500Hz   |    true      |        1       |
   * @n |---------------------------------------------------------------------------|
   */
   void setSampleDiv(uint8_t div);
  
  /**
   * @fn reset
   * @brief Reset, the register will restore the initial value, and you need to call begin to configuration.
   */
  void reset();
  
  /**
   * @fn sleep
   * @brief Entering sleep mode, it will reduce power consumption, and The gyroscope and acceleration will stop working.
   * @n You need to call wakeup function to wake up sensor.
   */
  void sleep();
  
  /**
   * @fn wakeup
   * @brief Wake up sensor from sleep, and you will restore the configuration before sleep.
   */
  void wakeup();
    
  /**
   * @fn setINTPinMotionTriggerPolarity
   * @brief Set the level polarity of the INT pin when the accelerometer sensor is triggered to wake up the motion interrupt.
   * @param polarity: the level signal of the sensor INT pin when the wake-up motion is triggered
   * @n     HIGH:The initial signal of the pin is LOW. When an accelerometer wake-up motion occurs, the level signal of the INT pin will change to HIGH. 
   * @n Then the readINTStatus function needs to be called to clear the signal and restore the initial signal.
   * @n     LOW: The initial signal of the pin is HIGH. When an accelerometer wake-up motion occurs, the level signal of the INT pin will change to LOW. 
   * @n Then the readINTStatus function needs to be called to clear the signal and restore the initial signal.
   * @note After triggering the accelerometer wake-up motion, if the read_int_status function is not called to clear the sign, 
   * @n the INT pin will always maintain the level polarity when the motion is triggered.
   */
  void setINTPinMotionTriggerPolarity(int polarity);
    
  /**
   * @fn getINTPinMotionTriggerPolarity
   * @brief Get the polarity of the INT pin of sensor when the sensor INT pin triggers an interrupt.
   * @return The level signal when the INT pin triggers an interrupt.
   * @n      HIGH:  INT pin level held  HIGH LEVEL until interrupt status is cleared.
   * @n      LOW:   INT pin level held  LOW LEVEL until interrupt status is cleared.
   */
  int getINTPinMotionTriggerPolarity();
    
  /**
   * @fn setWakeOnMotionThresholdForAccel
   * @brief Set the threshold value for the Wake on Motion Interrupt for accelerometer. 
   * @param level: WoM thresholds are expressed in fixed “mg” independent of the selected Range [0g : 1g]; Resolution 1g/256=~3.9mg
   * @n     level = 0~255
   * @return Actul WoM thresholds, unit : g   re_value = (level * 3.9)/1000 g
   */
  float setWakeOnMotionThresholdForAccel(uint8_t level);
  
  /**
   * @fn readINTStatus
   * @brief Read interrupt status register, and clear INT pin's interrupt signal. 
   * @return Interrupt status register value.
   * @n  INT_STATUS register：addr:0x3A,acess:rw
   * @n  ------------------------------------------------------------------------------------
   * @n  |     b7    |    b6     |    b5     |      b4        | b3 | b2 | b1 |      b0      |
   * @n  ------------------------------------------------------------------------------------
   * @n  |             WOM_XYZ_INT           | FIFO_OFLOW_INT |     rsv      | DATA_RDY_INT |
   * @n  ------------------------------------------------------------------------------------
   * @n  DATA_RDY_INT  : This bit automatically sets to 1 when a Data Ready interrupt is generated. The bit clears to 0 after the register has been read.
   * @n  rsv           : reserve
   * @n  FIFO_OFLOW_INT: This bit automatically sets to 1 when a FIFO buffer overflow has been generated. The bit clears to 0 after the register has been read.
   * @n  WOM_XYZ_INT   : These bits automatically set to a non-zero number when the X-axis,Y-axis or Z-axis of accelerometer which trigger WOM(wake on motion) 
   * @n                  interrupt.Cleared on Read.
   */
  uint8_t readINTStatus();
  
  /**
   * @fn getSensorData
   * @brief Get Sensor's accel, gyro and temperature data.
   * @param accel: sIcg20660SensorData_t structure pointer which points to accel or NULL.
   * @param gyro: sIcg20660SensorData_t structure pointer which points to gyro or NULL.
   * @param t:  A float pointer which points to temperature or NULL.
   */
  void getSensorData(sIcg20660SensorData_t *accel, sIcg20660SensorData_t *gyro, float *t);
  
  /**
   * @fn getAccelDataX
   * @brief Get X axis acceleration, unit g.
   * @return  X axis acceleration.
   */
  float getAccelDataX();
  
  /**
   * @fn getAccelDataY
   * @brief Get Y axis acceleration, unit g.
   * @return  Y axis acceleration.
   */
  float getAccelDataY();
  
  /**
   * @fn getAccelDataZ
   * @brief Get Z axis acceleration, unit g.
   * @return  Z axis acceleration.
   */
  float getAccelDataZ();
  
  /**
   * @fn getTemperatureC
   * @brief Get temperature data, uint: ℃.
   * @return  Temperature data.
   */
  float getTemperatureC();
    
  /**
   * @fn getGyroDataX
   * @brief Get X-axis gyroscope speed, unit dps.
   * @return  X-axis gyroscope speed.
   */
  float getGyroDataX();
  
  /**
   * @fn getGyroDataY
   * @brief Get Y-axis gyroscope speed, unit dps.
   * @return  Y-axis gyroscope speed.
   */
  float getGyroDataY();
  
  /**
   * @fn getGyroDataZ
   * @brief Get Z-axis gyroscope speed, unit dps.
   * @return  Z-axis gyroscope speed.
   */
  float getGyroDataZ();
  
  /**
   * @fn getRawData
   * @brief Get 14 bytes raw data, including accel, gyro and temperature.
   * @param data:  buffer for storing 14 bytes of raw data.
   * @n     The first byte of data :  Acceleration X-axis high byte data.
   * @n     The second byte of data:  Acceleration X-axis low byte data.
   * @n     The third byte of data :  Acceleration Y-axis high byte data.
   * @n     The 4th byte of data   :  Acceleration Y-axis low byte data.
   * @n     The 5th byte of data   :  Acceleration Z-axis high byte data.
   * @n     The 6th byte of data   :  Acceleration Z-axis low byte data.
   * @n     The 7th byte of data   :  Temperature high byte data.
   * @n     The 8th byte of data   :  Temperature low byte data.
   * @n     The 9th byte of data   :  Gyro X-axis high byte data.
   * @n     The 10th byte of data  :  Gyro X-axis low byte data.
   * @n     The 11th byte of data  :  Gyro Y-axis high byte data.
   * @n     The 12th byte of data  :  Gyro Y-axis low byte data.
   * @n     The 13th byte of data  :  Gyro Z-axis high byte data.
   * @n     The 14th byte of data  :  Gyro Z-axis low byte data.
   * @note You can use RAW_DATA_LENGTH to creat data Arrya, and you can use  
   * @n RAW_DATA_AX_H_INDEX, RAW_DATA_AX_L_INDEX, RAW_DATA_AY_H_INDEX, RAW_DATA_AY_L_INDEX, RAW_DATA_AZ_H_INDEX, RAW_DATA_AZ_L_INDEX,
   * @n RAW_DATA_T_H_INDEX, RAW_DATA_T_L_INDEX,RAW_DATA_GX_H_INDEX, RAW_DATA_GX_L_INDEX, 
   * @n RAW_DATA_GY_H_INDEX, RAW_DATA_GY_L_INDEX, RAW_DATA_GZ_H_INDEX, RAW_DATA_GZ_L_INDEX or 0~13 to index data array.
   * @param len: The length of data array.
   */
  void getRawData(uint8_t *data, uint8_t len = 0);
```

## Compatibility

MCU                |  Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino Uno        |       √       |              |             | 
Mega2560           |      √       |              |             | 
Leonardo           |      √       |              |             | 
ESP32              |      √       |              |             | 
ESP8266            |      √       |              |             | 
micro:bit          |      √       |              |             | 
FireBeetle M0      |      √       |              |             | 

## History

- 2021/06/01 - Version 1.0.0 released.

## Credits

Written by Arya(xue.peng@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))





