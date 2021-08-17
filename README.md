# DFRobot_ICG20660L.h
6-axis MEMS sensor library for Arduino. ICG20660L combines a 3-axis gyroscope, 3-axis accelerometer.<br>
It support two communication interface: IIC(100~400KHz) and SPI(7MHz)<br>
Features:<br>
* Accelerometer scale range: ±2g、±4g、±8g、±16g, g = 9.80665 m/s².<br>
* Gyro scale range: ±125dps、±250dps、±500dps, 1dps = Π/180° rad/s, Π = 3.1415926<br>
* Support 512 bytes FIFO.<br>
* Support wake on motion for accelerometer. 发生条件：如果前一次和后一次测得的加速度的阈值差大于或等于设定的阈值，将产生中断<br>
* 加速度的X，Y,Z，温度，陀螺仪的x，Y，Z轴都可以单独使用和禁用，温度除外，必须和其中一轴数据一起采集。禁止某轴或温度可以降低功耗<br>
* 支持进入低功耗睡眠模式，该模式下，陀螺仪，加速度，温度等传感器都将停止采样<br>
* 加速度支持低功耗模式，在该模式下陀螺仪将禁止工作。<br>
* IIC可通过SDO的高低电平来改变地址，为高0x69， 为低0x68<br>
* 支持从寄存器或FIFO读取数据，但是从FIFO读取必须使能加速度、陀螺仪、温度等所有功能，并将陀螺仪和加速度的内部采样设置成一样。<br>

![正反面svg效果图](https://github.com/Arya11111/DFRobot_MCP23017/blob/master/resources/images/SEN0245svg1.png)


## Product Link（链接到英文商城）
    
   
## Table of Contents

* [Summary](#summary)
* [Connected](#connected)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary
This is a 6-axis MEMS sensor ICG-20660L library. It can only support IIC and SPI communication.<br>
It can detect accelerometer, gyroscope, and onboard temperature.<br>

## Connected
Hardware conneted table<br>
### hardware conneted table in SPI
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
### hardware conneted table in IIC
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
 * @brief The constructor of the ICG20660L sensor using IIC communication.
 * @param addr:  7-bit IIC address, controlled by SDO pin.
 * @n     IIC_ADDR_SDO_H or 0x69:  SDO pull high.(default)
 * @n     IIC_ADDR_SDO_L or 0x68:  SDO pull down.
 * @param pWire:   TwoWire class pointer.
 */
DFRobot_ICG20660L_IIC(uint8_t addr = IIC_ADDR_SDO_H, TwoWire *pWire = &Wire);

/**
 * @brief The constructor of the ICG20660L sensor using SPI communication.
 * @param csPin:  SPI chip select pin, connected to IO pin of MCU.
 * @param spi: SPIClass class pointer.
 */
DFRobot_ICG20660L_SPI(int csPin, SPIClass *spi);

/**
 * @brief Init sensor, 初始化后，所有传感器都被关闭，需通过enableSensor打开相应的配置. 
 * @param mode: Enum variable,from eDataReadMode_t,配置读取传感器数据是从FIFO还是从寄存器。
 * @n     eRegMode:  配置为从寄存器读取传感器数据
 * @n     eFIFOMode: 从512字节FIFO读取数据,注意：从FIFO读取，加速度，陀螺仪、温度必须全部使能，且将其内部采样率必须配置成一致
 * @return status:
 * @n      0 :   Initialization sucess.
 * @n      -1:   Interface Initialization failed(IIC or SPI).
 * @n      -2:   读取设备ID失败，ID不是0x91
 */
int begin(eDataReadMode_t  mode = eRegMode);

/**
 * @brief Get device ID, ICG20660L is 0x91 (145).
 * @return  If device is ICG20660L, it will return 0x91.
 */
uint8_t readID();

/**
 * @brief Enable sensor, Include Accel of xyz axis, Gyro of xyz, temperature. 
 * @param bit: 8位字节数据，每一位都代表使能一个功能位，如下表所示：
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
 * @n Note: 使能陀螺仪的任意轴，都会自动使能传感器板载温度传感器。
 * @n   eGyroAxisZ: The bit0 of the bit, enable gyro's z axis and temperature.
 * @n   eGyroAxisY: The bit1 of the bit, enable gyro's y axis and temperature.
 * @n   eGyroAxisX: The bit2 of the bit, enable gyro's X axis and temperature.
 * @n   eAccelAxisZ: The bit3 of the bit, enable accel's z axis.
 * @n   eAccelAxisY: The bit4 of the bit, enable Accel's y axis.
 * @n   eAccelAxisX: The bit5 of the bit, enable Accel's X axis.
 * @n   eGyroAxisXYZ or eGyroAxisX|eGyroAxisY|eGyroAxisZ: The bit0/bit1/bit2 of the bit, enable gyro's xyz axis and temperature.
 * @n   eAccelAxisXYZ or eAccelAxisX|eAccelAxisY|eAccelAxisZ: The bit3/bit4/bit5 of the bit, enable Accel's xyz axis.
 * @n   eAxisAll or eGyroAxisX|eGyroAxisY|eGyroAxisZ|eAccelAxisX|eAccelAxisY|eAccelAxisZ: The bit0/bit1/bit2/bit3/bit4/bit5 of the bit, enable temperature, Accel's and gyro's xyz axis. 
 */
void enableSensor(uint8_t bit);

/**
 * @brief Disable sensor, Include Accel of xyz axis, Gyro of xyz, temperature. 
 * @param bit: 8位字节数据，每一位都代表使能一个功能位，如下表所示：
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
 * @n Note: 只有当陀螺仪的X,Y,Z轴全部关闭的时候，才会关闭温度传感器，任意一轴开启，都不会关闭温度传感器
 * @n   eGyroAxisZ: The bit0 of the bit, disable gyro's z axis.
 * @n   eGyroAxisY: The bit1 of the bit, disable gyro's y axis.
 * @n   eGyroAxisX: The bit2 of the bit, disable gyro's X axis.
 * @n   eAccelAxisZ: The bit3 of the bit, disable accel's z axis.
 * @n   eAccelAxisY: The bit4 of the bit, disable Accel's y axis.
 * @n   eAccelAxisX: The bit5 of the bit, disable Accel's X axis.
 * @n   eGyroAxisXYZ or eGyroAxisX|eGyroAxisY|eGyroAxisZ: The bit0/bit1/bit2 of the bit, disable gyro's xyz axis and temperature.
 * @n   eAccelAxisXYZ or eAccelAxisX|eAccelAxisY|eAccelAxisZ: The bit3/bit4/bit5 of the bit, disable Accel's xyz axis.
 * @n   eAxisAll or eGyroAxisX|eGyroAxisY|eGyroAxisZ|eAccelAxisX|eAccelAxisY|eAccelAxisZ: The bit0/bit1/bit2/bit3/bit4/bit5 of the bit, disable temperature, Accel's and gyro's xyz axis. 
 */
void disableSensor(uint8_t bit);

/**
 * @brief Config of gyro's full scale 、dlpf bandwidth and internal sample rate. 
 * @param scale  The full scale of gyro, unit: dps(Degrees per second).
 * @n     eFSR_G_125DPS:  The full scale range is ±125 dps.
 * @n     eFSR_G_250DPS:  The full scale range is ±250 dps.
 * @n     eFSR_G_500DPS:  The full scale range is ±500 dps.
 * @param bd  Set 3-db bandwidth.
 * @n     eGyro_DLPF_8173_32KHZ:    当信号等于或大于8173Hz时，会出现明显衰减，衰减3-db，内部采样率为32KHz
 * @n     eGyro_DLPF_3281_32KHZ: 当信号等于或大于3281Hz时，会出现明显衰减，衰减3-db，内部采样率为32KHz
 * @n     eGyro_DLPF_250_8KHZ:     当信号等于或大于250Hz时，会出现明显衰减，衰减3-db，内部采样率为8KHz
 * @n     eGyro_DLPF_176_1KHZ:     当信号等于或大于176Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
 * @n     eGyro_DLPF_92_1KHZ:      当信号等于或大于92Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
 * @n     eGyro_DLPF_3281_8KHZ:  当信号等于或大于3281Hz时，会出现明显衰减，衰减3-db，内部采样率为8KHz
 * @n 注意：当陀螺仪和加速度都使能的时候，如果通过FIFO读取传感器数据，必须保证陀螺仪和加速度的内部采样率一致
 */
void configGyro(eGyroFSR_t scale, eGyroBandwidth_t  bd);
void configGyro(uint8_t scale, uint8_t  bd);

/**
 * @brief Config of accel's full scale 、dlpf bandwidth and internal sample rate. 
 * @param scale  The full scale of accel, unit: g(1g = 9.80665 m/s²).
 * @n     eFSR_A_2G:  The full scale range is ±2g.
 * @n     eFSR_A_4G:  The full scale range is ±4g.
 * @n     eFSR_A_8G:  The full scale range is ±8g.
 * @n     eFSR_A_16G:  The full scale range is ±16g.
 * @param bd  Set 3-db bandwidth.
 * @n     eAccel_DLPF_5_1KHZ or 0:    当信号小于或等于5Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
 * @n     eAccel_DLPF_10_1KHZ or 1:   当信号小于或等于10Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
 * @n     eAccel_DLPF_21_1KHZ or 2:   当信号小于或等于21Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
 * @n     eAccel_DLPF_44_1KHZ or 3:   当信号小于或等于44Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
 * @n     eAccel_DLPF_99_1KHZ or 4:   当信号小于或等于99Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
 * @n     eAccel_DLPF_218_1KHZ or 5:  当信号小于或等于218Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz，支持低功耗模式
 * @n     eAccel_DLPF_420_1KHZ or 6:  当信号小于或等于420Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz，支持低功耗模式
 * @n     eAccel_DLPF_1046_4KHZ or 7: 当信号小于或等于1046Hz时，会出现明显衰减，衰减3-db，内部采样率为4KHz，支持低功耗模式
 * @n     eAccel_DLPF_55_1KHZ or 8:   当信号小于或等于55Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz，仅支持低功耗模式
 * @n     eAccel_DLPF_110_1KHZ or 9:  当信号小于或等于110Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz，仅支持低功耗模式
 * @n 注意：当陀螺仪和加速度都使能的时候，如果通过FIFO读取传感器数据，必须保证陀螺仪和加速度的内部采样率一致
 * @param odr:  Sets the frequency of waking up the chip to take a sample of accel data – the low power accel Output Data Rate.
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
 * @brief Set sample rate divider. 
 * @param div  Sample rate divider, the range is 0~255.
 * @n     采样率 = 内部采样率/(div+1)
 * @n Note: 如果加速度配置为低功耗模式，即configAccel函数的形参lowPowerFlag为true，则采样率必须和configAccel的形参odr输出率相匹配，如下表所示：
 * @n ----------------------------------------------------------------------------
 * @n |                           configAccel                    |  setSampleDiv  |
 * @n ----------------------------------------------------------------------------|
 * @n |            bd             |      odr      | lowPowerFlag |      div       |
 * @n ----------------------------------------------------------------------------|
 * @n |            X              |       X       |    false     |      0~255     |
 * @n ----------------------------------------------------------------------------|
 * @n |                           |  eODR_125Hz   |    true      |        7       |
 * @n |                           |-----------------------------------------------|
 * @n |  支持低功耗模式的bd       |  eODR_250Hz   |    true      |        3       |
 * @n |                           |-----------------------------------------------|
 * @n |                           |  eODR_500Hz   |    true      |        1       |
 * @n |---------------------------------------------------------------------------|
 */
 void setSampleDiv(uint8_t div);

/**
 * @brief Reset, the register will restore the initial value, you need to call begin to configuration.
 */
void reset();

/**
 * @brief Enter sleep mode, it will reduce power consumption, and The gyroscope and acceleration will stop working.
 * @n You need to call wakeup function to wake up sensor.
 */
void sleep();

/**
 * @brief Waking up sensor from sleep, and you will restore the configuration before sleep.
 */
void wakeup();
  
/**
 * @brief 设置触发加速度传感器唤醒运动中断时，INT引脚的电平极性。
 * @param polarity: 触发唤醒运动时，传感器INT引脚的电平信号。
 * @n     HIGH:  INT引脚初始信号为LOW，当产生加速度唤醒运动时，INT引脚电平信号将变为HIGH，需要调用readINTStatus函数，才能清除该信号，重新恢复初始信号。
 * @n     LOW:   INT引脚初始信号为HIGH，当产生加速度唤醒运动时，INT引脚电平信号将变为LOW，需要调用readINTStatus函数，才能清除该信号，重新恢复初始信号。
 * @n Note: 触发加速度唤醒运动后，如果不调用readINTStatus函数清除该标志，INT引脚将一直保持触发运动时的电平极性。
 */
void setINTPinMotionTriggerPolarity(int polarity);
  
/**
 * @brief Get the polarity of the INT pin of sensor when the sensor INT pin triggers an interrupt.
 * @return The level signal when the INT pin triggers an interrupt.
 * @n      HIGH:  INT pin level held  HIGH LEVEL until interrupt status is cleared.
 * @n      LOW:   INT pin level held  LOW LEVEL until interrupt status is cleared.
 */
int getINTPinMotionTriggerPolarity();
  
/**
 * @brief Set the threshold value for the Wake on Motion Interrupt for accelerometer. 
 * @param level: WoM thresholds are expressed in fixed “mg” independent of the selected Range [0g : 1g]; Resolution 1g/256=~3.9mg
 * @n     level = 0~255
 * @return Actul WoM thresholds, unit : g   re_value = (level * 3.9)/1000 g
 */
float setWakeOnMotionThresholdForAccel(uint8_t level);

/**
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
 * @brief Get Sensor's accel, gyro and temperature data.
 * @param accel: sIcg20660SensorData_t structure pointer which point to accel or NULL.
 * @param gyro: sIcg20660SensorData_t structure pointer which point to gyro or NULL.
 * @param t:  A float pointer which point to temperature or NULL.
 */
void getSensorData(sIcg20660SensorData_t *accel, sIcg20660SensorData_t *gyro, float *t);

/**
 * @brief Get X axis acceleration, unit g.
 * @return  X axis acceleration.
 */
float getAccelDataX();

/**
 * @brief Get Y axis acceleration, unit g.
 * @return  Y axis acceleration.
 */
float getAccelDataY();

/**
 * @brief Get Z axis acceleration, unit g.
 * @return  Z axis acceleration.
 */
float getAccelDataZ();

/**
 * @brief Get temperature data, uint: ℃.
 * @return  Temperature data.
 */
float getTemperatureC();
  
/**
 * @brief Get X-axis gyroscope speed, unit dps.
 * @return  X-axis gyroscope speed.
 */
float getGyroDataX();

/**
 * @brief Get Y-axis gyroscope speed, unit dps.
 * @return  Y-axis gyroscope speed.
 */
float getGyroDataY();

/**
 * @brief Get Z-axis gyroscope speed, unit dps.
 * @return  Z-axis gyroscope speed.
 */
float getGyroDataZ();

/**
 * @brief Get 14 bytes raw data, include accel, gyro, and temperature.
 * @param data:  存放14字节原始数据的buffer。
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
 * @n Note: You can use RAW_DATA_LENGTH to creat data Arrya, and you can use  
 * @n RAW_DATA_AX_H_INDEX, RAW_DATA_AX_L_INDEX, RAW_DATA_AY_H_INDEX, RAW_DATA_AY_L_INDEX, RAW_DATA_AZ_H_INDEX, RAW_DATA_AZ_L_INDEX,
 * @n RAW_DATA_T_H_INDEX, RAW_DATA_T_L_INDEX,RAW_DATA_GX_H_INDEX, RAW_DATA_GX_L_INDEX, 
 * @n RAW_DATA_GY_H_INDEX, RAW_DATA_GY_L_INDEX, RAW_DATA_GZ_H_INDEX, RAW_DATA_GZ_L_INDEX or 0~13 to index data array.
 * @param len: The length of data array.
 */
void getRawData(uint8_t *data, uint8_t len = 0);
```

## Compatibility

MCU                |   Compatibility  |
------------------ | :----------: |
Arduino Uno        |      X       |
Mega2560           |      √       |
Leonardo           |      X       |
ESP32              |      √       |
ESP8266            |      √       |
micro:bit          |      √       |
FireBeetle M0      |      √       |
raspberry          |      √       |

## History

- Data 2021-06-1
- Version V1.0

## Credits

Written by(xue.peng@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))





