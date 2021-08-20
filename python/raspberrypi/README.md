# DFRobot_ICG20660L.h
6-axis MEMS sensor library for Arduino. ICG20660L combines a 3-axis gyroscope, 3-axis accelerometer.<br>
It support two communication interface: IIC(100~400KHz) and SPI(7MHz)<br>
Features:<br>
* Accelerometer scale range: ±2g、±4g、±8g、±16g, g = 9.80665 m/s².<br>
* Gyro scale range: ±125dps、±250dps、±500dps, 1dps = Π/180° rad/s, Π = 3.1415926<br>
* Support 512 bytes FIFO.<br>
* Support wake on motion for accelerometer. Condition: If the threshold difference of measured accelerometer between the previous and the next is greater than or equal to the set threshold, an interrupt will be generated.<br>
* X, Y, Z axes of accelerometer and gyroscope can be enabled and disabled individually. Temperature’ s X, Y,Z axes can be disabled individually but must be enabled with one of the axis data. Disabling one axis or temperature can reduce power consumption.<br>
* Support to enter low-power sleep mode. In this mode, sensors like gyroscope, accelerometer, temperature will stop sampling.<br>
* Accelerometer supports low power consumption mode, in which the gyroscope will be forbidden to work.<br>
* IIC can change the address through the high and low level of SDO. If level is high, the address is 0x69 and if low, then 0x68.<br>
* 6.Support to read data from registers or FIFO, but reading from FIFO must enable all functions including accelerometer, gyroscope and temperature, and the internal sampling of gyroscope and accelerometer needs setting to be the same.<br>

![Front and back svg effect drawing](https://github.com/Arya11111/DFRobot_MCP23017/blob/master/resources/images/SEN0245svg1.png)


## Product Link（Link to the store）
    
   
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
 Sensor      |                   raspberry pi                    |
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
 Sensor      |                   raspberry pi                    |
------------ | :------------------------------------------------:|
FSY          | not connected, floating                           |
INT          | connected to the external interrupt IO pin of MCU |
SDA          | connected to SDA of mcu'iic                       |
SCL          | connected to scl of mcu'iic                       |
GND          | GND                                               |
3V3/VCC      | 3V3/VCC                                           |

## Installation
To use this library, first download the library file, then open the examples folder and run the demo in the folder Proceed as follows:
* sudo git clone https://github.com/DFRobot/DFRobot_SCW8916B
* cd python
* cd raspberrypi
* cd examples
* python demo_*


## Methods

```C++
'''
  @brief The constructor of the ICG20660L sensor using IIC communication.
  @param addr:  7-bit IIC address, controlled by SDO pin.
  @n     IIC_ADDR_SDO_H or 0x69:  SDO pull high.(default)
  @n     IIC_ADDR_SDO_L or 0x68:  SDO pull down.
'''
def __init__(self,addr):

'''
  @brief The constructor of the ICG20660L sensor using SPI communication.
  @param cs:  SPI chip select pin, connected to IO pin of raspberry pi.
'''
def __init__(self, cs):

'''
  @brief Initialize the sensor. After initialization, all sensors are turned off, and the corresponding configuration needs to be turned on through enableSensor.
  @param mode: Does configuration read sensor data from FIFO or register?
  @n     eREG_MODE :   Read sensor data from data register.
  @n     eFIFO_MODE:   Read sensor data from 512 bytes FIFO. Note:Read from FIFO, accelerometer, gyroscope, and temperature must all be enabled,
  @n and the internal sampling rate must be configured to be consistent.
  @return status:
  @n      0 : Initialization success.
  @n      -1: Interface Initialization failed(IIC or SPI).
  @n      -2: Failed to read the device ID, the ID is not 0x91
'''
def begin(self, mode = 0):

'''
  @brief Get device ID, ICG20660L is 0x91 (145).
  @return  If device is ICG20660L, it will return 0x91.
'''
def read_id(self):

'''
  @brief Reset, the register will restore the initial value, you need to call begin to configuration.
'''
def reset(self):

'''
  @brief Enter sleep mode, it will reduce power consumption, and The gyroscope and acceleration will stop working. 
  @n You need to call wakeup function to wake up sensor.
'''
def sleep(self):

'''
  @brief Waking up sensor from sleep, and you will restore the configuration before sleep.
'''
def wakeup(self):

'''
  @brief Enable sensor, Include Accel of xyz axis, Gyro of xyz, temperature and fifo low power enable bit. 
  @param bit: 8-bit byte data. Each bit represents enabling a function bit, as shown in the following table:
  @n -------------------------------------------------------------------------------------------------------------------
  @n |        bit7      |     bit6     |      bit5   |    bit4     |     bit3    |     bit2   |    bit1    |    bit0    |
  @n -------------------------------------------------------------------------------------------------------------------
  @n |     reserve     |    reserve    |eACCEL_AXIS_X|eACCEL_AXIS_Y|eACCEL_AXIS_Z|eGYRO_AXIS_X|eGYRO_AXIS_Y|eGYRO_AXIS_Z|
  @n |                                 |            eACCEL_AXIS_XYZ              |           eGYRO_AXIS_XYZ             |
  @n |                                 |                                eAXIS_ALL                                       |
  @n -------------------------------------------------------------------------------------------------------------------
  @n   bit0:  Z-axis of gyro and temperature.
  @n   bit1:  Y-axis of gyro and temperature.
  @n   bit2:  X-axis of gyro and temperature.
  @n   bit3:  Z-axis of acceleration.
  @n   bit4:  Z-axis of acceleration.
  @n   bit5:  Z-axis of acceleration.
  @n   bit6:  reserve.
  @n   bit7:  reserve.
  @n Note: Enabling any axis of the gyroscope will automatically enable the on-board temperature sensor.
  @n   eGYRO_AXIS_Z: The bit0 of the bit, enable gyro's z axis and temperature.
  @n   eGYRO_AXIS_Y: The bit1 of the bit, enable gyro's y axis and temperature.
  @n   eGYRO_AXIS_X: The bit2 of the bit, enable gyro's X axis and temperature.
  @n   eACCEL_AXIS_Z: The bit3 of the bit, enable accel's z axis.
  @n   eACCEL_AXIS_Y: The bit4 of the bit, enable Accel's y axis.
  @n   eACCEL_AXIS_X: The bit5 of the bit, enable Accel's X axis.
  @n   eGYRO_AXIS_XYZ or eGYRO_AXIS_X|eGYRO_AXIS_Y|eGYRO_AXIS_Z: The bit0/bit1/bit2 of the bit, enable gyro's xyz axis and temperature.
  @n   eACCEL_AXIS_XYZ or eACCEL_AXIS_X|eACCEL_AXIS_Y|eACCEL_AXIS_Z: The bit3/bit4/bit5 of the bit, enable Accel's xyz axis.
  @n   eAXIS_ALL or eGYRO_AXIS_Z|eGYRO_AXIS_Y|eGYRO_AXIS_X|eACCEL_AXIS_Z|eACCEL_AXIS_Y|eACCEL_AXIS_Z: The bit0/bit1/bit2/bit3/bit4/bit5 of the bit, enable temperature, Accel's and gyro's xyz axis. 
'''
def enable_sensor(self, bit):
    
'''
  @brief Disable sensor, Include Accel of xyz axis, Gyro of xyz, temperature and fifo low power enable bit. 
  @param bit: 8-bit byte data. Each bit represents enabling a function bit, as shown in the following table:
  @n -------------------------------------------------------------------------------------------------------------------
  @n |        bit7      |     bit6     |      bit5   |    bit4     |     bit3    |     bit2   |    bit1    |    bit0    |
  @n -------------------------------------------------------------------------------------------------------------------
  @n |     reserve     |    reserve    |eACCEL_AXIS_X|eACCEL_AXIS_Y|eACCEL_AXIS_Z|eGYRO_AXIS_X|eGYRO_AXIS_Y|eGYRO_AXIS_Z|
  @n |                                 |            eACCEL_AXIS_XYZ              |           eGYRO_AXIS_XYZ             |
  @n |                                 |                                eAXIS_ALL                                       |
  @n -------------------------------------------------------------------------------------------------------------------
  @n   bit0:  Z-axis of gyro and temperature.
  @n   bit1:  Y-axis of gyro and temperature.
  @n   bit2:  X-axis of gyro and temperature.
  @n   bit3:  Z-axis of acceleration.
  @n   bit4:  Z-axis of acceleration.
  @n   bit5:  Z-axis of acceleration.
  @n   bit6:  reserve.
  @n   bit7:  reserve.
  @n Note: Only when the X, Y, and Z axes of the gyroscope are all closed, the temperature sensor will be turned off. Any axis’s turning on will make the temperature sensor not be turned off.
  @n   eGYRO_AXIS_Z: The bit0 of the bit, disable gyro's z axis and temperature.
  @n   eGYRO_AXIS_Y: The bit1 of the bit, disable gyro's y axis and temperature.
  @n   eGYRO_AXIS_X: The bit2 of the bit, disable gyro's X axis and temperature.
  @n   eACCEL_AXIS_Z: The bit3 of the bit, disable accel's z axis.
  @n   eACCEL_AXIS_Y: The bit4 of the bit, disable Accel's y axis.
  @n   eACCEL_AXIS_X: The bit5 of the bit, disable Accel's X axis.
  @n   eGYRO_AXIS_XYZ or eGYRO_AXIS_X|eGYRO_AXIS_Y|eGYRO_AXIS_Z: The bit0/bit1/bit2 of the bit, disable gyro's xyz axis and temperature.
  @n   eACCEL_AXIS_XYZ or eACCEL_AXIS_X|eACCEL_AXIS_Y|eACCEL_AXIS_Z: The bit3/bit4/bit5 of the bit, disable Accel's xyz axis.
  @n   eAXIS_ALL or eGYRO_AXIS_Z|eGYRO_AXIS_Y|eGYRO_AXIS_X|eACCEL_AXIS_Z|eACCEL_AXIS_Y|eACCEL_AXIS_Z: The bit0/bit1/bit2/bit3/bit4/bit5 of the bit, disable temperature, Accel's and gyro's xyz axis. 
'''
def disable_sensor(self, bit):

'''
  @brief Config of accel's full scale 、dlpf bandwidth and internal sample rate. 
  @param scale  The full scale of accel, unit: g(1g = 9.80665 m/s²).
  @n     eFSR_A_2G:  The full scale range is ±2g.
  @n     eFSR_A_4G:  The full scale range is ±4g.
  @n     eFSR_A_8G:  The full scale range is ±8g.
  @n     eFSR_A_16G:  The full scale range is ±16g.
  @param bd  Set 3-db bandwidth.
  @n     eACCEL_DLPF_5_1KHZ or 0:    When the signal is less than or equal to 5Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  @n     eACCEL_DLPF_10_1KHZ or 1:   When the signal is less than or equal to 10Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  @n     eACCEL_DLPF_21_1KHZ or 2:   When the signal is less than or equal to 21Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  @n     eACCEL_DLPF_44_1KHZ or 3:   When the signal is less than or equal to 44Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  @n     eACCEL_DLPF_99_1KHZ or 4:   When the signal is less than or equal to 99Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  @n     eACCEL_DLPF_218_1KHZ or 5:  When the signal is less than or equal to 218Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz. Support low power consumption mode
  @n     eACCEL_DLPF_420_1KHZ or 6:  When the signal is less than or equal to 420Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz. Support low power consumption mode
  @n     eACCEL_DLPF_1046_4KHZ or 7: When the signal is less than or equal to 1046Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 4KHz. Support low power consumption mode
  @n     eACCEL_DLPF_55_1KHZ or 8:   When the signal is less than or equal to 55Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz. Only support low power consumption mode
  @n     eACCEL_DLPF_110_1KHZ or 9:  When the signal is less than or equal to 110Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz. Only support low power consumption mode
  @n Note: When the gyroscope and accelerometer are both enabled, if the sensor data is read through the FIFO, the internal sampling rate of the gyroscope and accelerometer must be the same.
  @param odr:  Sets the frequency of waking up the chip to take a sample of accel data – the low power accel Output Data Rate.
  @n     eODR_125HZ or 9:    The low power accel Output Data Rate: 125Hz
  @n     eODR_250HZ or 10:   The low power accel Output Data Rate: 250Hz
  @n     eODR_500HZ or 11:   The low power accel Output Data Rate: 500Hz
  @param low_power_flag:  Whether to configure the Acceleration to low power mode.
  @n     True:          Enter low power mode.
  @n     False:         Not configure the Acceleration to low power mode.(default)
'''
def config_accel(self,scale, bd, odr = 0, low_power_flag = False):

'''
  @brief Config of gyro's full scale 、dlpf bandwidth and internal sample rate. 
  @param scale  The full scale of gyro, unit: dps(Degrees per second).
  @n     eFSR_G_125DPS:  The full scale range is ±125 dps.
  @n     eFSR_G_250DPS:  The full scale range is ±250 dps.
  @n     eFSR_G_500DPS:  The full scale range is ±500 dps.
  @param bd  Set 3-db bandwidth.
  @n     eGYRO_DLPF_8173_32KHZ:  When the signal is equal to or greater than 8173Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 32KHz.
  @n     eGYRO_DLPF_3281_32KHZ: When the signal is equal to or greater than 3281Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 32KHz.
  @n     eGYRO_DLPF_250_8KHZ:   When the signal is equal to or greater than 250Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 8KHz.
  @n     eGYRO_DLPF_176_1KHZ:   When the signal is equal to or greater than 176Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  @n     eGYRO_DLPF_92_1KHZ:    When the signal is equal to or greater than 92Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  @n     eGYRO_DLPF_3281_8KHZ:  When the signal is equal to or greater than 3281Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 8KHz.
  @n When the gyroscope and accelerometer are both enabled, if the sensor data is read through the FIFO, the internal sampling rate of the gyroscope and accelerometer must be the same.
'''
def config_gyro(self, scale, bd):

'''
  @brief Set sample rate divider. 
  @param div  Sample rate divider, the range is 0~255.
  @n    Sampling rate = internal sampling rate/(div+1)
  @n Note: If the accelerometer configuration is in low power consumption mode, that is, the formal parameter lowPowerFlag of the configAccel function is true, the sampling rate must match the output rate of the formal parameter odr of configAccel , as shown in the following table:
    @n ----------------------------------------------------------------------------
    @n |                        config_accel                      | set_sample_div |
    @n ----------------------------------------------------------------------------|
    @n |            bd             |      odr      | lowPowerFlag |      div       |
    @n ----------------------------------------------------------------------------|
    @n |            X              |       X       |    false     |      0~255     |
    @n ----------------------------------------------------------------------------|
    @n |                           |  eODR_125Hz   |    true      |        7       |
    @n |                           |-----------------------------------------------|
    @n |bd of supporting low power consumption mode|  eODR_250Hz   |    true      |        3       |
    @n |                           |-----------------------------------------------|
    @n |                           |  eODR_500Hz   |    true      |        1       |
    @n |---------------------------------------------------------------------------|
  '''
  icg.set_sample_div(div = 19)

  while True:
    '''
      @brief Get 14 bytes raw data, include accel, gyro, and temperature.
      @param length: The length of return list.
      @return data:  list type, Buffer for storing 14 bytes of raw data.
      @n     The first byte of data :  Acceleration X-axis high byte data.
      @n     The second byte of data:  Acceleration X-axis low byte data.
      @n     The third byte of data :  Acceleration Y-axis high byte data.
      @n     The 4th byte of data   :  Acceleration Y-axis low byte data.
      @n     The 5th byte of data   :  Acceleration Y-axis high byte data.
      @n     The 6th byte of data   :  Acceleration Z-axis low byte data.
      @n     The 7th byte of data   :  Temperature high byte data.
      @n     The 8th byte of data   :  Temperature low byte data.
      @n     The 9th byte of data   :  Gyro X-axis high byte data.
      @n     The 10th byte of data  :  Gyro X-axis low byte data.
      @n     The 11th byte of data  :  Gyro Y-axis high byte data.
      @n     The 12th byte of data  :  Gyro Y-axis low byte data.
      @n     The 13th byte of data  :  Gyro Y-axis low byte data.
      @n     The 14th byte of data  :  Gyro Z-axis high byte data.
      @n Note: You can use RAW_DATA_LENGTH to creat data Arrya, and you can use  
      @n RAW_DATA_AX_H_INDEX, RAW_DATA_AX_L_INDEX, RAW_DATA_AY_H_INDEX, RAW_DATA_AY_L_INDEX, RAW_DATA_AZ_H_INDEX, RAW_DATA_AZ_L_INDEX,
      @n RAW_DATA_T_H_INDEX, RAW_DATA_T_L_INDEX,RAW_DATA_GX_H_INDEX, RAW_DATA_GX_L_INDEX, 
      @n RAW_DATA_GY_H_INDEX, RAW_DATA_GY_L_INDEX, RAW_DATA_GZ_H_INDEX, RAW_DATA_GZ_L_INDEX or 0~13 to index data array.
    '''
    raw_data = icg.get_raw_data(RAW_DATA_LENGTH)
    lin = ['%02X' % i for i in raw_data]
    print(" ".join(lin))

    print("RAW_DATA_AX_H_INDEX = %02X"%raw_data[RAW_DATA_AX_H_INDEX])
    print("RAW_DATA_AX_L_INDEX = %02X"%raw_data[RAW_DATA_AX_L_INDEX])
    print("RAW_DATA_AY_H_INDEX = %02X"%raw_data[RAW_DATA_AY_H_INDEX])
    print("RAW_DATA_AY_L_INDEX = %02X"%raw_data[RAW_DATA_AY_L_INDEX])
    print("RAW_DATA_AZ_H_INDEX = %02X"%raw_data[RAW_DATA_AZ_H_INDEX])
    print("RAW_DATA_AZ_L_INDEX = %02X"%raw_data[RAW_DATA_AZ_L_INDEX])
    print("RAW_DATA_T_H_INDEX  = %02X"%raw_data[RAW_DATA_T_H_INDEX])
    print("RAW_DATA_T_L_INDEX  = %02X"%raw_data[RAW_DATA_T_L_INDEX])
    print("RAW_DATA_GX_H_INDEX = %02X"%raw_data[RAW_DATA_GX_H_INDEX])
    print("RAW_DATA_GX_L_INDEX = %02X"%raw_data[RAW_DATA_GX_L_INDEX])
    print("RAW_DATA_GY_H_INDEX = %02X"%raw_data[RAW_DATA_GY_H_INDEX])
    print("RAW_DATA_GY_L_INDEX = %02X"%raw_data[RAW_DATA_GY_L_INDEX])
    print("RAW_DATA_GZ_H_INDEX = %02X"%raw_data[RAW_DATA_GZ_H_INDEX])
    print("RAW_DATA_GZ_L_INDEX = %02X"%raw_data[RAW_DATA_GZ_L_INDEX])
    print("")
    time.sleep(1)

  @n ----------------------------------------------------------------------------
  @n |                        config_accel                      | set_sample_div |
  @n ----------------------------------------------------------------------------|
  @n |            bd             |      odr      | lowPowerFlag |      div       |
  @n ----------------------------------------------------------------------------|
  @n |            X              |       X       |    false     |      0~255     |
  @n ----------------------------------------------------------------------------|
  @n |                           |  eODR_125Hz   |    true      |        7       |
  @n |                           |-----------------------------------------------|
  @n |bd of supporting low power consumption mode|  eODR_250Hz   |    true      |        3       |
  @n |                           |-----------------------------------------------|
  @n |                           |  eODR_500Hz   |    true      |        1       |
  @n |---------------------------------------------------------------------------|
'''
def set_sample_div(self, div):

'''
  @brief Get 14 bytes raw data, include accel, gyro, and temperature.
  @param length: The length of return list.
  @return data:  list type, Buffer for storing 14 bytes of raw data
  @n     The first byte of data :  Acceleration X-axis high byte data.
  @n     The second byte of data:  Acceleration X-axis low byte data.
  @n     The third byte of data :  Acceleration Y-axis high byte data.
  @n     The 4th byte of data   :  Acceleration Y-axis low byte data.
  @n     The 5th byte of data   :  Acceleration Y-axis high byte data.
  @n     The 6th byte of data   :  Acceleration Z-axis low byte data.
  @n     The 7th byte of data   :  Temperature high byte data.
  @n     The 8th byte of data   :  Temperature low byte data.
  @n     The 9th byte of data   :  Gyro X-axis high byte data.
  @n     The 10th byte of data  :  Gyro X-axis low byte data.
  @n     The 11th byte of data  :  Gyro Y-axis high byte data.
  @n     The 12th byte of data  :  Gyro Y-axis low byte data.
  @n     The 13th byte of data  :  Gyro Y-axis low byte data.
  @n     The 14th byte of data  :  Gyro Z-axis high byte data.
  @n Note: You can use RAW_DATA_LENGTH to creat data Arrya, and you can use  
  @n RAW_DATA_AX_H_INDEX, RAW_DATA_AX_L_INDEX, RAW_DATA_AY_H_INDEX, RAW_DATA_AY_L_INDEX, RAW_DATA_AZ_H_INDEX, RAW_DATA_AZ_L_INDEX,
  @n RAW_DATA_T_H_INDEX, RAW_DATA_T_L_INDEX,RAW_DATA_GX_H_INDEX, RAW_DATA_GX_L_INDEX, 
  @n RAW_DATA_GY_H_INDEX, RAW_DATA_GY_L_INDEX, RAW_DATA_GZ_H_INDEX, RAW_DATA_GZ_L_INDEX or 0~13 to index data array.
'''
def get_raw_data(self, length = 0):

'''
  @brief Get Sensor's accel, gyro and temperature data.
  @return Dictionary format: {'accel':{'x':0, 'y':0, 'z':0}, 'gyro':{'x':0, 'y':0, 'z':0}, 'temp':0.0}
'''
def get_sensor_data(self):

'''
  @brief Get X axis acceleration, unit g.
  @return  X axis acceleration.
'''
def get_accel_x(self):
    
'''
   @brief Get Y axis acceleration, unit g.
   @return  y axis acceleration.
'''
def get_accel_y(self):
    
'''
  @brief Get Z axis acceleration, unit g.
  @return  Z axis acceleration.
'''
def get_accel_z(self):
    
'''
  @brief Get temperature data, uint: ℃.
  @return  Temperature data.
'''
def get_temperature_c(self):
    
'''
   @brief Get X-axis gyroscope speed, unit dps.
   @return  X-axis gyroscope speed.
'''    
def get_gyro_x(self):
    
'''
  @brief Get Y-axis gyroscope speed, unit dps.
  @return  Y-axis gyroscope speed.
'''
def get_gyro_y(self):
    
'''
  @brief Get z-axis gyroscope speed, unit dps.
  @return  z-axis gyroscope speed.
'''
def get_gyro_z(self):
    
'''
  @brief Set the level polarity of the INT pin when the accelerometer sensor is triggered to wake up the motion interrupt.
  @param polarity:  the level signal of the sensor INT pin when the wake-up motion is triggered
  @n     GPIO.HIGH:  The initial signal of the pin is LOW. When an accelerometer wake-up motion occurs, the level signal of the INT pin will change to HIGH. Then the readINTStatus function needs to be called to clear the signal and restore the initial signal.
  @n     GPIO.LOW:   The initial signal of the pin is HIGH. When an accelerometer wake-up motion occurs, the level signal of the INT pin will change to LOW. Then the readINTStatus function needs to be called to clear the signal and restore the initial signal.
  @n Note:  After triggering the accelerometer wake-up motion, if the read_int_status function is not called to clear the sign, the INT pin will always maintain the level polarity when the motion is triggered.
'''
def set_int_pin_motion_trigger_polarity(self, polarity):

'''
  @brief @brief Get the polarity of the INT pin of sensor when the sensor INT pin triggers an interrupt.
  @return The level signal when the INT pin triggers an interrupt.
  @n      GPIO.HIGH:  INT pin level held  HIGH LEVEL until interrupt status is cleared.
  @n      GPIO.LOW:   INT pin level held  LOW LEVEL until interrupt status is cleared.
'''
def get_int_pin_motion_trigger_polarity(self):
   
'''
  @brief Set the threshold value for the Wake on Motion Interrupt for accelerometer. 
  @param level: WoM thresholds are expressed in fixed “mg” independent of the selected Range [0g : 1g]; Resolution 1g/256=~3.9mg
  @n     level = 0~255
  @return Actul WoM thresholds, unit : g   re_value = (level * 3.9)/1000 g
'''
def set_wake_on_motion_thread_for_accel(self, level):

'''
  @brief Read interrupt status register, and clear INT pin's interrupt signal. 
  @return Interrupt status register value.
  @n  INT_STATUS register：addr:0x3A,acess:rw
  @n  ------------------------------------------------------------------------------------
  @n  |     b7    |    b6     |    b5     |      b4        | b3 | b2 | b1 |      b0      |
  @n  ------------------------------------------------------------------------------------
  @n  |             WOM_XYZ_INT           | FIFO_OFLOW_INT |      rsv     | DATA_RDY_INT |
  @n  ------------------------------------------------------------------------------------
  @n  DATA_RDY_INT  : This bit automatically sets to 1 when a Data Ready interrupt is generated. The bit clears to 0 after the register has been read.
  @n  rsv           : reserve
  @n  FIFO_OFLOW_INT: This bit automatically sets to 1 when a FIFO buffer overflow has been generated. The bit clears to 0 after the register has been read.
  @n  WOM_XYZ_INT   : These bits automatically set to a non-zero number when the X-axis,Y-axis or Z-axis of accelerometer which trigger WOM(wake on motion) 
  @n                  interrupt.Cleared on Read.
'''
def read_int_status(self):


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





