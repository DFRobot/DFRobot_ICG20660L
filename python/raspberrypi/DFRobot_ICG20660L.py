# -*- coding:utf-8 -*-

'''
  @file DFRobot_ICG20660L.py
  @brief The ICG-20660 is a 6-axis MotionTracking device that combines a 3-axis gyroscope, 3-axis accelerometer.
  @n It support two communication interface:
  @n (1) IIC-->freq: 100~400Khz
  @n (2) SPI-->freq: 100kHz~7MHz, only support mode0 or mode3
  @n Two communication methods are switched by cs pin, 0:SPI, 1:IIC
  @n 3-axis accelerometer feature:
  @n (1) Support max ranging: ±2g、±4g、±8g、±16g, g = 9.80665 m/s²
  @n (2) 1g = 9.80665 m/s²
  @n 3-axis gyroscope feature:
  @n (1) Support max ranging: ±125dps、±250dps、±500dps
  @n (2) 1dps = Π/180° rad/s, Π = 3.1415926
  @n Motion threshold wake-up detection：
  @n The motion threshold is the acceleration thresholds difference between the previous and next. If it is greater than or equal to the set threshold, an interrupt will be generated.
  @n Support to read from register and FIFO：
  @n Read from FIFO. Accelerometer, gyroscope and temperature must all be enabled, and its internal sampling rate must be configured to be consistent.
  @
  @n Hardware conneted table in SPI
  @n -------------------------------------------------------------------
  @n  Sensor      |                      raspberry pi                  |
  @n -------------------------------------------------------------------
  @n FSY          | not connected, floating                            |
  @n INT          | connected to the external interrupt IO pin of MCU  |
  @n CS           | connected to the IO pin of raspberry pi            |
  @n SDO          | connected to miso of raspberry pi'spi              |
  @n SDI          | connected to mosi of raspberry pi'spi              |
  @n SCK          | connected to sck of raspberry pi'spi               |
  @n GND          | GND                                                |
  @n 3V3/VCC      | 3V3/VCC                                            |
  @n ------------------------------------------------------------------
  @n Hardware conneted table in IIC
  @n ------------------------------------------------------------------
  @n  Sensor      |                   raspberry pi                    |
  @n ------------------------------------------------------------------
  @n TSET         | not connected, floating                           |
  @n INT          | connected to the external interrupt IO pin of MCU |
  @n SDA          | connected to sda of raspberry pi's IIC            |
  @n SCL          | connected to scl of raspberry pi's IIC            |
  @n GND          | GND                                               |
  @n 3V3/VCC      | 3V3/VCC                                           |
  @n ------------------------------------------------------------------
  @
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @licence     The MIT License (MIT)
  @author      [Arya](xue.peng@dfrobot.com)
  @version  V1.0
  @date  2021-06-04
  @get from https://www.dfrobot.com
  @url https://github.com/DFRobot/DFRobot_ICG20660L
'''
import sys
import smbus
import time
import RPi.GPIO as GPIO
import spidev

IIC_ADDR_SDO_H = 0x69
IIC_ADDR_SDO_L = 0x68
ICG20660L_DEVICE_ID = 0x91
CLOCK_SEL_PLL = 1
ADC_MAX_RANGE = 32767.0
GYRO_FULL_SCALE_125DPS = 125.0
GYRO_FULL_SCALE_250DPS = 250.0
GYRO_FULL_SCALE_500DPS = 500.0
ACCEL_FULL_SCALE_2G = 2
ACCEL_FULL_SCALE_4G = 4
ACCEL_FULL_SCALE_8G = 8
ACCEL_FULL_SCALE_16G = 16

#Can be accessed in sleep mode
REG_ICG20660L_SMPLRT_DIV   = 0x19
REG_ICG20660L_ACCEL_CONFIG2= 0x1D
'''
* ---------------------------------------------------------
* |  b7 |  b6 | b5 | b4  |        b3       | b2 | b1 | b0 |
* ---------------------------------------------------------
* | FIFO_SIZE | DEC2_CFG | ACCEL_FCHOICE_B |  A_DLPF_CFG  |
* ---------------------------------------------------------
'''
BIT_FIFO_SIZE = 6
OFFSET_FIFO_SIZE = 0x03
BIT_DEC2_CFG = 4
OFFSET_DEC2_CFG = 0x03
BIT_ACCEL_FCHOICE_B = 3
OFFSET_ACCEL_FCHOICE_B = 0x01
BIT_A_DLPF_CFG = 0
OFFSET_A_DLPF_CFG = 0x07


REG_ICG20660L_INT_PIN_CFG  = 0x37
REG_ICG20660L_INT_ENABLE   = 0x38
'''
INT_ENABLE register：addr:0x38,acess:rw
* -----------------------------------------------------------------
* | b7 | b6 | b5 |      b4       | b3 | b2 | b1 |       b0        |
* -----------------------------------------------------------------
* |    WOM_EN    | FIFO_OFLOW_EN |      rsv     | DATA_RDY_INT_EN |
* -----------------------------------------------------------------
'''
BIT_WOM_EN = 5
OFFSET_WOM_EN = 0x07
BIT_FIFO_OFLOW_EN = 4
OFFSET_FIFO_OFLOW_EN = 0x01
BIT_DATA_RDY_INT_EN = 0
OFFSET_DATA_RDY_INT_EN = 0x01


REG_ICG20660L_PWR_MGMT_1   = 0x6B
'''
PWR_MGMT_1 register description：addr:0x6B,acess:rw
* -------------------------------------------------------------------------
* |      b7      |  b6   |   b5  |      b4      |    b3    | b2 | b1 | b0 |
* -------------------------------------------------------------------------
* | DEVICE_RESET | SLEEP | CYCLE | GYRO_STANDBY | TEMP_DIS |  CLKSEL[2:0] |
* -------------------------------------------------------------------------
'''
BIT_DEVICE_RESET = 7
OFFSET_DEVICE_RESET = 0x01
BIT_SLEEP = 6
OFFSET_SLEEP = 0x01
BIT_CYCLE = 5
OFFSET_CYCLE = 0x01
BIT_GYRO_STANDBY = 4
OFFSET_GYRO_STANDBY = 0x01
BIT_TEMP_DIS = 3
OFFSET_TEMP_DIS = 0x01
BIT_CLKSEL = 0
OFFSET_CLKSEL = 0x07

REG_ICG20660L_PWR_MGMT_2   = 0x6C
'''
PWR_MGMT_2 register：addr:0x6C,acess:rw
* -------------------------------------------------------------------------------
* |     b7     |  b6 |    b5   |    b4   |    b3   |    b2   |    b1   |   b0   |
* -------------------------------------------------------------------------------
* | FIFO_LP_EN | rsv | STBY_XA | STBY_YA | STBY_ZA | STBY_XG | STBY_YG | STBY_ZG|
* -------------------------------------------------------------------------------
'''
BIT_FIFO_LP_EN    = 7
OFFSET_FIFO_LP_EN = 0x01
BIT_STBY_XA    = 5
OFFSET_STBY_XA = 0x01
BIT_STBY_YA    = 4
OFFSET_STBY_YA = 0x01
BIT_STBY_ZA    = 3
OFFSET_STBY_ZA = 0x01
BIT_STBY_XG    = 2
OFFSET_STBY_XG = 0x01
BIT_STBY_YG    = 1
OFFSET_STBY_YG = 0x01
BIT_STBY_ZG    = 0
OFFSET_STBY_ZG = 0x01

REG_ICG20660L_FIFO_COUNTH  = 0x72
REG_ICG20660L_FIFO_COUNTL  = 0x73
REG_ICG20660L_FIFO_R_W     = 0x74

#Can't be accessed in sleep mode
REG_ICG20660L_SELF_TEST_X_GYRO   = 0x00
REG_ICG20660L_SELF_TEST_Y_GYRO   = 0x01
REG_ICG20660L_SELF_TEST_Z_GYRO   = 0x02
REG_ICG20660L_XG_OFFS_TC_H       = 0x04
REG_ICG20660L_XG_OFFS_TC_L       = 0x05
REG_ICG20660L_YG_OFFS_TC_H       = 0x07
REG_ICG20660L_YG_OFFS_TC_L       = 0x08
REG_ICG20660L_ZG_OFFS_TC_H       = 0x0A
REG_ICG20660L_ZG_OFFS_TC_L       = 0x0B
REG_ICG20660L_SELF_TEST_X_ACCEL  = 0x0D
REG_ICG20660L_SELF_TEST_Y_ACCEL  = 0x0E
REG_ICG20660L_SELF_TEST_Z_ACCEL  = 0x0F
REG_ICG20660L_XG_OFFS_USRH       = 0x13
REG_ICG20660L_XG_OFFS_USRL       = 0x14
REG_ICG20660L_YG_OFFS_USRH       = 0x15
REG_ICG20660L_YG_OFFS_USRL       = 0x16
REG_ICG20660L_ZG_OFFS_USRH       = 0x17
REG_ICG20660L_ZG_OFFS_USRL       = 0x18
REG_ICG20660L_CONFIG             = 0x1A
'''
Config register：addr:0x1A,acess:rw
* -------------------------------------------------
* |  b7 |     b6    | b5 | b4 | b3 | b2 | b1 | b0 |
* -------------------------------------------------
* | rsv | FIFO_MODE | EXT_SYNC_SET |   DLPF_CFG   |
* -------------------------------------------------
'''
BIT_FIFO_MODE  = 6
OFFSET_FIFO_MODE = 0x01
BIT_EXT_SYNC_SET  = 3
OFFSET_EXT_SYNC_SET = 0x07
BIT_DLPF_CFG  = 0
OFFSET_DLPF_CFG = 0x07

REG_ICG20660L_GYRO_CONFIG        = 0x1B
'''
GYRO_CONFIG register：addr:0x1B,acess:rw
* -----------------------------------------------------
* |   b7  |  b6   |   b5  | b4 | b3 |  b2 |  b1 |  b0 |
* -----------------------------------------------------
* | XG_ST | YG_ST | ZG_ST |  FS_SEL | rsv | FCHOICE_B |
* -----------------------------------------------------
'''
BIT_XG_ST = 7
OFFSET_XG_ST = 0x01
BIT_YG_ST = 6
OFFSET_YG_ST = 0x01
BIT_ZG_ST = 5
OFFSET_ZG_ST = 0x01
BIT_FS_SEL = 3
OFFSET_FS_SEL = 0x03
BIT_FCHOICE_B = 0
OFFSET_FCHOICE_B = 0x03

REG_ICG20660L_ACCEL_CONFIG       = 0x1C
'''
ACCEL_CONFIG：addr:0x1C,acess:rw
* -------------------------------------------------------
* |   b7  |  b6   |   b5  |  b4  |  b3   | b2 | b1 | b0 |
* -------------------------------------------------------
* | XA_ST | YA_ST | ZA_ST | ACCEL_FS_SEL |     rsv      |
* -------------------------------------------------------
'''
BIT_XA_ST = 7
OFFSET_XA_ST = 0x01
BIT_YA_ST = 6
OFFSET_YA_ST = 0x01
BIT_ZA_ST = 5
OFFSET_ZA_ST = 0x01
BIT_ACCEL_FS_SEL = 3
OFFSET_ACCEL_FS_SEL = 0x03

REG_ICG20660L_ACCEL_CONFIG2      = 0x1D
REG_ICG20660L_LP_MODE_CFG        = 0x1E
REG_ICG20660L_ACCEL_WOM_THR      = 0x1F
REG_ICG20660L_FIFO_EN            = 0x23
'''
FIFO_EN register：addr:0x23,acess:rw
* --------------------------------------------------------------------------------------
* |      b7      |     b6     |     b5     |     b4     |      b3       | b2 | b1 | b0 |
* --------------------------------------------------------------------------------------
* | TEMP_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN | ACCEL_FIFO_EN |      rsv     |
* --------------------------------------------------------------------------------------
'''
BIT_TEMP_FIFO_EN = 7
OFFSET_TEMP_FIFO_EN = 0x01
BIT_XG_FIFO_EN = 6
OFFSET_XG_FIFO_EN = 0x01
BIT_YG_FIFO_EN = 5
OFFSET_YG_FIFO_EN = 0x01
BIT_ZG_FIFO_EN = 4
OFFSET_ZG_FIFO_EN = 0x01
BIT_ACCEL_FIFO_EN = 3
OFFSET_ACCEL_FIFO_EN = 0x01

REG_ICG20660L_FSYNC_INT          = 0x36
REG_ICG20660L_INT_PIN_CFG        = 0x37
'''
INT_PIN_CFG register：addr:0x37,acess:rw
* ------------------------------------------------------------------------------------------------------
* |     b7    |    b6    |       b5     |      b4      |        b3       |         b2        | b1 | b0 |
* ------------------------------------------------------------------------------------------------------
* | INT_LEVEL | INT_OPEN | LATCH_INT_EN | INT_RD_CLEAR | FSYNC_INT_LEVEL | FSYNC_INT_MODE_EN |   rsv   |
* ------------------------------------------------------------------------------------------------------
'''
BIT_INT_LEVEL = 7
OFFSET_INT_LEVEL = 0x01
BIT_INT_OPEN = 6
OFFSET_INT_OPEN = 0x01
BIT_LATCH_INT_EN = 5
OFFSET_LATCH_INT_EN = 0x01
BIT_INT_RD_CLEAR = 4
OFFSET_INT_RD_CLEAR = 0x01
BIT_FSYNC_INT_LEVEL = 3
OFFSET_FSYNC_INT_LEVEL = 0x01
BIT_FSYNC_INT_MODE_EN = 2
OFFSET_FSYNC_INT_MODE_EN = 0x01

REG_ICG20660L_INT_STATUS         = 0x3A
REG_ICG20660L_ACCEL_XOUT_H       = 0x3B
REG_ICG20660L_ACCEL_XOUT_L       = 0x3C
REG_ICG20660L_ACCEL_YOUT_H       = 0x3D
REG_ICG20660L_ACCEL_YOUT_L       = 0x3E
REG_ICG20660L_ACCEL_ZOUT_H       = 0x3F
REG_ICG20660L_ACCEL_ZOUT_L       = 0x40
REG_ICG20660L_TEMP_OUT_H         = 0x41
REG_ICG20660L_TEMP_OUT_L         = 0x42
REG_ICG20660L_GYRO_XOUT_H        = 0x43
REG_ICG20660L_GYRO_XOUT_L        = 0x44
REG_ICG20660L_GYRO_YOUT_H        = 0x45
REG_ICG20660L_GYRO_YOUT_L        = 0x46
REG_ICG20660L_GYRO_ZOUT_H        = 0x47
REG_ICG20660L_GYRO_ZOUT_L        = 0x48
REG_ICG20660L_SIGNAL_PATH_RESET  = 0x68
REG_ICG20660L_ACCEL_INTEL_CTRL   = 0x69
'''
ACCEL_INTEL_CTRL register：addr:0x69,acess:rw
* ----------------------------------------------------------------------------
* |       b7       |        b6        | b5 | b4 | b3 | b2 | b1 |      b0     |
* ----------------------------------------------------------------------------
* | ACCEL_INTEL_EN | ACCEL_INTEL_MODE |           rsv          | WOM_TH_MODE |
* ----------------------------------------------------------------------------
'''
BIT_ACCEL_INTEL_EN = 7
OFFSET_ACCEL_INTEL_EN = 0x01
BIT_ACCEL_INTEL_MODE = 6
OFFSET_ACCEL_INTEL_MODE = 0x01
BIT_WOM_TH_MODE = 0
OFFSET_WOM_TH_MODE = 0x01

REG_ICG20660L_USER_CTRL          = 0x6A
'''
USER_CTRL register：addr:0x6A,acess:rw
* -------------------------------------------------------------------------
* | b7  |    b6   |  b5 |     b4     | b3  |    b2    | b1  |      b0     |
* -------------------------------------------------------------------------
* | rsv | FIFO_EN | rsv | I2C_IF_DIS | rsv | FIFO_RST | rsv | SIG_COND_RST|
* -------------------------------------------------------------------------
'''
BIT_FIFO_EN    = 6
OFFSET_FIFO_EN = 0x01
BIT_I2C_IF_DIS    = 4
OFFSET_I2C_IF_DIS = 0x01
BIT_FIFO_RST    = 2
OFFSET_FIFO_RST = 0x01
BIT_SIG_COND_RST    = 0
OFFSET_SIG_COND_RST = 0x01

REG_ICG20660L_WHO_AM_I           = 0x75
REG_ICG20660L_XA_OFFSET_H        = 0x77
REG_ICG20660L_XA_OFFSET_L        = 0x78
REG_ICG20660L_YA_OFFSET_H        = 0x7A
REG_ICG20660L_YA_OFFSET_L        = 0x7B
REG_ICG20660L_ZA_OFFSET_H        = 0x7D
REG_ICG20660L_ZA_OFFSET_L        = 0x7E

RAW_DATA_AX_H_INDEX  =  0x00
RAW_DATA_AX_L_INDEX  =  0x01
RAW_DATA_AY_H_INDEX  =  0x02
RAW_DATA_AY_L_INDEX  =  0x03
RAW_DATA_AZ_H_INDEX  =  0x04
RAW_DATA_AZ_L_INDEX  =  0x05
RAW_DATA_T_H_INDEX   =  0x06
RAW_DATA_T_L_INDEX   =  0x07
RAW_DATA_GX_H_INDEX  =  0x08
RAW_DATA_GX_L_INDEX  =  0x09
RAW_DATA_GY_H_INDEX  =  0x0A
RAW_DATA_GY_L_INDEX  =  0x0B
RAW_DATA_GZ_H_INDEX  =  0x0C
RAW_DATA_GZ_L_INDEX  =  0x0D
RAW_DATA_LENGTH      =  14

ICG20660L_WOM_XYZ_INT = 7 << 5

BIT_FIFO_LOW_EN = 7
OFFSET_FIFO_LOW_EN = 0x01
BIT_TEMP = 6
OFFSET_TEMP = 0x01
BIT_ACCEL_X = 5
OFFSET_ACCEL_X = 0x01
BIT_ACCEL_Y = 4
OFFSET_ACCEL_Y = 0x01
BIT_ACCEL_Z = 3
OFFSET_ACCEL_Z = 0x01
BIT_GYRO_X = 2
OFFSET_GYRO_X = 0x01
BIT_GYRO_Y = 1
OFFSET_GYRO_Y = 0x01
BIT_GYRO_Z = 0
OFFSET_GYRO_Z = 0x01



class DFRobot_ICG20660L:
  _data_mode = 0
  _mode = 2
  _level = GPIO.LOW
  _fifo_frame_size = 0
  _accel_scale = ADC_MAX_RANGE/ACCEL_FULL_SCALE_16G
  _accel_range = ACCEL_FULL_SCALE_16G
  _gyro_scale = ADC_MAX_RANGE/GYRO_FULL_SCALE_500DPS
  _gyro_range = GYRO_FULL_SCALE_500DPS
  _raw_data = [0]*14
  _update = 0

  '''Enum power mode'''
  eSLEEP_MODE = 0              #gyro: off, accel:off, low power consumption about 70uA
  eACCEL_LOW_POWER_MODE = 1    #gyro: off, accel:duty_cycled, consumption about 1.5mA
  eSIX_AXIS_LOW_NOISE_MODE = 2 #gyro: on, accel:on, consumption about 3.3mA
  

  '''Enum data read mode'''
  eREG_MODE = 0      #Read sensor data from data register.
  eFIFO_MODE = 1     #Read sensor data from 512 bytes FIFO.
  
  '''Enum gyro full scale range'''
  eFSR_G_125DPS = 0  #The full scale range of gyro: 0~±125dps, 1dps = Π/180° rad/s, Π = 3.1415926.
  eFSR_G_250DPS = 1  #The full scale range of gyro: 0~±250dps, 1dps = Π/180° rad/s, Π = 3.1415926.
  eFSR_G_500DPS = 2  #The full scale range of gyro: 0~±500dps, 1dps = Π/180° rad/s, Π = 3.1415926.
  
  '''Enum accelerometer full scale range'''
  eFSR_A_2G = 0   #The full scale range of accel: 0~±2g, 1g = 9.80665 m/s².
  eFSR_A_4G = 1   #The full scale range of accel: 0~±4g, 1g = 9.80665 m/s².
  eFSR_A_8G = 2   #The full scale range of accel: 0~±8g, 1g = 9.80665 m/s².
  eFSR_A_16G = 3  #The full scale range of accel: 0~±16g, 1g = 9.80665 m/s².
  
  '''Enum sensor enable bit'''
  eGYRO_AXIS_Z = 1 << 6 | 1 << 0        #The bit is gyro's z axis, you can call enableSensor or disableSensor function to enable or disable the gyro's Z axis.
  eGYRO_AXIS_Y = 1 << 6 | 1 << 1        #The bit is gyro's y axis, you can call enableSensor or disableSensor function to enable or disable the gyro's Y axis.
  eGYRO_AXIS_X = 1 << 6 | 1 << 2        #The bit is gyro's z axis, you can call enableSensor or disableSensor function to enable or disable the gyro's X axis.
  eACCEL_AXIS_Z = 3                     #The bit is accel's z axis, you can call enableSensor or disableSensor function to enable or disable the accel's Z axis.
  eACCEL_AXIS_Y = 4                     #The bit is accel's Y axis, you can call enableSensor or disableSensor function to enable or disable the accel's Y axis.
  eACCEL_AXIS_X = 5                     #The bit is accel's X axis, you can call enableSensor or disableSensor function to enable or disable the accel's X axis.
  eGYRO_AXIS_XYZ = 1 << 6 | 0X07        #The bits of gyro's xyz axis, you can call enableSensor or disableSensor function to enable or disable the gyro's xyz axis.
  eACCEL_AXIS_XYZ = 0X07 << 3           #The bits of accel's xyz axis, you can call enableSensor or disableSensor function to enable or disable the accel's xyz axis.
  eAXIS_ALL = 1 << 6 | 0X3F             #The gryo's and accel's xyz axis, you can call enableSensor or disableSensor function to enable or disable the accel's and gyro's xyz axis.
  eTEMPERATURE = 1 << 6                 #The bits of six-axis temperature, you can call enableSensor or disableSensor function to enable or disable the temperature.
  eFIFO_LOW_POWER_EN = 1<< 7            #The bits of fifo low power enable bit.
  
  '''Enum gyro bandwidth'''
  eGYRO_DLPF_8173_32KHZ = 0  #When the signal is equal to or greater than 8173Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 32KHz.
  eGYRO_DLPF_3281_32KHZ = 1  #When the signal is equal to or greater than 3281Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 32KHz.
  eGYRO_DLPF_250_8KHZ = 2    #When the signal is equal to or greater than 250Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 8KHz.
  eGYRO_DLPF_176_1KHZ = 3    #When the signal is equal to or greater than 176Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz
  eGYRO_DLPF_92_1KHZ  = 4    #When the signal is equal to or greater than 92Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  eGYRO_DLPF_3281_8KHZ = 5   #When the signal is equal to or greater than 3281Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 8KHz.
  
  '''Enum accelerometer bandwidth'''
  eACCEL_DLPF_5_1KHZ = 0     #When the signal is less than or equal to 5Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  eACCEL_DLPF_10_1KHZ = 1    #When the signal is less than or equal to 10Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  eACCEL_DLPF_21_1KHZ = 2    #When the signal is less than or equal to 21Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  eACCEL_DLPF_44_1KHZ = 3    #When the signal is less than or equal to 44Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  eACCEL_DLPF_99_1KHZ = 4    #When the signal is less than or equal to 99Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  eACCEL_DLPF_218_1KHZ = 5   #This configuration also supports low power consumption. When the signal is less than or equal to 218Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  eACCEL_DLPF_420_1KHZ = 6   #This configuration also supports low power consumption. When the signal is less than or equal to 420Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  eACCEL_DLPF_1046_4KHZ = 7  #This configuration also supports low power consumption. When the signal is less than or equal to 1046Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  eACCEL_DLPF_55_1KHZ = 8    #This configuration only supports low power consumption. When the signal is less than or equal to 55Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  eACCEL_DLPF_110_1KHZ = 9   #This configuration only supports low power consumption. When the signal is less than or equal to 110Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
  
  '''Enum accelerometer output data rate'''
  eODR_0_24HZ = 0    #the low power accel Output Data Rate: 0.24Hz
  eODR_0_49HZ = 1    #the low power accel Output Data Rate: 0.49Hz
  eODR_0_98HZ = 2    #the low power accel Output Data Rate: 0.98Hz
  eODR_1_95HZ = 3    #the low power accel Output Data Rate: 1.95Hz
  eODR_3_91HZ = 4    #the low power accel Output Data Rate: 3.91Hz
  eODR_7_81HZ = 5    #the low power accel Output Data Rate: 7.81Hz
  eODR_15_63HZ = 6   #the low power accel Output Data Rate: 15.63Hz
  eODR_31_25HZ = 7   #the low power accel Output Data Rate: 31.25Hz
  eODR_62_50HZ = 8   #the low power accel Output Data Rate: 62.50Hz
  eODR_125HZ = 9     #the low power accel Output Data Rate: 125Hz
  eODR_250HZ = 10    #the low power accel Output Data Rate: 250Hz
  eODR_500HZ = 11    #<the low power accel Output Data Rate: 500Hz
  
  

  def __init__(self):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

  def begin(self, mode = 0):
    '''
      @brief Initialize the sensor. After initialization, all sensors are turned off, and the corresponding configuration needs to be turned on through enable_sensor.
      @param mode:  Does configuration read sensor data from FIFO or register?
      @n     eREG_MODE :   Read sensor data from data register.
      @n     eFIFO_MODE:   Read sensor data from 512 bytes FIFO. Note:Read from FIFO, accelerometer, gyroscope, and temperature must all be enabled,
      @n and the internal sampling rate must be configured to be consistent. 
      @return status:
      @n      0 : Initialization sucess.
      @n      -1: Interface Initialization failed(IIC or SPI).
      @n      -2:  Failed to read the device ID, the ID is not 0x91
    '''
    self._data_mode = mode
    self._mode = self.eSLEEP_MODE
    self.reset()
    time.sleep(1)
    
    self.wakeup()
    self._mode = self.eSIX_AXIS_LOW_NOISE_MODE
    time.sleep(1)
    self._select_clock_source(CLOCK_SEL_PLL)
    if self.read_id() != ICG20660L_DEVICE_ID:
      return -2
    self.disable_sensor(self.eAXIS_ALL)
    self.config_accel(self.eFSR_A_16G, self.eACCEL_DLPF_218_1KHZ, self.eODR_0_24HZ, False)
    self.config_gyro(self.eFSR_G_500DPS, self.eGYRO_DLPF_176_1KHZ)
    return 0
    
  def read_id(self):
    '''
      @brief Get device ID, ICG20660L is 0x91 (145).
      @return  If device is ICG20660L, it will return 0x91.
    '''
    rslt = self._read_bytes(REG_ICG20660L_WHO_AM_I, 1)
    #print("id+++++=%0#x"%rslt[0])
    return rslt[0]

  def reset(self):
    '''
      @brief Reset, the register will restore the initial value, you need to call begin to configuration.
    '''
    wait_for_timeout_ms = 0.1
    wait_for_timout_inc = 0.005
    t = 0
    rslt = self._read_bytes(REG_ICG20660L_PWR_MGMT_1, 1)
    if len(rslt) == 1:
      rslt = self._update_reg_bit_value(rslt[0], BIT_DEVICE_RESET, OFFSET_DEVICE_RESET, 1)
    else:
      return None
    self._write_bytes(REG_ICG20660L_PWR_MGMT_1, [rslt])
    while True:
      time.sleep(wait_for_timout_inc)
      t += wait_for_timout_inc
      if t >= wait_for_timeout_ms:
        break
      rslt = self._read_bytes(REG_ICG20660L_PWR_MGMT_1, 1)
      if len(rslt) == 1:
        if self._get_reg_bit_value(rslt[0], BIT_DEVICE_RESET, OFFSET_DEVICE_RESET) == 0:
          break
      
      
  def sleep(self):
    '''
      @brief Enter sleep mode, it will reduce power consumption, and The gyroscope and acceleration will stop working. 
      @n You need to call wakeup function to wake up sensor.
    '''
    rslt = self._read_bytes(REG_ICG20660L_PWR_MGMT_1, 1)
    if len(rslt) == 1:
      rslt = self._update_reg_bit_value(rslt[0], BIT_SLEEP, OFFSET_SLEEP, 1)
    else:
      return None
    self._write_bytes(REG_ICG20660L_PWR_MGMT_1, [rslt])
    time.sleep(0.1)

  def wakeup(self):
    '''
      @brief Waking up sensor from sleep, and you will restore the configuration before sleep.
    '''
    rslt = self._read_bytes(REG_ICG20660L_PWR_MGMT_1, 1)
    #print("rslt=%#x"%rslt[0])
    if len(rslt) == 1:
      rslt = self._update_reg_bit_value(rslt[0], BIT_SLEEP, OFFSET_SLEEP, 0)
      #print("rslt=%#x"%rslt)
    else:
      return None
    
    self._write_bytes(REG_ICG20660L_PWR_MGMT_1, [rslt])
    rslt = self._read_bytes(REG_ICG20660L_PWR_MGMT_1, 1)
    #print("rslt=%#x"%rslt[0])
    time.sleep(1)

  def enable_sensor(self, bit):
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
    rslt = self._read_bytes(REG_ICG20660L_PWR_MGMT_1, 2)
    #print("REG_ICG20660L_PWR_MGMT_1=%#X"%rslt[0])
    #print("REG_ICG20660L_PWR_MGMT_2=%#X"%rslt[1])
    if len(rslt) == 2:
      if self._get_reg_bit_value(bit, BIT_FIFO_LOW_EN, OFFSET_FIFO_LOW_EN):
        rslt[1] = self._update_reg_bit_value(rslt[1], BIT_FIFO_LP_EN, OFFSET_FIFO_LP_EN, 1)

      if self._get_reg_bit_value(bit, BIT_TEMP, OFFSET_TEMP):
        rslt[0] = self._update_reg_bit_value(rslt[0], BIT_TEMP_DIS, OFFSET_TEMP_DIS, 0)

      if self._get_reg_bit_value(bit, BIT_ACCEL_X, OFFSET_ACCEL_X):
        rslt[1] = self._update_reg_bit_value(rslt[1], BIT_STBY_XA, OFFSET_STBY_XA, 0)

      if self._get_reg_bit_value(bit, BIT_ACCEL_Y, OFFSET_ACCEL_Y):
        rslt[1] = self._update_reg_bit_value(rslt[1], BIT_STBY_YA, OFFSET_STBY_YA, 0)

      if self._get_reg_bit_value(bit, BIT_ACCEL_Z, OFFSET_ACCEL_Z):
        rslt[1] = self._update_reg_bit_value(rslt[1], BIT_STBY_ZA, OFFSET_STBY_ZA, 0)

      if self._get_reg_bit_value(bit, BIT_GYRO_X, OFFSET_GYRO_X):
        rslt[1] = self._update_reg_bit_value(rslt[1], BIT_STBY_XG, OFFSET_STBY_XG, 0)

      if self._get_reg_bit_value(bit, BIT_GYRO_Y, OFFSET_GYRO_Y):
        rslt[1] = self._update_reg_bit_value(rslt[1], BIT_STBY_YG, OFFSET_STBY_YG, 0)

      if self._get_reg_bit_value(bit, BIT_GYRO_Z, OFFSET_GYRO_Z):
        rslt[1] = self._update_reg_bit_value(rslt[1], BIT_STBY_ZG, OFFSET_STBY_ZG, 0)
      
      if self._data_mode == self.eFIFO_MODE:
        self._enable_fifo(True, True, True, True, True)
      else:
        self._enable_fifo(False, False, False, False, False)
      self._write_bytes(REG_ICG20660L_PWR_MGMT_1, rslt)
      rslt = self._read_bytes(REG_ICG20660L_PWR_MGMT_1, 2)
      #print("REG_ICG20660L_PWR_MGMT_2=%#X"%rslt[1])
      #print("REG_ICG20660L_PWR_MGMT_1=%#X"%rslt[0])

  def disable_sensor(self, bit):
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
      @n Note: Only when the X, Y, and Z axes of the gyroscope are all closed, the temperature sensor will be turned off. Any axis’s turning on will make the temperature sensor not be turned off.。
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
    rslt = self._read_bytes(REG_ICG20660L_PWR_MGMT_1, 2)
    if self._get_reg_bit_value(bit, BIT_FIFO_LOW_EN, OFFSET_FIFO_LOW_EN):
      rslt[1] = self._update_reg_bit_value(rslt[1], BIT_FIFO_LP_EN, OFFSET_FIFO_LP_EN, 0)

    if self._get_reg_bit_value(bit, BIT_TEMP, OFFSET_TEMP):
      rslt[0] = self._update_reg_bit_value(rslt[0], BIT_TEMP_DIS, OFFSET_TEMP_DIS, 1)

    if self._get_reg_bit_value(bit, BIT_ACCEL_X, OFFSET_ACCEL_X):
      rslt[1] = self._update_reg_bit_value(rslt[1], BIT_STBY_XA, OFFSET_STBY_XA, 1)

    if self._get_reg_bit_value(bit, BIT_ACCEL_Y, OFFSET_ACCEL_Y):
      rslt[1] = self._update_reg_bit_value(rslt[1], BIT_STBY_YA, OFFSET_STBY_YA, 1)

    if self._get_reg_bit_value(bit, BIT_ACCEL_Z, OFFSET_ACCEL_Z):
      rslt[1] = self._update_reg_bit_value(rslt[1], BIT_STBY_ZA, OFFSET_STBY_ZA, 1)

    if self._get_reg_bit_value(bit, BIT_GYRO_X, OFFSET_GYRO_X):
      rslt[1] = self._update_reg_bit_value(rslt[1], BIT_STBY_XG, OFFSET_STBY_XG, 1)

    if self._get_reg_bit_value(bit, BIT_GYRO_Y, OFFSET_GYRO_Y):
      rslt[1] = self._update_reg_bit_value(rslt[1], BIT_STBY_YG, OFFSET_STBY_YG, 1)

    if self._get_reg_bit_value(bit, BIT_GYRO_Z, OFFSET_GYRO_Z):
      rslt[1] = self._update_reg_bit_value(rslt[1], BIT_STBY_ZG, OFFSET_STBY_ZG, 1)
    
    if self._data_mode == self.eFIFO_MODE:
      self._enable_fifo(True, True, True, True, True)
    else:
      self._enable_fifo(False, False, False, False, False)
    if (self._get_reg_bit_value(rslt[1], BIT_GYRO_X, OFFSET_GYRO_X) == 0) or (self._get_reg_bit_value(rslt[1], BIT_GYRO_Z, OFFSET_GYRO_Z) == 0) or (self._get_reg_bit_value(rslt[1], BIT_GYRO_Y, OFFSET_GYRO_Y) == 0):
      rslt[0] = self._update_reg_bit_value(rslt[0], BIT_TEMP_DIS, OFFSET_TEMP_DIS, 0)

    self._write_bytes(REG_ICG20660L_PWR_MGMT_1, rslt)

  def config_accel(self,scale, bd, odr = 0, low_power_flag = False):
    '''
      @brief Config of accel's full scale 、dlpf bandwidth and internal sample rate. 
      @param scale  The full scale of accel, unit: g(1g = 9.80665 m/s²).
      @n     eFSR_A_2G:  The full scale range is ±2g.
      @n     eFSR_A_4G:  The full scale range is ±4g.
      @n     eFSR_A_8G:  The full scale range is ±8g.
      @n     eFSR_A_16G:  The full scale range is ±16g.
      @param bd  Set 3-db bandwidth.
      @n     eACCEL_DLPF_5_1KHZ or 0:     When the signal is less than or equal to 5Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
      @n     eACCEL_DLPF_10_1KHZ or 1:   When the signal is less than or equal to 10Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
      @n     eACCEL_DLPF_21_1KHZ or 2:   When the signal is less than or equal to 21Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
      @n     eACCEL_DLPF_44_1KHZ or 3:   When the signal is less than or equal to 44Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
      @n     eACCEL_DLPF_99_1KHZ or 4:   When the signal is less than or equal to 99Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
      @n     eACCEL_DLPF_218_1KHZ or 5:  When the signal is less than or equal to 218Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz. Support low power consumption
      @n     eACCEL_DLPF_420_1KHZ or 6:  When the signal is less than or equal to 420Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz. Support low power consumption
      @n     eACCEL_DLPF_1046_4KHZ or 7: When the signal is less than or equal to 1046Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 4KHz. Support low power consumption
      @n     eACCEL_DLPF_55_1KHZ or 8:    When the signal is less than or equal to 55Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz. Only support low power consumption
      @n     eACCEL_DLPF_110_1KHZ or 9:  When the signal is less than or equal to 110Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz. Only support low power consumption
      @n  Note: When the gyroscope and accelerometer are both enabled, if the sensor data is read through the FIFO, the internal sampling rate of the gyroscope and accelerometer must be the same.
      @param odr:  Sets the frequency of waking up the chip to take a sample of accel data – the low power accel Output Data Rate.
      @n     eODR_125HZ or 9:    The low power accel Output Data Rate: 125Hz
      @n     eODR_250HZ or 10:   The low power accel Output Data Rate: 250Hz
      @n     eODR_500HZ or 11:   The low power accel Output Data Rate: 500Hz
      @param low_power_flag:  Whether to configure the Acceleration to low power mode.
      @n     True:          Enter low power mode.
      @n     False:         Not configure the Acceleration to low power mode.(default)
    '''
    if low_power_flag:
      self._mode = self.eACCEL_LOW_POWER_MODE
    else:
      self._mode = self.eSIX_AXIS_LOW_NOISE_MODE
    self._set_full_scale_for_accel(scale)
    self._set_bandwidth_for_accel(bd)
    self._write_bytes(REG_ICG20660L_LP_MODE_CFG, [odr])

  def config_gyro(self, scale, bd):
    '''
      @brief Config of gyro's full scale 、dlpf bandwidth and internal sample rate. 
      @param scale  The full scale of gyro, unit: dps(Degrees per second).
      @n     eFSR_G_125DPS:  The full scale range is ±125 dps.
      @n     eFSR_G_250DPS:  The full scale range is ±250 dps.
      @n     eFSR_G_500DPS:  The full scale range is ±500 dps.
      @param bd  Set 3-db bandwidth.
      @n     eGYRO_DLPF_8173_32KHZ: When the signal is equal to or greater than 8173Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 32KHz.
      @n     eGYRO_DLPF_3281_32KHZ: When the signal is equal to or greater than 3281Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 32KHz.
      @n     eGYRO_DLPF_250_8KHZ:   When the signal is equal to or greater than 250Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 8KHz.
      @n     eGYRO_DLPF_176_1KHZ:   When the signal is equal to or greater than 176Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
      @n     eGYRO_DLPF_92_1KHZ:    When the signal is equal to or greater than 92Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
      @n     eGYRO_DLPF_3281_8KHZ:  When the signal is equal to or greater than 3281Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 8KHz.
      @n When the gyroscope and accelerometer are both enabled, if the sensor data is read through the FIFO, the internal sampling rate of the gyroscope and accelerometer must be the same.
    '''
    self._set_full_scale_for_gyro(scale)
    self._set_bandwidth_for_gyro(bd)


  def set_sample_div(self, div):
    '''
      @brief Set sample rate divider. 
      @param div  Sample rate divider, the range is 0~255.
      @n     Sampling rate = internal sampling rate/(div+1)
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
      @brief Get Sensor's accel, gyro and temperature data.
      @return Dictionary format: {'accel':{'x':0, 'y':0, 'z':0}, 'gyro':{'x':0, 'y':0, 'z':0}, 'temp':0.0}
    '''
    sensor = icg.get_sensor_data()
    #print(sensor)
    print("Accel:  x: %.3f g,  y: %.3f g,  z:%.3f g"%(sensor['accel']['x'], sensor['accel']['y'],sensor['accel']['z']))
    print("Gyro:  x: %.3f dps,  y: %.3f dps,  z:%.3f dps"%(sensor['gyro']['x'], sensor['gyro']['y'],sensor['gyro']['z']))
    print("Temp:   t: %.3f C"%(sensor['temp']))
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
    div &= 0xff
    self._write_bytes(REG_ICG20660L_SMPLRT_DIV, [div])

  def get_raw_data(self, length = 0):
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
    self._raw_data = [0]*RAW_DATA_LENGTH
    if self._data_mode == self.eFIFO_MODE:
      self._read_data_from_fifo()
    else:
      self._read_data_from_reg()
    self._update = 0x7F
    return self._raw_data[0:length]

  def get_sensor_data(self):
    '''
      @brief Get Sensor's accel, gyro and temperature data.
      @return Dictionary format: {'accel':{'x':0, 'y':0, 'z':0}, 'gyro':{'x':0, 'y':0, 'z':0}, 'temp':0.0}
    '''
    self.get_raw_data()
    sensor = {'accel':{'x':0.0, 'y':0.0, 'z':0.0}, 'gyro':{'x':0.0, 'y':0.0, 'z':0.0}, 'temp':0.0}
    sensor['accel']['x'] = self._cal_value(self._raw_data[RAW_DATA_AX_H_INDEX], self._raw_data[RAW_DATA_AX_L_INDEX])/self._accel_scale
    sensor['accel']['y'] = self._cal_value(self._raw_data[RAW_DATA_AY_H_INDEX], self._raw_data[RAW_DATA_AY_L_INDEX])/self._accel_scale
    sensor['accel']['z'] = self._cal_value(self._raw_data[RAW_DATA_AZ_H_INDEX], self._raw_data[RAW_DATA_AZ_L_INDEX])/self._accel_scale
    
    sensor['gyro']['x'] = self._cal_value(self._raw_data[RAW_DATA_GX_H_INDEX], self._raw_data[RAW_DATA_GX_L_INDEX])/self._gyro_scale
    sensor['gyro']['y'] = self._cal_value(self._raw_data[RAW_DATA_GY_H_INDEX], self._raw_data[RAW_DATA_GY_L_INDEX])/self._gyro_scale
    sensor['gyro']['z'] = self._cal_value(self._raw_data[RAW_DATA_GZ_H_INDEX], self._raw_data[RAW_DATA_GZ_L_INDEX])/self._gyro_scale
    
    sensor['temp'] = self._cal_value(self._raw_data[RAW_DATA_T_H_INDEX], self._raw_data[RAW_DATA_T_L_INDEX])/326.8 + 25
    self._update = 0
    return sensor

  def get_accel_x(self):
    '''
      @brief Get X axis acceleration, unit g.
      @return  X axis acceleration.
    '''
    if self._update & 0x01 == 0:
      self.get_raw_data()
    self._update &= 0xFE
    return self._cal_value(self._raw_data[RAW_DATA_AX_H_INDEX], self._raw_data[RAW_DATA_AX_L_INDEX])/self._accel_scale

  def get_accel_y(self):
    '''
       @brief Get Y axis acceleration, unit g.
       @return  y axis acceleration.
    '''
    if self._update & 0x02 == 0:
      self.get_raw_data()
    self._update &= 0xFD
    return self._cal_value(self._raw_data[RAW_DATA_AY_H_INDEX], self._raw_data[RAW_DATA_AY_L_INDEX])/self._accel_scale

  def get_accel_z(self):
    '''
      @brief Get Z axis acceleration, unit g.
      @return  Z axis acceleration.
    '''
    if self._update & 0x04 == 0:
      self.get_raw_data()
    self._update &= 0xFB
    return self._cal_value(self._raw_data[RAW_DATA_AZ_H_INDEX], self._raw_data[RAW_DATA_AZ_L_INDEX])/self._accel_scale
  
  def get_temperature_c(self):
    '''
      @brief Get temperature data, uint: ℃.
      @return  Temperature data.
    '''
    if self._update & 0x08 == 0:
      self.get_raw_data()
    self._update &= 0xF7
    return self._cal_value(self._raw_data[RAW_DATA_T_H_INDEX], self._raw_data[RAW_DATA_T_L_INDEX])/326.8 + 25
  
  def get_gyro_x(self):
    '''
       @brief Get X-axis gyroscope speed, unit dps.
       @return  X-axis gyroscope speed.
    '''
    if self._update & 0x10 == 0:
      self.get_raw_data()
    self._update &= 0xEF

    return self._cal_value(self._raw_data[RAW_DATA_GX_H_INDEX], self._raw_data[RAW_DATA_GX_L_INDEX])/self._gyro_scale

  def get_gyro_y(self):
    '''
      @brief Get Y-axis gyroscope speed, unit dps.
      @return  Y-axis gyroscope speed.
    '''
    if self._update & 0x20 == 0:
      self.get_raw_data()
    self._update &= 0xDF
    return self._cal_value(self._raw_data[RAW_DATA_GY_H_INDEX], self._raw_data[RAW_DATA_GY_L_INDEX])/self._gyro_scale

  def get_gyro_z(self):
    '''
      @brief Get z-axis gyroscope speed, unit dps.
      @return  z-axis gyroscope speed.
    '''
    if self._update & 0x40 == 0:
      self.get_raw_data()
    self._update &= 0xBF
    return self._cal_value(self._raw_data[RAW_DATA_GZ_H_INDEX], self._raw_data[RAW_DATA_GZ_L_INDEX])/self._gyro_scale

  def set_int_pin_motion_trigger_polarity(self, polarity):
    '''
      @brief Set the level polarity of the INT pin when the accelerometer sensor is triggered to wake up the motion interrupt.
      @param polarity:  the level signal of the sensor INT pin when the wake-up motion is triggered
      @n     GPIO.HIGH:  The initial signal of the pin is LOW. When an accelerometer wake-up motion occurs, the level signal of the INT pin will change to HIGH. Then the readINTStatus function needs to be called to clear the signal and restore the initial signal.
      @n     GPIO.LOW:   The initial signal of the pin is HIGH. When an accelerometer wake-up motion occurs, the level signal of the INT pin will change to LOW. Then the readINTStatus function needs to be called to clear the signal and restore the initial signal.
      @n Note:  After triggering the accelerometer wake-up motion, if the read_int_status function is not called to clear the sign, the INT pin will always maintain the level polarity when the motion is triggered.
    '''
    acl_rslt = self._read_bytes(REG_ICG20660L_ACCEL_CONFIG2, 1)
    acl_rslt = self._update_reg_bit_value(acl_rslt[0], BIT_A_DLPF_CFG, OFFSET_A_DLPF_CFG, 2)
    self._write_bytes(REG_ICG20660L_ACCEL_CONFIG2, [acl_rslt])
    self._level = polarity
    pin_rslt = self._read_bytes(REG_ICG20660L_INT_PIN_CFG, 1)
    
    if polarity == GPIO.HIGH:
      pin_rslt = self._update_reg_bit_value(pin_rslt[0], BIT_INT_LEVEL, OFFSET_INT_LEVEL, 0)
    else:
      pin_rslt = self._update_reg_bit_value(pin_rslt[0], BIT_INT_LEVEL, OFFSET_INT_LEVEL, 1)
    pin_rslt = self._update_reg_bit_value(pin_rslt, BIT_INT_OPEN, OFFSET_INT_OPEN, 0)
    pin_rslt = self._update_reg_bit_value(pin_rslt, BIT_LATCH_INT_EN, OFFSET_LATCH_INT_EN, 1)
    self._write_bytes(REG_ICG20660L_INT_PIN_CFG, [pin_rslt])
    
    irq_rslt = self._read_bytes(REG_ICG20660L_INT_ENABLE, 1)
    irq_rslt = self._update_reg_bit_value(irq_rslt[0], BIT_WOM_EN, OFFSET_WOM_EN, 7)
    irq_rslt = self._update_reg_bit_value(irq_rslt, BIT_DATA_RDY_INT_EN, OFFSET_DATA_RDY_INT_EN, 0)
    self._write_bytes(REG_ICG20660L_INT_ENABLE, [irq_rslt])
    time.sleep(1)
    
    itel_rslt = self._read_bytes(REG_ICG20660L_ACCEL_INTEL_CTRL, 1)
    itel_rslt = self._update_reg_bit_value(itel_rslt[0], BIT_ACCEL_INTEL_EN, OFFSET_ACCEL_INTEL_EN, 1)
    itel_rslt = self._update_reg_bit_value(itel_rslt, BIT_WOM_TH_MODE, OFFSET_WOM_TH_MODE, 1)
    itel_rslt = self._update_reg_bit_value(itel_rslt, BIT_ACCEL_INTEL_MODE, OFFSET_ACCEL_INTEL_MODE, 1)
    self._write_bytes(REG_ICG20660L_ACCEL_INTEL_CTRL, [itel_rslt])

  def set_wake_on_motion_thread_for_accel(self, level):
    '''
      @brief Set the threshold value for the Wake on Motion Interrupt for accelerometer. 
      @param level: WoM thresholds are expressed in fixed “mg” independent of the selected Range [0g : 1g]; Resolution 1g/256=~3.9mg
      @n     level = 0~255
      @return Actul WoM thresholds, unit : g   re_value = (level * 3.9)/1000 g
    '''
    g = (level *  3.9)/1000.0
    self._write_bytes(REG_ICG20660L_ACCEL_WOM_THR, [level])

  def get_int_pin_motion_trigger_polarity(self):
    '''
      @brief @brief Get the polarity of the INT pin of sensor when the sensor INT pin triggers an interrupt.
      @return The level signal when the INT pin triggers an interrupt.
      @n      GPIO.HIGH:  INT pin level held  HIGH LEVEL until interrupt status is cleared.
      @n      GPIO.LOW:   INT pin level held  LOW LEVEL until interrupt status is cleared.
    '''
    return self._level
  
  def read_int_status(self):
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
    rslt = self._read_bytes(REG_ICG20660L_INT_STATUS, 1)
    return rslt[0]
    
  def _cal_value(self, h, l):
    value = 0
    if h & 0x80:
      h &= 0x7F
      value = -((~((h <<8) | l)) &  0x7fff)
    else:
      value = (h <<8) | l
    return value

  def _update_reg_bit_value(self, reg_value, bit, offset, value):
    #print("reg_value = %#x, bit = %d, offset = %#x, value = %#x"%(reg_value,bit,offset, value))
    reg_value &= ((~(offset << bit)) & 0xFF)
    #print("reg_value1 = %#x"%reg_value)
    reg_value |= (value << bit)
    #print("reg_value2 = %#x"%reg_value)
    return reg_value & 0xFF

  def _get_reg_bit_value(self, reg_value, bit, offset):
    #print("reg_value = %#x, bit = %d, offset = %#x"%(reg_value,bit,offset))
    reg_value >>= bit
    #print("reg_value1 = %#x"%reg_value)
    reg_value &= offset
    #print("reg_value2 = %#x"%reg_value)
    return reg_value & 0xFF
  
  def _select_clock_source(self, clk):
    rslt = self._read_bytes(REG_ICG20660L_PWR_MGMT_1, 1)
    if len(rslt) == 1:
      rslt = self._update_reg_bit_value(rslt[0], BIT_CLKSEL, OFFSET_CLKSEL, clk)
    else:
      return None
    self._write_bytes(REG_ICG20660L_PWR_MGMT_1, [rslt])
    time.sleep(1)
  
  def _enable_fifo(self, temp, gx, gy, gz, accel):
    rslt_fifo_en = self._read_bytes(REG_ICG20660L_FIFO_EN, 1)
    rslt_usr = self._read_bytes(REG_ICG20660L_USER_CTRL, 1)
    rslt_cfg = self._read_bytes(REG_ICG20660L_CONFIG, 1)
    
    if temp or gx or gy or gz or accel:
      rslt_usr = self._update_reg_bit_value(rslt_usr[0], BIT_FIFO_EN, OFFSET_FIFO_EN, 1)
      rslt_usr = self._update_reg_bit_value(rslt_usr, BIT_FIFO_RST, OFFSET_FIFO_RST, 1)
      rslt_usr = self._update_reg_bit_value(rslt_usr, BIT_SIG_COND_RST, OFFSET_SIG_COND_RST, 1)
      rslt_cfg = self._update_reg_bit_value(rslt_cfg[0], BIT_FIFO_MODE, OFFSET_FIFO_MODE, 0)
    else:
      rslt_usr = self._update_reg_bit_value(rslt_usr[0], BIT_FIFO_EN, OFFSET_FIFO_EN, 0)
      rslt_usr = self._update_reg_bit_value(rslt_usr, BIT_FIFO_RST, OFFSET_FIFO_RST, 1)
      rslt_usr = self._update_reg_bit_value(rslt_usr, BIT_SIG_COND_RST, OFFSET_SIG_COND_RST, 1)
      rslt_cfg = self._update_reg_bit_value(rslt_cfg[0], BIT_FIFO_MODE, OFFSET_FIFO_MODE, 0)
    rslt_fifo_en = self._update_reg_bit_value(rslt_fifo_en[0], BIT_ACCEL_FIFO_EN, OFFSET_ACCEL_FIFO_EN, accel)
    rslt_fifo_en = self._update_reg_bit_value(rslt_fifo_en, BIT_ZG_FIFO_EN, OFFSET_ZG_FIFO_EN, gz)
    slt_fifo_en = self._update_reg_bit_value(rslt_fifo_en, BIT_YG_FIFO_EN, OFFSET_YG_FIFO_EN, gy)
    slt_fifo_en = self._update_reg_bit_value(rslt_fifo_en, BIT_XG_FIFO_EN, OFFSET_XG_FIFO_EN, gx)
    slt_fifo_en = self._update_reg_bit_value(rslt_fifo_en, BIT_TEMP_FIFO_EN, OFFSET_TEMP_FIFO_EN, temp)
    self._write_bytes(REG_ICG20660L_FIFO_EN,  [slt_fifo_en])
    rslt_fifo_en = self._read_bytes(REG_ICG20660L_FIFO_EN, 1)
    temp = self._get_reg_bit_value(rslt_fifo_en[0], BIT_TEMP_FIFO_EN, OFFSET_TEMP_FIFO_EN)
    gx = self._get_reg_bit_value(rslt_fifo_en[0], BIT_XG_FIFO_EN, OFFSET_XG_FIFO_EN)
    gy = self._get_reg_bit_value(rslt_fifo_en[0], BIT_YG_FIFO_EN, OFFSET_YG_FIFO_EN)
    gz = self._get_reg_bit_value(rslt_fifo_en[0], BIT_ZG_FIFO_EN, OFFSET_ZG_FIFO_EN)
    accel = self._get_reg_bit_value(rslt_fifo_en[0], BIT_ACCEL_FIFO_EN, OFFSET_ZG_FIFO_EN)
    self._fifo_frame_size = accel*6 + (temp+gx+gy+gz)*2
  
  def _set_full_scale_for_gyro(self, scale):
    scale &= 0x03
    rslt = self._read_bytes(REG_ICG20660L_GYRO_CONFIG, 1)
    rslt = self._update_reg_bit_value(rslt[0], BIT_XG_ST, OFFSET_XG_ST, 0)
    rslt = self._update_reg_bit_value(rslt, BIT_YG_ST, OFFSET_YG_ST, 0)
    rslt = self._update_reg_bit_value(rslt, BIT_ZG_ST, OFFSET_ZG_ST, 0)
    rslt = self._update_reg_bit_value(rslt, BIT_FS_SEL, OFFSET_FS_SEL, scale)
    self._write_bytes(REG_ICG20660L_GYRO_CONFIG, [rslt])
    if scale == self.eFSR_G_125DPS:
      self._gyro_range = GYRO_FULL_SCALE_125DPS
    elif scale == self.eFSR_G_250DPS:
      self._gyro_range = GYRO_FULL_SCALE_250DPS
    elif scale == self.eFSR_G_500DPS:
      self._gyro_range = GYRO_FULL_SCALE_500DPS
    self._gyro_scale = ADC_MAX_RANGE/self._gyro_range

  def _set_bandwidth_for_gyro(self, bd):
    rslt_gyro = self._read_bytes(REG_ICG20660L_GYRO_CONFIG, 1)
    rslt = self._read_bytes(REG_ICG20660L_CONFIG, 1)
    rslt_gyro = self._update_reg_bit_value(rslt_gyro[0], BIT_FCHOICE_B, OFFSET_FCHOICE_B, 0)
    rslt = self._update_reg_bit_value(rslt[0], BIT_DLPF_CFG, OFFSET_DLPF_CFG, 0)

    if bd == self.eGYRO_DLPF_8173_32KHZ:
      rslt_gyro = self._update_reg_bit_value(rslt_gyro, BIT_FCHOICE_B, OFFSET_FCHOICE_B, 1)

    elif bd == self.eGYRO_DLPF_3281_32KHZ:
      rslt_gyro = self._update_reg_bit_value(rslt_gyro, BIT_FCHOICE_B, OFFSET_FCHOICE_B, 2)

    elif bd == self.eGYRO_DLPF_250_8KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_DLPF_CFG, OFFSET_DLPF_CFG, 0)

    elif bd == self.eGYRO_DLPF_176_1KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_DLPF_CFG, OFFSET_DLPF_CFG, 1)

    elif bd == self.eGYRO_DLPF_92_1KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_DLPF_CFG, OFFSET_DLPF_CFG, 2)

    elif bd == self.eGYRO_DLPF_3281_8KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_DLPF_CFG, OFFSET_DLPF_CFG, 7)

    #print(type(rslt))
    #print(rslt)
    #print(type(rslt_gyro))
    #print(rslt_gyro)
    #print("rslt_gyro=%#x,rslt=%#x"%(rslt_gyro, rslt))
    self._write_bytes(REG_ICG20660L_GYRO_CONFIG, [rslt_gyro])
    self._write_bytes(REG_ICG20660L_CONFIG, [rslt])
    rslt_gyro = self._read_bytes(REG_ICG20660L_GYRO_CONFIG, 1)
    rslt = self._read_bytes(REG_ICG20660L_CONFIG, 1)
    #print("rslt_gyro=%#x,rslt=%#x"%(rslt_gyro[0], rslt[0]))

  def _set_full_scale_for_accel(self, scale):
    scale &= 0x03
    rslt = self._read_bytes(REG_ICG20660L_ACCEL_CONFIG, 1)
    rslt = self._update_reg_bit_value(rslt[0], BIT_XA_ST, OFFSET_XA_ST, 0)
    rslt = self._update_reg_bit_value(rslt, BIT_YA_ST, OFFSET_YA_ST, 0)
    rslt = self._update_reg_bit_value(rslt, BIT_ZA_ST, OFFSET_ZA_ST, 0)
    rslt = self._update_reg_bit_value(rslt, BIT_ACCEL_FS_SEL, OFFSET_ACCEL_FS_SEL, scale)
    self._write_bytes(REG_ICG20660L_ACCEL_CONFIG, [rslt])
    if scale == self.eFSR_A_2G:
      self._accel_range = ACCEL_FULL_SCALE_2G
    elif scale == self.eFSR_A_4G:
      self._accel_range = ACCEL_FULL_SCALE_4G
    elif scale == self.eFSR_A_8G:
      self._accel_range = ACCEL_FULL_SCALE_8G
    elif scale == self.eFSR_A_16G:
      _accel_range = ACCEL_FULL_SCALE_16G
    self.accel_scale = ADC_MAX_RANGE/self._accel_range

  def _set_bandwidth_for_accel(self, bd):
    rslt = self._read_bytes(REG_ICG20660L_PWR_MGMT_1, 1)
    if self._mode == self.eACCEL_LOW_POWER_MODE:
      rslt = self._update_reg_bit_value(rslt[0], BIT_CYCLE, OFFSET_CYCLE, 1)
      self._set_bandwidth_for_accel_in_low_power_mode(bd)
    elif self._mode == self.eSIX_AXIS_LOW_NOISE_MODE:
      rslt = self._update_reg_bit_value(rslt[0], BIT_CYCLE, OFFSET_CYCLE, 0)
      rslt = self._update_reg_bit_value(rslt, BIT_GYRO_STANDBY, OFFSET_GYRO_STANDBY, 0)
      self._set_bandwidth_for_accel_in_others_mode(bd)
    self._write_bytes(REG_ICG20660L_PWR_MGMT_1, [rslt])

  def _set_bandwidth_for_accel_in_others_mode(self, bd):
    rslt = self._read_bytes(REG_ICG20660L_ACCEL_CONFIG2, 1)
    rslt = self._update_reg_bit_value(rslt[0], BIT_FIFO_SIZE, OFFSET_FIFO_SIZE, 0)
    rslt = self._update_reg_bit_value(rslt, BIT_DEC2_CFG, OFFSET_DEC2_CFG, 0)
    if bd == self.eACCEL_DLPF_5_1KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_ACCEL_FCHOICE_B, OFFSET_ACCEL_FCHOICE_B, 0)
      rslt = self._update_reg_bit_value(rslt, BIT_A_DLPF_CFG, OFFSET_A_DLPF_CFG, 6)
    elif bd == self.eACCEL_DLPF_10_1KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_ACCEL_FCHOICE_B, OFFSET_ACCEL_FCHOICE_B, 0)
      rslt = self._update_reg_bit_value(rslt, BIT_A_DLPF_CFG, OFFSET_A_DLPF_CFG, 5)
    elif bd == self.eACCEL_DLPF_21_1KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_ACCEL_FCHOICE_B, OFFSET_ACCEL_FCHOICE_B, 0)
      rslt = self._update_reg_bit_value(rslt, BIT_A_DLPF_CFG, OFFSET_A_DLPF_CFG, 4)
    elif bd == self.eACCEL_DLPF_44_1KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_ACCEL_FCHOICE_B, OFFSET_ACCEL_FCHOICE_B, 0)
      rslt = self._update_reg_bit_value(rslt, BIT_A_DLPF_CFG, OFFSET_A_DLPF_CFG, 3)
    elif bd == self.eACCEL_DLPF_99_1KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_ACCEL_FCHOICE_B, OFFSET_ACCEL_FCHOICE_B, 0)
      rslt = self._update_reg_bit_value(rslt, BIT_A_DLPF_CFG, OFFSET_A_DLPF_CFG, 2)
    elif bd == self.eACCEL_DLPF_218_1KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_ACCEL_FCHOICE_B, OFFSET_ACCEL_FCHOICE_B, 0)
      rslt = self._update_reg_bit_value(rslt, BIT_A_DLPF_CFG, OFFSET_A_DLPF_CFG, 1)
    elif bd == self.eACCEL_DLPF_420_1KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_ACCEL_FCHOICE_B, OFFSET_ACCEL_FCHOICE_B, 0)
      rslt = self._update_reg_bit_value(rslt, BIT_A_DLPF_CFG, OFFSET_A_DLPF_CFG, 7)
    elif bd == self.eACCEL_DLPF_1046_4KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_ACCEL_FCHOICE_B, OFFSET_ACCEL_FCHOICE_B, 1)
    self._write_bytes(REG_ICG20660L_ACCEL_CONFIG2, [rslt])

  def _set_bandwidth_for_accel_in_low_power_mode(self, bd):
    rslt = self._read_bytes(REG_ICG20660L_ACCEL_CONFIG2, 1)
    rslt = self._update_reg_bit_value(rslt[0], BIT_FIFO_SIZE, OFFSET_FIFO_SIZE, 0)
    rslt = self._update_reg_bit_value(rslt, BIT_DEC2_CFG, OFFSET_DEC2_CFG, 0)
    if bd == self.eACCEL_DLPF_218_1KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_ACCEL_FCHOICE_B, OFFSET_ACCEL_FCHOICE_B, 0)
      rslt = self._update_reg_bit_value(rslt, BIT_A_DLPF_CFG, OFFSET_A_DLPF_CFG, 7)
      rslt = self._update_reg_bit_value(rslt, BIT_DEC2_CFG, OFFSET_DEC2_CFG, 1)
    elif bd == self.eACCEL_DLPF_420_1KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_ACCEL_FCHOICE_B, OFFSET_ACCEL_FCHOICE_B, 0)
      rslt = self._update_reg_bit_value(rslt, BIT_A_DLPF_CFG, OFFSET_A_DLPF_CFG, 7)
      rslt = self._update_reg_bit_value(rslt, BIT_DEC2_CFG, OFFSET_DEC2_CFG, 0)
    elif bd == self.eACCEL_DLPF_55_1KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_ACCEL_FCHOICE_B, OFFSET_ACCEL_FCHOICE_B, 0)
      rslt = self._update_reg_bit_value(rslt, BIT_A_DLPF_CFG, OFFSET_A_DLPF_CFG, 7)
      rslt = self._update_reg_bit_value(rslt, BIT_DEC2_CFG, OFFSET_DEC2_CFG, 3)
    elif bd == self.eACCEL_DLPF_110_1KHZ:
      rslt = self._update_reg_bit_value(rslt, BIT_ACCEL_FCHOICE_B, OFFSET_ACCEL_FCHOICE_B, 0)
      rslt = self._update_reg_bit_value(rslt, BIT_A_DLPF_CFG, OFFSET_A_DLPF_CFG, 7)
      rslt = self._update_reg_bit_value(rslt, BIT_DEC2_CFG, OFFSET_DEC2_CFG, 2)
    self._write_bytes(REG_ICG20660L_ACCEL_CONFIG2, [rslt])
  
  def _read_data_from_fifo(self):
    rslt_fifo = self._read_bytes(REG_ICG20660L_FIFO_EN, 1)
    val = self._read_bytes(REG_ICG20660L_FIFO_COUNTH, 2)
    val = (((val[0] & 0x1F) << 8)&0xffff) | val[1]
    count = 0
    if val >= self._fifo_frame_size:
      rslt = self._read_bytes(REG_ICG20660L_FIFO_R_W, self._fifo_frame_size)
      if self._get_reg_bit_value(rslt_fifo[0], BIT_TEMP_FIFO_EN, OFFSET_TEMP_FIFO_EN) and count < self._fifo_frame_size:
        self._raw_data[RAW_DATA_T_H_INDEX] = rslt[count]
        self._raw_data[RAW_DATA_T_L_INDEX] = rslt[count+1]
        count += 2
      if self._get_reg_bit_value(rslt_fifo[0], BIT_XG_FIFO_EN, OFFSET_XG_FIFO_EN) and count < self._fifo_frame_size:
        self._raw_data[RAW_DATA_GX_H_INDEX] = rslt[count]
        self._raw_data[RAW_DATA_GX_L_INDEX] = rslt[count+1]
        count += 2
      if self._get_reg_bit_value(rslt_fifo[0], BIT_YG_FIFO_EN, OFFSET_YG_FIFO_EN) and count < self._fifo_frame_size:
        self._raw_data[RAW_DATA_GY_H_INDEX] = rslt[count]
        self._raw_data[RAW_DATA_GY_L_INDEX] = rslt[count+1]
        count += 2
      if self._get_reg_bit_value(rslt_fifo[0], BIT_ZG_FIFO_EN, OFFSET_ZG_FIFO_EN) and count < self._fifo_frame_size:
        self._raw_data[RAW_DATA_GZ_H_INDEX] = rslt[count]
        self._raw_data[RAW_DATA_GZ_L_INDEX] = rslt[count+1]
        count += 2
      if self._get_reg_bit_value(rslt_fifo[0], BIT_ACCEL_FIFO_EN, OFFSET_ACCEL_FIFO_EN) and count < self._fifo_frame_size:
        self._raw_data[RAW_DATA_AX_H_INDEX] = rslt[count]
        self._raw_data[RAW_DATA_AX_L_INDEX] = rslt[count+1]
        self._raw_data[RAW_DATA_AY_H_INDEX] = rslt[count+2]
        self._raw_data[RAW_DATA_AY_L_INDEX] = rslt[count+3]
        self._raw_data[RAW_DATA_AZ_H_INDEX] = rslt[count+4]
        self._raw_data[RAW_DATA_AZ_L_INDEX] = rslt[count+5]
        count += 6
    
  def _read_data_from_reg(self):
    rslt = self._read_bytes(REG_ICG20660L_ACCEL_XOUT_H, RAW_DATA_LENGTH)
    #print(rslt)
    if len(rslt) == RAW_DATA_LENGTH:
      self._raw_data[RAW_DATA_AX_H_INDEX] = rslt[0]
      self._raw_data[RAW_DATA_AX_L_INDEX] = rslt[1]
      self._raw_data[RAW_DATA_AY_H_INDEX] = rslt[2]
      self._raw_data[RAW_DATA_AY_L_INDEX] = rslt[3]
      self._raw_data[RAW_DATA_AZ_H_INDEX] = rslt[4]
      self._raw_data[RAW_DATA_AZ_L_INDEX] = rslt[5]
      self._raw_data[RAW_DATA_T_H_INDEX] =  rslt[6]
      self._raw_data[RAW_DATA_T_L_INDEX] =  rslt[7]
      self._raw_data[RAW_DATA_GX_H_INDEX] = rslt[8]
      self._raw_data[RAW_DATA_GX_L_INDEX] = rslt[9]
      self._raw_data[RAW_DATA_GY_H_INDEX] = rslt[10]
      self._raw_data[RAW_DATA_GY_L_INDEX] = rslt[11]
      self._raw_data[RAW_DATA_GZ_H_INDEX] = rslt[12]
      self._raw_data[RAW_DATA_GZ_L_INDEX] = rslt[13]
  
  def _write_bytes(self, reg, buf):
    pass

  def _read_bytes(self, reg, length):
    pass

class DFRobot_ICG20660L_IIC(DFRobot_ICG20660L):
  def __init__(self,addr):
    '''
      @brief The constructor of the ICG20660L sensor using IIC communication.
      @param addr:  7-bit IIC address, controlled by SDO pin.
      @n     IIC_ADDR_SDO_H or 0x69:  SDO pull high.(default)
      @n     IIC_ADDR_SDO_L or 0x68:  SDO pull down.
    '''
    self._addr = addr
    self._bus = smbus.SMBus(1)
    DFRobot_ICG20660L.__init__(self)


  def _write_bytes(self, reg, buf):
    try:
      self._bus.write_i2c_block_data(self._addr, reg, buf)
    except:
      pass

  def _read_bytes(self, reg, length):
    try:
      rslt = self._bus.read_i2c_block_data(self._addr, reg, length)
      return rslt
    except:
      return [0]*length

class DFRobot_ICG20660L_SPI(DFRobot_ICG20660L):
  def __init__(self, cs):
    '''
      @brief The constructor of the ICG20660L sensor using SPI communication.
      @param cs:  SPI chip select pin, connected to IO pin of raspberry pi.
    '''
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    self._cs = cs
    self._spi = spidev.SpiDev()
    self._spi.open(0,0)
    self._spi.no_cs = True
    self._spi.max_speed_hz = 7000000
    GPIO.setup(self._cs, GPIO.OUT)
    DFRobot_ICG20660L.__init__(self)

  def _write_bytes(self, reg, buf):
    try:
      self.set_cs_low()
      self._spi.writebytes([reg])
      self._spi.writebytes(buf)
      self.set_cs_high()
    except:
      pass

  def _read_bytes(self, reg, length):
    try:
      self.set_cs_low()
      reg |= 0x80
      self._spi.writebytes([reg])
      rslt = self._spi.readbytes(length)
      self.set_cs_high()
      return rslt
    except:
      return [0]*length

  def set_cs_low(self):
     GPIO.output(self._cs, GPIO.LOW)

  def set_cs_high(self):
     GPIO.output(self._cs, GPIO.HIGH)
    


