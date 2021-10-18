/*!
 * @file DFRobot_ICG20660L.h
 * @brief The ICG-20660 is a 6-axis MotionTracking device that combines a 3-axis gyroscope, 3-axis accelerometer.
 * @n It support two communication interface:
 * @n (1) IIC-->freq: 100~400Khz
 * @n (2) SPI-->freq: 100kHz~7MHz, only support mode0 or mode3
 * @n Two communication methods are switched by cs pin, 0:SPI, 1:IIC
 * @n 3-axis accelerometer feature:
 * @n (1) Support max ranging: ±2g、±4g、±8g、±16g, g = 9.80665 m/s²
 * @n (2) 1g = 9.80665 m/s²
 * @n 3-axis gyroscope feature:
 * @n (1) Support max ranging: ±125dps、±250dps、±500dps
 * @n (2) 1dps = Π/180° rad/s, Π = 3.1415926
 * @n 运动阈值唤醒检测：
 * @n 是前一次和后一次加速度的阈值差，如果大于或等于设定的阈值，将产生中断。
 * @n 支持从寄存器读取和从FIFO读取：
 * @n FIFO读取，加速度，陀螺仪、温度必须全部使能，且将其内部采样率必须配置成一致
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [Arya](xue.peng@dfrobot.com)
 * @version  V1.0
 * @date  2021-05-25
 * @https://github.com/DFRobot/DFRobot_ICG20660L
 */
#ifndef __DFRobot_ICG20660L_H
#define __DFRobot_ICG20660L_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Wire.h>
#include <SPI.h>

//Define DBG, change 0 to 1 open the DBG, 1 to 0 to close.  
#if 0
#define DBG(...) {Serial.print("["); Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

#if 0
#define DBGREG(name, addr, value) {Serial.print("["); Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.print(name);Serial.print("="); Serial.print(addr,HEX); Serial.print("=");Serial.println(value,HEX);}
#else
#define DBGREG(name, addr, value)
#endif

#define IIC_ADDR_SDO_H   0x69
#define IIC_ADDR_SDO_L   0x68

//Can be accessed in sleep mode
#define REG_ICG20660L_SMPLRT_DIV     0x19
#define REG_ICG20660L_ACCEL_CONFIG2  0x1D
#define REG_ICG20660L_INT_PIN_CFG    0x37
#define REG_ICG20660L_INT_ENABLE     0x38
#define REG_ICG20660L_PWR_MGMT_1     0x6B
#define REG_ICG20660L_PWR_MGMT_2     0x6C
#define REG_ICG20660L_FIFO_COUNTH    0x72
#define REG_ICG20660L_FIFO_COUNTL    0x73
#define REG_ICG20660L_FIFO_R_W       0x74




//Can't be accessed in sleep mode
#define REG_ICG20660L_SELF_TEST_X_GYRO              0x00
#define REG_ICG20660L_SELF_TEST_Y_GYRO              0x01
#define REG_ICG20660L_SELF_TEST_Z_GYRO              0x02
#define REG_ICG20660L_XG_OFFS_TC_H                  0x04
#define REG_ICG20660L_XG_OFFS_TC_L                  0x05
#define REG_ICG20660L_YG_OFFS_TC_H                  0x07
#define REG_ICG20660L_YG_OFFS_TC_L                  0x08
#define REG_ICG20660L_ZG_OFFS_TC_H                  0x0A
#define REG_ICG20660L_ZG_OFFS_TC_L                  0x0B
#define REG_ICG20660L_SELF_TEST_X_ACCEL             0x0D
#define REG_ICG20660L_SELF_TEST_Y_ACCEL             0x0E
#define REG_ICG20660L_SELF_TEST_Z_ACCEL             0x0F
#define REG_ICG20660L_XG_OFFS_USRH                  0x13
#define REG_ICG20660L_XG_OFFS_USRL                  0x14
#define REG_ICG20660L_YG_OFFS_USRH                  0x15
#define REG_ICG20660L_YG_OFFS_USRL                  0x16
#define REG_ICG20660L_ZG_OFFS_USRH                  0x17
#define REG_ICG20660L_ZG_OFFS_USRL                  0x18
#define REG_ICG20660L_CONFIG                        0x1A
#define REG_ICG20660L_GYRO_CONFIG                   0x1B
#define REG_ICG20660L_ACCEL_CONFIG                  0x1C
#define REG_ICG20660L_ACCEL_CONFIG2                 0x1D
#define REG_ICG20660L_LP_MODE_CFG                   0x1E
#define REG_ICG20660L_ACCEL_WOM_THR                 0x1F
#define REG_ICG20660L_FIFO_EN                       0x23
#define REG_ICG20660L_FSYNC_INT                     0x36
#define REG_ICG20660L_INT_PIN_CFG                   0x37
#define REG_ICG20660L_INT_STATUS                    0x3A
#define REG_ICG20660L_ACCEL_XOUT_H                  0x3B
#define REG_ICG20660L_ACCEL_XOUT_L                  0x3C
#define REG_ICG20660L_ACCEL_YOUT_H                  0x3D
#define REG_ICG20660L_ACCEL_YOUT_L                  0x3E
#define REG_ICG20660L_ACCEL_ZOUT_H                  0x3F
#define REG_ICG20660L_ACCEL_ZOUT_L                  0x40
#define REG_ICG20660L_TEMP_OUT_H                    0x41
#define REG_ICG20660L_TEMP_OUT_L                    0x42
#define REG_ICG20660L_GYRO_XOUT_H                   0x43
#define REG_ICG20660L_GYRO_XOUT_L                   0x44
#define REG_ICG20660L_GYRO_YOUT_H                   0x45
#define REG_ICG20660L_GYRO_YOUT_L                   0x46
#define REG_ICG20660L_GYRO_ZOUT_H                   0x47
#define REG_ICG20660L_GYRO_ZOUT_L                   0x48
#define REG_ICG20660L_SIGNAL_PATH_RESET             0x68
#define REG_ICG20660L_ACCEL_INTEL_CTRL              0x69
#define REG_ICG20660L_USER_CTRL                     0x6A
#define REG_ICG20660L_WHO_AM_I                      0x75
#define REG_ICG20660L_XA_OFFSET_H                   0x77
#define REG_ICG20660L_XA_OFFSET_L                   0x78
#define REG_ICG20660L_YA_OFFSET_H                   0x7A
#define REG_ICG20660L_YA_OFFSET_L                   0x7B
#define REG_ICG20660L_ZA_OFFSET_H                   0x7D
#define REG_ICG20660L_ZA_OFFSET_L                   0x7E

#define RAW_DATA_AX_H_INDEX                      0x00
#define RAW_DATA_AX_L_INDEX                      0x01
#define RAW_DATA_AY_H_INDEX                      0x02
#define RAW_DATA_AY_L_INDEX                      0x03
#define RAW_DATA_AZ_H_INDEX                      0x04
#define RAW_DATA_AZ_L_INDEX                      0x05
#define RAW_DATA_T_H_INDEX                       0x06
#define RAW_DATA_T_L_INDEX                       0x07
#define RAW_DATA_GX_H_INDEX                      0x08
#define RAW_DATA_GX_L_INDEX                      0x09
#define RAW_DATA_GY_H_INDEX                      0x0A
#define RAW_DATA_GY_L_INDEX                      0x0B
#define RAW_DATA_GZ_H_INDEX                      0x0C
#define RAW_DATA_GZ_L_INDEX                      0x0D
#define RAW_DATA_LENGTH                          14

#define ICG20660L_WOM_XYZ_INT     7 << 5

typedef struct{
  String reg;
  uint8_t value;
}sRegDecrisption_t;

typedef struct{
  float x;/*<X-axis sensor data*/
  float y;/*<Y-axis sensor data*/
  float z;/*<Z-axis sensor data */
}sIcg20660SensorData_t;

class DFRobot_ICG20660L{
public:
  
  typedef enum{/*35page */
    eSleepMode = 0,/**<gyro: off, accel:off, power consumption in esp32: 67.3uA*/
    eStandbyMode = 1,/**<gyro: drive on, accel:off,*/
    eAccelLowPowerMode = 2,/**<gyro: off, accel:duty_cycled,*/
    eAccelLowNoiseMode = 3,/**<gyro: off, accel:on,*/
    eSixAxisLowNoiseMode = 4,/**<gyro: on, accel:on,*/
  }ePowerMode_t;
  
  typedef enum{
      eFIFOMode,/**<Read sensor data by data register.*/
      eRegMode/**<Read sensor data by 512 bytes fifo.*/
  }eDataReadMode_t;
  
  typedef enum{
    eFSR_G_125DPS = 0,/**<The full scale range of gyro: 0~±125dps, 1dps = Π/180° rad/s, Π = 3.1415926.*/
    eFSR_G_250DPS,/**<The full scale range of gyro: 0~±250dps, 1dps = Π/180° rad/s, Π = 3.1415926.*/
    eFSR_G_500DPS/**<The full scale range of gyro: 0~±500dps, 1dps = Π/180° rad/s, Π = 3.1415926.*/
  }eGyroFSR_t;
  
  typedef enum{
    eFSR_A_2G = 0,/**<The full scale range of accel: 0~±2g, 1g = 9.80665 m/s².*/
    eFSR_A_4G,/**<The full scale range of accel: 0~±4g, 1g = 9.80665 m/s².*/
    eFSR_A_8G,/**<The full scale range of accel: 0~±8g, 1g = 9.80665 m/s².*/
    eFSR_A_16G/**<The full scale range of accel: 0~±16g, 1g = 9.80665 m/s².*/
  }eAccelFSR_t;
  
  typedef enum{
    eGyroAxisZ  = 1 << 6|1 << 0,/**<The bit is gyro's z axis, you can call enableSensor or disableSensor function to enable or disable the gyro's Z axis.*/
    eGyroAxisY  = 1 << 6|1 << 1,/**<The bit is gyro's y axis, you can call enableSensor or disableSensor function to enable or disable the gyro's Y axis. */
    eGyroAxisX  = 1 << 6|1 << 2,/**<The bit is gyro's z axis, you can call enableSensor or disableSensor function to enable or disable the gyro's X axis. */
    eAccelAxisZ = 1 << 3,/**<The bit is accel's z axis, you can call enableSensor or disableSensor function to enable or disable the accel's Z axis.*/
    eAccelAxisY = 1 << 4,/**<The bit is accel's Y axis, you can call enableSensor or disableSensor function to enable or disable the accel's Y axis.*/
    eAccelAxisX = 1 << 5,/**<The bit is accel's X axis, you can call enableSensor or disableSensor function to enable or disable the accel's X axis.*/
    eGyroAxisXYZ = 1 << 6|0x07,/**<The bits of gyro's xyz axis, you can call enableSensor or disableSensor function to enable or disable the gyro's xyz axis.*/
    eAccelAxisXYZ = 0x07 << 3,/**<The bits of accel's xyz axis, you can call enableSensor or disableSensor function to enable or disable the accel's xyz axis.*/
    eAxisAll = 1 << 6|0x3F,/**<The gryo's and accel's xyz axis, you can call enableSensor or disableSensor function to enable or disable the accel's and gyro's xyz axis.*/
  }eSensorEnable_t;
  
  typedef enum{
    eGyro_DLPF_8173_32KHZ = 0,/**<当信号等于或大于8173Hz时，会出现明显衰减，衰减3-db，内部采样率为32KHz*/
    eGyro_DLPF_3281_32KHZ,/**<当信号等于或大于3218Hz时，会出现明显衰减，衰减3-db，内部采样率为32KHz*/
    eGyro_DLPF_250_8KHZ,/**<当信号等于或大于250Hz时，会出现明显衰减，衰减3-db，内部采样率为8KHz*/
    eGyro_DLPF_176_1KHZ,/**<当信号等于或大于176Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz*/
    eGyro_DLPF_92_1KHZ,/**<当信号等于或大于92Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz*/
    eGyro_DLPF_3281_8KHZ,/**<当信号等于或大于3218Hz时，会出现明显衰减，衰减3-db，内部采样率为8KHz*/
  }eGyroBandwidth_t;
  
  typedef enum{
    eAccel_DLPF_5_1KHZ = 0,/**<当信号小于或等于5Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz*/
    eAccel_DLPF_10_1KHZ,/**<当信号小于或等于10Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz*/
    eAccel_DLPF_21_1KHZ,/**<当信号小于或等于21Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz*/
    eAccel_DLPF_44_1KHZ,/**<当信号小于或等于44Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz*/
    eAccel_DLPF_99_1KHZ,/**<当信号小于或等于99Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz*/
    eAccel_DLPF_218_1KHZ,/**<该配置也支持低功耗，当信号小于或等于218Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz*/
    eAccel_DLPF_420_1KHZ,/**<该配置也支持低功耗，当信号小于或等于420Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz*/
    eAccel_DLPF_1046_4KHZ,/**<该配置也支持低功耗，当信号小于或等于1046Hz时，会出现明显衰减，衰减3-db，内部采样率为4KHz*/
    eAccel_DLPF_55_1KHZ,/**<该配置只支持低功耗，当信号小于或等于55Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz*/
    eAccel_DLPF_110_1KHZ,/**<该配置只支持低功耗，当信号小于或等于110Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz*/
  }eAccelBandwidth_t;
  
  typedef enum{
      eODR_0_24Hz = 0, /**<the low power accel Output Data Rate: 0.24Hz*/
      eODR_0_49Hz = 1,/**<the low power accel Output Data Rate: 0.49Hz*/
      eODR_0_98Hz = 2,/**<the low power accel Output Data Rate: 0.98Hz*/
      eODR_1_95Hz = 3,/**<the low power accel Output Data Rate: 1.95Hz*/
      eODR_3_91Hz = 4,/**<the low power accel Output Data Rate: 3.91Hz*/
      eODR_7_81Hz = 5,/**<the low power accel Output Data Rate: 7.81Hz*/
      eODR_15_63Hz = 6,/**<the low power accel Output Data Rate: 15.63Hz*/
      eODR_31_25Hz = 7,/**<the low power accel Output Data Rate: 31.25Hz*/
      eODR_62_50Hz = 8,/**<the low power accel Output Data Rate: 62.50Hz*/
      eODR_125Hz = 9,/**<the low power accel Output Data Rate: 125Hz*/
      eODR_250Hz = 10,/**<the low power accel Output Data Rate: 250Hz*/
      eODR_500Hz = 11,/**<the low power accel Output Data Rate: 500Hz*/
  }eODR_t;
  
  DFRobot_ICG20660L();
  ~DFRobot_ICG20660L();
/**
 * @brief 初始化传感器，初始化后，所有传感器都被关闭，需通过enableSensor打开相应的配置. 
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

protected:
  virtual int init() = 0;
  virtual void writeReg(uint8_t reg, void *pData, size_t len) = 0;
  virtual size_t readReg(uint8_t reg, void *pdata, size_t len) = 0;
  void setFullScaleForGyro(uint8_t scale);
  void setFullScaleForAccel(uint8_t scale);
  void setBandwidthForAccel(eAccelBandwidth_t  bd);
  void setBandwidthForGyro(eGyroBandwidth_t  bd);
  //void test();
  //void test1();
  void selectClockSource(uint8_t clksel);
  void setBandwidthForAccelInOthersMode(eAccelBandwidth_t  bd);
  void setBandwidthForAccelInLowPowerMode(eAccelBandwidth_t  bd);
  void enableFifo(bool temp, bool gx, bool gy, bool gz, bool accel);
/**
 * @brief Take out of sleep mode. 
 * @param en  The IO pin of MCU which is connected to the EN pin of Non-contact liquid level sensor.
 * @param cmd  calibration command in UART Mode which is Used to distinguish between upper water level calibration and lower water level calibration.
 * @n     CALIB_UART_CMD_LWL: Lower water Level calibration command.
 * @n     CALIB_UART_CMD_UWL: Upper water Level calibration command.
 * @return Calibration state:
 * @n      true:  Calibration sucess.
 * @n      false:  Calibration failed.
 */
  void outSleepMode();//从睡眠模式退出，达到了3.09mA，将gyroStandby置1，达到1.61mA
  void readDataFromREG();
  void readDataFromFIFO();
private:
  #define CLOCK_SEL_PLL   1
  #define ADC_MAX_RANGE   32767.0
  #define GYRO_FULL_SCALE_125DPS  125.0  //unit: dps
  #define GYRO_FULL_SCALE_250DPS  250.0  //unit: dps
  #define GYRO_FULL_SCALE_500DPS  500.0  //unit: dps
  #define ICG20660L_DEVICE_ID     0x91
  
  #define ACCEL_FULL_SCALE_2G   2    //unit: g
  #define ACCEL_FULL_SCALE_4G   4    //unit: g
  #define ACCEL_FULL_SCALE_8G   8    //unit: g
  #define ACCEL_FULL_SCALE_16G  16   //unit: g
  
  typedef union{
    struct{
        uint8_t bitGyroZ: 1;
        uint8_t bitGyroY: 1;
        uint8_t bitGyroX: 1;
        uint8_t bitAccelZ: 1;
        uint8_t bitAccelY: 1;
        uint8_t bitAccelX: 1;
        uint8_t bitTemp: 1;
        uint8_t bitFifoLpEn:1;
    };
    uint8_t value;
  }uSensorEnable_t;
  /*
   FIFO_EN register：addr:0x23,acess:rw
   * --------------------------------------------------------------------------------------
   * |      b7      |     b6     |     b5     |     b4     |      b3       | b2 | b1 | b0 |
   * --------------------------------------------------------------------------------------
   * | TEMP_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN | ACCEL_FIFO_EN |      rsv     |
   * --------------------------------------------------------------------------------------
   */
  typedef union{
    struct{
        uint8_t rsv:3;/**<Reserved.*/
        uint8_t ACCEL_FIFO_EN:1;/**<1 – Write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L to the FIFO at the sample rate.*/
        uint8_t ZG_FIFO_EN:1;/**< 1 – Write GYRO_ZOUT_H and GYRO_ZOUT_L to the FIFO at the sample rate; If enabled, buffering of data occurs even if data path is in standby.*/
        uint8_t YG_FIFO_EN:1;/**<1 – Write GYRO_YOUT_H and GYRO_YOUT_L to the FIFO at the sample rate; If enabled, buffering of data occurs even if data path is in standby.*/
        uint8_t XG_FIFO_EN:1;/**<1 – Write GYRO_XOUT_H and GYRO_XOUT_L to the FIFO at the sample rate; If enabled, buffering of data occurs even if data path is in standby.*/
        uint8_t TEMP_FIFO_EN:1;/**<1 – Write TEMP_OUT_H and TEMP_OUT_L to the FIFO at the sample rate; If enabled, buffering of data occurs even if data path is in standby.*/
    };
    uint8_t value;
  }eFifoEnReg_t;
 
  /*
   Config register：addr:0x1A,acess:rw
   * -------------------------------------------------
   * |  b7 |     b6    | b5 | b4 | b3 | b2 | b1 | b0 |
   * -------------------------------------------------
   * | rsv | FIFO_MODE | EXT_SYNC_SET |   DLPF_CFG   |
   * -------------------------------------------------
   */
  typedef union{
    struct{
        uint8_t DLPF_CFG:3;/**<For the DLPF to be used, FCHOICE_B[1:0] is 2’b00.*/
        uint8_t EXT_SYNC_SET:3;/**<Enables the FSYNC pin data to be sampled.0:disable*/
        uint8_t FIFO_MODE:1;/**< When set to ‘1’, when the FIFO is full, additional writes will not be written to FIFO.Or additional writes will be written to the FIFO, replacing the oldest data.*/
        uint8_t rsv:1;/**<reserve bit*/
    };
    uint8_t value;
  }uConfigReg_t;
  
  /*
   GYRO_CONFIG register：addr:0x1B,acess:rw
   * -----------------------------------------------------
   * |   b7  |  b6   |   b5  | b4 | b3 |  b2 |  b1 |  b0 |
   * -----------------------------------------------------
   * | XG_ST | YG_ST | ZG_ST |  FS_SEL | rsv | FCHOICE_B |
   * -----------------------------------------------------
   */
  typedef union{
    struct{
        uint8_t FCHOICE_B:2;/**<Used to bypass DLPF*/
        uint8_t rsv:1;/**<reserve bit*/
        uint8_t FS_SEL:2;/**<*Gyro Full Scale Select: 00- ±125dps, 01- ±250dps, 10: ±500dps*/
        uint8_t ZG_ST:1;/**<Z Gyro self-test*/
        uint8_t YG_ST:1;/**<Y Gyro self-test*/
        uint8_t XG_ST:1;/**<X Gyro self-test*/
    };
    uint8_t value;
  }uGyroConfigReg_t;
  
  /*
   ACCEL_CONFIG：addr:0x1C,acess:rw
   * -------------------------------------------------------
   * |   b7  |  b6   |   b5  |  b4  |  b3   | b2 | b1 | b0 |
   * -------------------------------------------------------
   * | XA_ST | YA_ST | ZA_ST | ACCEL_FS_SEL |     rsv      |
   * -------------------------------------------------------
   */
  typedef union{
    struct{
        uint8_t rsv:3;/**<reserve bit*/
        uint8_t ACCEL_FS_SEL:2;/**<Accel Full Scale Select:±2g (00), ±4g (01), ±8g (10), ±16g (11)*/
        uint8_t ZA_ST:1;/**<Z Accel self-test.*/
        uint8_t YA_ST:1;/**<Y Accel self-test.*/
        uint8_t XA_ST:1;/**<X Accel self-test.*/
    };
    uint8_t value;
  }uAccelConfigReg_t;
  
  /*
   ACCEL_CONFIG2：addr:0x1D,acess:rw
   * ---------------------------------------------------------
   * |  b7 |  b6 | b5 | b4  |        b3       | b2 | b1 | b0 |
   * ---------------------------------------------------------
   * | FIFO_SIZE | DEC2_CFG | ACCEL_FCHOICE_B |  A_DLPF_CFG  |
   * ---------------------------------------------------------
   */
  typedef union{
    struct{
        uint8_t A_DLPF_CFG:3;/**<Accelerometer low pass filter setting as shown in table 2 below.*/
        uint8_t ACCEL_FCHOICE_B:1;/**<Used to bypass DLPF as shown in the table below.*/
        uint8_t DEC2_CFG:2;/**<Averaging filter settings for Low Power Accelerometer mode: 0 = Average 4 samples,1 = Average 8 samples,2 = Average 16 samples,3 = Average 32 samples.*/
        uint8_t FIFO_SIZE:2;/**<Specifies FIFO size according to the following: 0 = 512bytes.*/
          };
    uint8_t value;
  }uAccelConfig2Reg_t;

  /*
   FSYNC_INT register：addr:0x36,acess:rw
   * ------------------------------------------------
   * |     b7    | b6 | b5 | b4 | b3 | b2 | b1 | b0 |
   * ------------------------------------------------
   * | FSYNC_INT |               rsv                |
   * ------------------------------------------------
   */
  typedef union{
    struct{
        uint8_t rsv:7;/**<Reserved.*/
        uint8_t FSYNC_INT:1;/**<This bit automatically sets to 1 when a FSYNC interrupt has been generated. The bit clears to 0 after the register has been read.*/
    };
    uint8_t value;
  }uFsyncIntReg_t;

  /*
   INT_PIN_CFG register：addr:0x37,acess:rw
   * ------------------------------------------------------------------------------------------------------
   * |     b7    |    b6    |       b5     |      b4      |        b3       |         b2        | b1 | b0 |
   * ------------------------------------------------------------------------------------------------------
   * | INT_LEVEL | INT_OPEN | LATCH_INT_EN | INT_RD_CLEAR | FSYNC_INT_LEVEL | FSYNC_INT_MODE_EN |   rsv   |
   * ------------------------------------------------------------------------------------------------------
   */
  typedef union{
    struct{
        uint8_t rsv:2;/**<Reserved.*/
        uint8_t FSYNC_INT_MODE_EN:1;/**<When this bit is equal to 1, the FSYNC pin will trigger an interrupt when it transitions to the level specified by FSYNC_INT_LEVEL. When this bit is equal to 0, the FSYNC pin is disabled from causing an interrupt.*/
        uint8_t FSYNC_INT_LEVEL:1;/**< 1 – The logic level for the FSYNC pin as an interrupt is active low.0 – The logic level for the FSYNC pin as an interrupt is active high.*/
        uint8_t INT_RD_CLEAR:1;/**<1 – Interrupt status is cleared if any read operation is performed.0 – Interrupt status is cleared only by reading INT_STATUS register*/
        uint8_t LATCH_INT_EN:1;/**<1 – INT pin level held until interrupt status is cleared.0 – INT pin indicates interrupt pulse’s width is 50us.*/
        uint8_t INT_OPEN:1;/**<1 – INT pin is configured as open drain.0 – INT pin is configured as push-pull.*/
        uint8_t INT_LEVEL:1;/**<1 – The logic level for INT pin is active low.0 – The logic level for INT pin is active high.*/
    };
    uint8_t value;
  }uIntPinCfgReg_t;//不支持开漏输出，如果要支持需要添加外部上拉电阻才能得到高电平
  
  /*
   INT_ENABLE register：addr:0x38,acess:rw
   * -----------------------------------------------------------------
   * | b7 | b6 | b5 |      b4       | b3 | b2 | b1 |       b0        |
   * -----------------------------------------------------------------
   * |    WOM_EN    | FIFO_OFLOW_EN |      rsv     | DATA_RDY_INT_EN |
   * -----------------------------------------------------------------
   */
  typedef union{
    struct{
        uint8_t DATA_RDY_INT_EN:1;/**<Data ready interrupt enable..*/
        uint8_t rsv:3;/**<When this bit is equal to 1, the FSYNC pin will trigger an interrupt when it transitions to the level specified by FSYNC_INT_LEVEL. When this bit is equal to 0, the FSYNC pin is disabled from causing an interrupt.*/
        uint8_t FIFO_OFLOW_EN:1;/**< 1 – Enables a FIFO buffer overflow to generate an interrupt.0 – Function is disabled.*/
        uint8_t WOM_EN:3;/**<‘111’ – Enable WoM interrupt.‘000’ – Disable WoM interrupt – This is the default setting.*/
    };
    uint8_t value;
  }uIntEnableReg_t;

  /*
   INT_STATUS register：addr:0x3A,acess:rw
   * ------------------------------------------------------------------------------------
   * |     b7    |    b6     |    b5     |      b4        | b3 | b2 | b1 |      b0      |
   * ------------------------------------------------------------------------------------
   * | WOM_X_INT | WOM_Y_INT | WOM_Z_INT | FIFO_OFLOW_INT |      rsv     | DATA_RDY_INT |
   * ------------------------------------------------------------------------------------
   */
  typedef union{
    struct{
        uint8_t DATA_RDY_INT:1;/**<This bit automatically sets to 1 when a Data Ready interrupt is generated. The bit clears to 0 after the register has been read.*/
        uint8_t rsv:3;/**<reserve*/
        uint8_t FIFO_OFLOW_INT:1;/**< This bit automatically sets to 1 when a FIFO buffer overflow has been generated. The bit clears to 0 after the register has been read.*/
        uint8_t WOM_Z_INT:1;/**<Z-axis accelerometer WoM interrupt status. Cleared on Read*/
        uint8_t WOM_Y_INT:1;/**<Y-axis accelerometer WoM interrupt status. Cleared on Read*/
        uint8_t WOM_X_INT:1;/**<X-axis accelerometer WoM interrupt status. Cleared on Read*/
    };
    uint8_t value;
  }uIntStatusReg_t;

  /*
   ACCEL_INTEL_CTRL register：addr:0x69,acess:rw
   * ----------------------------------------------------------------------------
   * |       b7       |        b6        | b5 | b4 | b3 | b2 | b1 |      b0     |
   * ----------------------------------------------------------------------------
   * | ACCEL_INTEL_EN | ACCEL_INTEL_MODE |           rsv          | WOM_TH_MODE |
   * ----------------------------------------------------------------------------
   */
  typedef union{
    struct{
        uint8_t WOM_TH_MODE:1;/**<This bit enables the Wake-on-Motion detection logic.*/
        uint8_t rsv:5;/**<reserve*/
        uint8_t ACCEL_INTEL_MODE:1;/**< 0 – Do not use.1 – Compare the current sample with the previous sample.*/
        uint8_t ACCEL_INTEL_EN:1;/**<0 – Set WoM interrupt on the OR of all enabled accelerometer thresholds.1 – Set WoM interrupt on the AND of all enabled accelerometer thresholds.*/
    };
    uint8_t value;
  }uAccelIntelCtrlReg_t;
  
  /*
   USER_CTRL register：addr:0x6A,acess:rw
   * -------------------------------------------------------------------------
   * | b7  |    b6   |  b5 |     b4     | b3  |    b2    | b1  |      b0     |
   * -------------------------------------------------------------------------
   * | rsv | FIFO_EN | rsv | I2C_IF_DIS | rsv | FIFO_RST | rsv | SIG_COND_RST|
   * -------------------------------------------------------------------------
   */
  typedef union{
    struct{
        uint8_t SIG_COND_RST:1; /**<1 – Reset all gyro digital signal path, accel digital signal path, and temp digital signal path. This bit also clears all the sensor registers.*/
        uint8_t rsv1:1;/**<Reserved.*/
        uint8_t FIFO_RST:1;/**<1 – Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle of the internal 20 MHz clock.*/
        uint8_t rsv2:1;/**<Reserved.*/
        uint8_t I2C_IF_DIS:1;/**<1 – Reset I2C Slave module and put the serial interface in SPI mode only. This bit auto clears afte one clock cycle of the internal 20 MHz clock.*/
        uint8_t rsv3:1;/**<Reserved.*/
        uint8_t FIFO_EN:1;/**<1 – Enable FIFO operation mode.*/
        uint8_t rsv4:1;/**<Reserved.*/
    };
    uint8_t value;
  }uUsrCtrlReg_t;
  
  /*
   PWR_MGMT_1 register：addr:0x6B,acess:rw
   * -------------------------------------------------------------------------
   * |      b7      |  b6   |   b5  |      b4      |    b3    | b2 | b1 | b0 |
   * -------------------------------------------------------------------------
   * | DEVICE_RESET | SLEEP | CYCLE | GYRO_STANDBY | TEMP_DIS |  CLKSEL[2:0] |
   * -------------------------------------------------------------------------
   */
  typedef union{
    struct{
        uint8_t clkSel:3;/**Clock Source: 0/6:Internal 20 MHz oscillator, 1~5:Auto selects the best available clock source – PLL if ready, else use the Internal oscillator, 7:Stops the clock and keeps timing generator in reset*/
        uint8_t tempDis:1;/**<When set to 1, this bit disables the temperature sensor.*/
        uint8_t gyroStandby:1;/**<When set, the gyro drive and pll circuitry are enabled, but the sense paths are disabled. This is a low power mode that allows quick enabling of the gyros.*/
        uint8_t cycle:1;/**<This bit depends on other registers and bits*/
        uint8_t sleep:1; /**<When set to 1, the chip is set to sleep mode.*/
        uint8_t deviceReset:1;/**<1:Reset the internal registers and restores the default settings,automatically clear after completion*/
    };
    uint8_t value;
  }uPowerManagement1Reg_t;
  
  /*
   PWR_MGMT_2 register：addr:0x6C,acess:rw
   * -------------------------------------------------------------------------------
   * |     b7     |  b6 |    b5   |    b4   |    b3   |    b2   |    b1   |   b0   |
   * -------------------------------------------------------------------------------
   * | FIFO_LP_EN | rsv | STBY_XA | STBY_YA | STBY_ZA | STBY_XG | STBY_YG | STBY_ZG|
   * -------------------------------------------------------------------------------
   */
  typedef union{
    struct{
        uint8_t STBY_ZG:1;/**<1: Z gyro is disabled, 0: Z gyro is on.*/
        uint8_t STBY_YG:1;/**<1: Y gyro is disabled, 0: Y gyro is on.*/
        uint8_t STBY_XG:1;/**<1: X gyro is disabled, 0: X gyro is on.*/
        uint8_t STBY_ZA:1;/**<1: Z accelerometer is disabled, 0:Z accelerometer is on.*/
        uint8_t STBY_YA:1;/**<1: Y accelerometer is disabled, 0: Y accelerometer is on.*/
        uint8_t STBY_XA:1;/**<1: X accelerometer is disabled, 0: X accelerometer is on.*/
        uint8_t rsv:1;/**<This bit depends on other registers and bits*/
        uint8_t FIFO_LP_EN:1; /**<1:fifo low power enable.*/
    };
    uint8_t value;
  }uPowerManagement2Reg_t;
  float _gyroScale;
  float _accelScale;
  float _gyroRange;
  float _accelRange;
  ePowerMode_t _mode;
  eDataReadMode_t _dataMode;
  uint8_t _fifoFrameSize;
  int _level;
  uint8_t _rawData[RAW_DATA_LENGTH];
  uint8_t _update;
};

class DFRobot_ICG20660L_IIC: public DFRobot_ICG20660L{
public:
/**
 * @brief The constructor of the ICG20660L sensor using IIC communication.
 * @param addr:  7-bit IIC address, controlled by SDO pin.
 * @n     IIC_ADDR_SDO_H or 0x69:  SDO pull high.(default)
 * @n     IIC_ADDR_SDO_L or 0x68:  SDO pull down.
 * @param pWire:   TwoWire class pointer.
 */
  DFRobot_ICG20660L_IIC(uint8_t addr = IIC_ADDR_SDO_H, TwoWire *pWire = &Wire);
  ~DFRobot_ICG20660L_IIC();
protected:
  int init();
  
  void writeReg(uint8_t reg, void *pBuf, size_t len);
  size_t readReg(uint8_t reg, void *pBuf, size_t len);
private:
  TwoWire *_pWire;
  uint8_t _addr;
  
};

class DFRobot_ICG20660L_SPI:public DFRobot_ICG20660L{
public:
/**
 * @brief The constructor of the ICG20660L sensor using SPI communication.
 * @param csPin:  SPI chip select pin, connected to IO pin of MCU.
 * @param spi: SPIClass class pointer.
 */
  DFRobot_ICG20660L_SPI(int csPin, SPIClass *spi);
  ~DFRobot_ICG20660L_SPI(){}
protected:
  int init();
  /**
   * @brief 通过SPI总线写入寄存器值
   * @param reg  寄存器地址 8bits
   * @param pBuf 要写入数据的存放缓存
   * @param size 要写入数据的长度
   * @return 返回实际读取的长度，返回0表示读取失败
   */
  void writeReg(uint8_t reg, void *pBuf, size_t len);
  /**
   * @brief 通过SPI总线读取寄存器值
   * @param reg  寄存器地址 8bits
   * @param pBuf 要写入数据的存放缓存
   * @param size 要写入数据的长度
   * @return 返回实际读取的长度，返回0表示读取失败
   */
  size_t readReg(uint8_t reg, void *pBuf, size_t len);
private:
  void setCSPinLow();
  void setCSPinHigh();
  SPIClass *_spi;
  uint8_t _cs;
  
};

// class DFRobot_ICG20660L_SPI:: public DFRobot_ICG20660L{
// public:

// };
#endif
