# DFRobot_18B20_RS485

- [English Version](./README.md)

这是一个基于Arduino平台的6轴陀螺仪库。ICG20660L包含3轴陀螺仪和3轴加速度，它支持两种接口：I2C(100~400KHz)和SPI(7MHz)通信，具有以下特点: <br>
* 加速度量程范围: ±2g、±4g、±8g、±16g, g = 9.80665 m/s².
* 陀螺仪量程范围: ±125dps、±250dps、±500dps, 1dps = Π/180° rad/s, Π = 3.1415926
* 支持512字节的FIFO
* 支持加速度唤醒检测.发生条件：如果前一次和后一次测得的加速度的阈值差大于或等于设定的阈值，将产生中断
* 加速度的X，Y,Z，温度，陀螺仪的x，Y，Z轴都可以单独使用和禁用，温度除外，必须和其中一轴数据一起采集。禁止某轴或温度可以降低功耗
* 支持进入低功耗睡眠模式，该模式下，陀螺仪，加速度，温度等传感器都将停止采样
* 加速度支持低功耗模式，在该模式下陀螺仪将禁止工作。
* IIC可通过SDO的高低电平来改变地址，为高0x69， 为低0x68。
* 支持从寄存器或FIFO读取数据，但是从FIFO读取必须使能加速度、陀螺仪、温度等所有功能，并将陀螺仪和加速度的内部采样设置成一样。


这里需要显示拍照图片，可以一张图片，可以多张图片（不要用SVG图）

![产品效果图](../../resources/images/SEN0443.png)


## 产品链接（[https://www.dfrobot.com.cn/goods-3336.html](https://www.dfrobot.com.cn/goods-3336.html)）
    SKU: SEN0443 

## 目录

  * [概述](#概述)
  * [连接](#连接)
  * [库安装](#库安装)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [创作者](#创作者)

## 概述

这是一个基于Arduino平台的6轴陀螺仪库，型号为ICG-20660L，它支持I2C和SPI通信，能够检测陀螺仪、加速度和板载温度。<br>

## 连接
### SPI连接
 Sensor      |                   raspberry pi                    |
------------ | :------------------------------------------------:|
FSY          | 不连接，浮空                                       |
INT          | 连接到树莓派的外部中断引脚                          |
CS           | 连接到树莓派的IO引脚                                |
SDO          | 连接到树莓派的miso引脚                             |
SDI          | 连接到树莓派的mosi引脚                             |
SCK          | 连接到树莓派的sck引脚                              |
GND          | 连接到树莓派的GND                                  |
3V3/VCC      | 连接到树莓派的3V3/VCC                              |
### IIC连接
 Sensor      |                   raspberry pi                    |
------------ | :------------------------------------------------:|
FSY          | 不连接，浮空                                      |
INT          | 连接到树莓派的外部中断引脚                          |
SDA          | 连接到树莓派的sda引脚                             |
SCL          | c连接到树莓派的scl引脚                            |
GND          | 连接到树莓派的GND                                  |
3V3/VCC      | 连接到树莓派的3V3/VCC                              |

## 库安装
1. 下载库至树莓派，要使用这个库，首先要将库下载到Raspberry Pi，命令下载方法如下:<br>
```python
sudo git clone https://github.com/DFRobot/DFRobot_ICG20660L
```
2. 打开并运行例程，要执行一个例程demo_x.py，请在命令行中输入python demo_x.py。例如，要执行demo_get_accel_data.py例程，你需要输入:<br>

```python
python demo_get_accel_data.py 
或 
python2 demo_get_accel_data.py 
或 
python3 demo_get_accel_data.py
```

## 方法

```python
  '''!
    @brief 初始化传感器，初始化后，所有传感器都被关闭，需通过enable_sensor打开相应的配置.
    @param mode: 配置读取传感器数据是从FIFO还是从寄存器。
    @n     eREG_MODE :   配置为从寄存器读取传感器数据
    @n     eFIFO_MODE:   从512字节FIFO读取数据,注意：从FIFO读取，加速度，陀螺仪、温度必须全部使能，且将其内部采样率必须配置成一致
    @return 初始化状态：:
    @n      0:  初始化成功
    @n     -1:  (I2C or SPI)接口初始化失败.
    @n     -2:  读取设备ID失败，（ID为0x91）
  '''
  def begin(self, mode = 0):
    
  '''!
    @brief 获取设备ID, 型号ICG20660L的设备ID是0x91(145).
    @return  如果设备型号为ICG20660L，将返回ID0x91(145)
  '''
  def read_id(self):
    
  '''!
    @brief 复位将寄存器恢复到初始配置值，调用该函数后，需要调用begin重新配置传感器才能正常工作。
  '''
  def reset(self):
    
  '''!
    @brief 进入睡眠模式，该模式下会降低功耗，陀螺仪和加速度将停止工作，需要调用wakeup函数去唤醒传感器。
  '''  
  def sleep(self):

  '''!
    @brief  从睡眠模式唤醒，将恢复睡眠前的配置。
  '''
  def wakeup(self):
  
  '''!
    @brief 失能传感器的功能，包含加速度的xyz轴、陀螺仪的xyz轴，以及温度。
    @param bit: 8位字节数据，每一位都代表使能一个功能位，如下表所示：
    @n -------------------------------------------------------------------------------------------------------------------
    @n |        bit7      |     bit6     |      bit5   |    bit4     |     bit3    |     bit2   |    bit1    |    bit0    |
    @n -------------------------------------------------------------------------------------------------------------------
    @n |     reserve     |    reserve    |eACCEL_AXIS_X|eACCEL_AXIS_Y|eACCEL_AXIS_Z|eGYRO_AXIS_X|eGYRO_AXIS_Y|eGYRO_AXIS_Z|
    @n |                                 |            eACCEL_AXIS_XYZ              |           eGYRO_AXIS_XYZ             |
    @n |                                 |                                eAXIS_ALL                                       |
    @n -------------------------------------------------------------------------------------------------------------------
    @n   bit0:  Z轴陀螺仪和温度.
    @n   bit1:  Y轴陀螺仪和温度.
    @n   bit2:  X轴陀螺仪和温度.
    @n   bit3:  Z轴加速度.
    @n   bit4:  Y轴加速度.
    @n   bit5:  X轴加速度.
    @n   bit6:  保留.
    @n   bit7:  保留.
    @note 使能陀螺仪的任意轴，都会自动使能传感器板载温度传感器。
    @n   eGYRO_AXIS_Z:  8位形参第0位置1，表示使能陀螺仪的的Z轴和温度。
    @n   eGYRO_AXIS_Y:  8位形参第1位置1，表示使能陀螺仪的的Y轴和温度。
    @n   eGYRO_AXIS_X:  8位形参第2位置1，表示使能陀螺仪的的X轴和温度。
    @n   eACCEL_AXIS_Z: 8位形参第3位置1，表示使能加速度的Z轴。
    @n   eACCEL_AXIS_Y: 8位形参第4位置1，表示使能加速度的Y轴。
    @n   eACCEL_AXIS_X: 8位形参第5位置1，表示使能加速度的X轴。
    @n   eGYRO_AXIS_XYZ or eGYRO_AXIS_X|eGYRO_AXIS_Y|eGYRO_AXIS_Z: 8位形参第0、1、2位全部置1, 表示使能陀螺仪的xyz轴和温度。
    @n   eACCEL_AXIS_XYZ or eACCEL_AXIS_X|eACCEL_AXIS_Y|eACCEL_AXIS_Z: 8位形参第3、4、5位全部置1, 表示使能加速度的xyz轴。
    @n   eAXIS_ALL or eGYRO_AXIS_Z|eGYRO_AXIS_Y|eGYRO_AXIS_X|eACCEL_AXIS_Z|eACCEL_AXIS_Y|eACCEL_AXIS_Z: 8位形参低7位全部置1,表示使陀螺仪和加速度的xyz轴和温度。
  '''
  def enable_sensor(self, bit):

  '''!
    @brief 失能传感器的功能，包含加速度的xyz轴、陀螺仪的xyz轴，以及温度。
    @param bit: 8位字节数据，每一位都代表使能一个功能位，如下表所示：
    @n -------------------------------------------------------------------------------------------------------------------
    @n |        bit7      |     bit6     |      bit5   |    bit4     |     bit3    |     bit2   |    bit1    |    bit0    |
    @n -------------------------------------------------------------------------------------------------------------------
    @n |     reserve     |    reserve    |eACCEL_AXIS_X|eACCEL_AXIS_Y|eACCEL_AXIS_Z|eGYRO_AXIS_X|eGYRO_AXIS_Y|eGYRO_AXIS_Z|
    @n |                                 |            eACCEL_AXIS_XYZ              |           eGYRO_AXIS_XYZ             |
    @n |                                 |                                eAXIS_ALL                                       |
    @n -------------------------------------------------------------------------------------------------------------------
    @n   bit0:  Z轴陀螺仪和温度.
    @n   bit1:  Y轴陀螺仪和温度.
    @n   bit2:  X轴陀螺仪和温度.
    @n   bit3:  Z轴加速度.
    @n   bit4:  Y轴加速度.
    @n   bit5:  X轴加速度.
    @n   bit6:  保留.
    @n   bit7:  保留.
    @note 只有当陀螺仪的X,Y,Z轴全部关闭的时候，才会关闭温度传感器，任意一轴开启，都不会关闭温度传感器
    @n   eGYRO_AXIS_Z:  8位形参第0位置1，表示失能陀螺仪的的Z轴。
    @n   eGYRO_AXIS_Y:  8位形参第1位置1，表示失能陀螺仪的的Y轴。
    @n   eGYRO_AXIS_X:  8位形参第2位置1，表示失能陀螺仪的的X轴。
    @n   eACCEL_AXIS_Z: 8位形参第3位置1，表示失能加速度的Z轴。
    @n   eACCEL_AXIS_Y: 8位形参第4位置1，表示失能加速度的Y轴。
    @n   eACCEL_AXIS_X: 8位形参第5位置1，表示失能加速度的X轴。
    @n   eGYRO_AXIS_XYZ or eGYRO_AXIS_X|eGYRO_AXIS_Y|eGYRO_AXIS_Z: 8位形参第0、1、2位全部置1, 表示失能陀螺仪的xyz轴和温度。
    @n   eACCEL_AXIS_XYZ or eACCEL_AXIS_X|eACCEL_AXIS_Y|eACCEL_AXIS_Z: 8位形参第3、4、5位全部置1, 表示失能加速度的xyz轴。
    @n   eAXIS_ALL or eGYRO_AXIS_Z|eGYRO_AXIS_Y|eGYRO_AXIS_X|eACCEL_AXIS_Z|eACCEL_AXIS_Y|eACCEL_AXIS_Z: 8位形参低7位全部置1,表示使陀螺仪和加速度的xyz轴和温度。
  '''
  def disable_sensor(self, bit):
  
  '''!
    @brief 配置加速度的满量程，dlpf带宽和内部采样率。
    @param scale  陀螺仪满量程, 单位: g(1g = 9.80665 m/s²).
    @n     eFSR_A_2G:  满量程范围 ±2g.
    @n     eFSR_A_4G:  满量程范围 ±4g.
    @n     eFSR_A_8G:  满量程范围 ±8g.
    @n     eFSR_A_16G: 满量程范围 ±16g.
    @param bd  设置 3-db 带宽.
    @n     eACCEL_DLPF_5_1KHZ or 0:    当信号小于或等于5Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
    @n     eACCEL_DLPF_10_1KHZ or 1:   当信号小于或等于10Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
    @n     eACCEL_DLPF_21_1KHZ or 2:   当信号小于或等于21Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
    @n     eACCEL_DLPF_44_1KHZ or 3:   当信号小于或等于44Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
    @n     eACCEL_DLPF_99_1KHZ or 4:   当信号小于或等于99Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
    @n     eACCEL_DLPF_218_1KHZ or 5:  当信号小于或等于218Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz，支持低功耗模式
    @n     eACCEL_DLPF_420_1KHZ or 6:  当信号小于或等于420Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz，支持低功耗模式
    @n     eACCEL_DLPF_1046_4KHZ or 7: 当信号小于或等于1046Hz时，会出现明显衰减，衰减3-db，内部采样率为4KHz，支持低功耗模式
    @n     eACCEL_DLPF_55_1KHZ or 8:   当信号小于或等于55Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz，仅支持低功耗模式
    @n     eACCEL_DLPF_110_1KHZ or 9:  当信号小于或等于110Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz，仅支持低功耗模式
    @n 注意：当陀螺仪和加速度都使能的时候，如果通过FIFO读取传感器数据，必须保证陀螺仪和加速度的内部采样率一致
    @param odr:  设置唤醒芯片以采样加速数据的频率 - 低功耗加速输出数据速率。
    @n     eODR_125HZ or 9:    低功耗加速度输出速率: 125Hz
    @n     eODR_250HZ or 10:   低功耗加速度输出速率: 250Hz
    @n     eODR_500HZ or 11:   低功耗加速度输出速率: 500Hz
    @param low_power_flag:  是否将加速配置为低功耗模式。
    @n     True:          进入低功耗模式.
    @n     False:         不将加速配置为低功耗模式。（默认）
  '''
  def config_accel(self,scale, bd, odr = 0, low_power_flag = False):

  '''!
    @brief 配置陀螺仪的满量程，dlpf带宽和内部采样率。 
    @param scale  陀螺仪满量程, 单位: dps(度每秒)
    @n     eFSR_G_125DPS:  满量程范围 ±125 dps.
    @n     eFSR_G_250DPS:  满量程范围 ±250 dps.
    @n     eFSR_G_500DPS:  满量程范围 ±500 dps.
    @param bd  Set 3-db bandwidth.
    @n     eGYRO_DLPF_8173_32KHZ:    当信号等于或大于8173Hz时，会出现明显衰减，衰减3-db，内部采样率为32KHz
    @n     eGYRO_DLPF_3281_32KHZ: 当信号等于或大于3281Hz时，会出现明显衰减，衰减3-db，内部采样率为32KHz
    @n     eGYRO_DLPF_250_8KHZ:     当信号等于或大于250Hz时，会出现明显衰减，衰减3-db，内部采样率为8KHz
    @n     eGYRO_DLPF_176_1KHZ:     当信号等于或大于176Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
    @n     eGYRO_DLPF_92_1KHZ:      当信号等于或大于92Hz时，会出现明显衰减，衰减3-db，内部采样率为1KHz
    @n     eGYRO_DLPF_3281_8KHZ:  当信号等于或大于3281Hz时，会出现明显衰减，衰减3-db，内部采样率为8KHz
    @note  当陀螺仪和加速度都使能的时候，如果通过FIFO读取传感器数据，必须保证陀螺仪和加速度的内部采样率一致
  '''
  def config_gyro(self, scale, bd):

  '''!
    @brief 设置采样分频. 
    @param div  采样分频系数, 范围0~255，采样率 = 内部采样率/(div+1).
    @n     采样率 = 内部采样率/(div+1)
    @note    如果加速度配置为低功耗模式，即configAccel函数的形参lowPowerFlag为true，则采样率必须和configAccel的形参odr输出率相匹配，如下表所示：
    @n ----------------------------------------------------------------------------
    @n |                        config_accel                      | set_sample_div |
    @n ----------------------------------------------------------------------------|
    @n |            bd             |      odr      | lowPowerFlag |      div       |
    @n ----------------------------------------------------------------------------|
    @n |            X              |       X       |    false     |      0~255     |
    @n ----------------------------------------------------------------------------|
    @n |                           |  eODR_125Hz   |    true      |        7       |
    @n |                           |-----------------------------------------------|
    @n |  支持低功耗模式的bd        |  eODR_250Hz   |    true      |        3       |
    @n |                           |-----------------------------------------------|
    @n |                           |  eODR_500Hz   |    true      |        1       |
    @n |---------------------------------------------------------------------------|
  '''
  def set_sample_div(self, div):

  '''!
    @brief 获取14字节原始数据，包括加速度、陀螺仪和温度。
    @param length: 返回列表的长度.
    @return data:  列表类型, 存放14字节原始数据的buffer。
    @n     第1字节数据   :  X轴加速度高字节数据
    @n     第2字节数据   :  X轴加速度低字节数据
    @n     第3字节数据   :  Y轴加速度高字节数据
    @n     第4字节数据   :  Y轴加速度低字节数据
    @n     第5字节数据   :  Z轴加速度高字节数据
    @n     第6字节数据   :  Z轴加速度低字节数据
    @n     第7字节数据   :  温度高字节数据.
    @n     第8字节数据   :  温度低字节数据.
    @n     第9字节数据   :  X轴陀螺仪高字节数据.
    @n     第10字节数据  :  X轴陀螺仪低字节数据.
    @n     第11字节数据  :  Y轴陀螺仪高字节数据.
    @n     第12字节数据  :  Y轴陀螺仪低字节数据.
    @n     第13字节数据  :  Z轴陀螺仪高字节数据..
    @n     第14字节数据  :  Z轴陀螺仪低字节数据.
    @note 可以使用 RAW_DATA_LENGTH 创造数据数组，可以使用   
    @n RAW_DATA_AX_H_INDEX, RAW_DATA_AX_L_INDEX, RAW_DATA_AY_H_INDEX, RAW_DATA_AY_L_INDEX, RAW_DATA_AZ_H_INDEX, RAW_DATA_AZ_L_INDEX,
    @n RAW_DATA_T_H_INDEX, RAW_DATA_T_L_INDEX,RAW_DATA_GX_H_INDEX, RAW_DATA_GX_L_INDEX, 
    @n RAW_DATA_GY_H_INDEX, RAW_DATA_GY_L_INDEX, RAW_DATA_GZ_H_INDEX, RAW_DATA_GZ_L_INDEX or 0~13 to 去索引这个数组.
  '''
  def get_raw_data(self, length = 0):
  
  '''!
    @brief 获取传感器的加速度，陀螺仪和温度数据。
    @return 字典格式: {'accel':{'x':0, 'y':0, 'z':0}, 'gyro':{'x':0, 'y':0, 'z':0}, 'temp':0.0}
  '''
  def get_sensor_data(self):
  
  '''!
    @brief 获取X轴加速度数据, 单位 g.
    @return  X 轴加速度.
  '''
  def get_accel_x(self):

  '''!
     @brief 获取Y轴加速度，单位 g.
     @return  Y 轴加速度.
  '''
  def get_accel_y(self):

  '''!
    @brief 获取Z轴加速度，单位 g.
    @return  Z轴加速度.
  '''
  def get_accel_z(self):
  
  '''!
    @brief 获取温度数据，单位: ℃.
    @return  温度数据.
  '''
  def get_temperature_c(self):
  
  '''!
     @brief 获取X轴陀螺仪数据，单位 dps.
     @return  X轴陀螺仪数据.
  '''
  def get_gyro_x(self):

  '''!
    @brief 获取Y轴陀螺仪数据，单位 dps.
    @return  Y轴陀螺仪速度.
  '''
  def get_gyro_y(self):

  '''!
    @brief 获取Z轴陀螺仪速度，单位 dps.
    @return Z轴陀螺仪速度.
  '''
  def get_gyro_z(self):

  '''!
    @brief 设置触发加速度传感器唤醒运动中断时，INT引脚的电平极性。
    @param polarity:  触发唤醒运动时，传感器INT引脚的电平信号。
    @n     GPIO.HIGH:  INT引脚初始信号为LOW，当产生加速度唤醒运动时，INT引脚电平信号将变为HIGH，需要调用read_int_status函数，才能清除该信号，重新恢复初始信号。
    @n     GPIO.LOW:   INT引脚初始信号为HIGH，当产生加速度唤醒运动时，INT引脚电平信号将变为LOW，需要调用read_int_status函数，才能清除该信号，重新恢复初始信号。
    @note  触发加速度唤醒运动后，如果不调用read_int_status函数清除该标志，INT引脚将一直保持触发运动时的电平极性。
  '''
  def set_int_pin_motion_trigger_polarity(self, polarity):

  '''!
    @brief 设置加速度的唤醒中断阈值. 
    @param level: WoM 阈值以固定的“mg”表示，独立于所选范围 [0g : 1g]； 分辨率 1g/256=~3.9mg，level = 0~255
    @return 实际 WoM 阈值，单位：g re_value = (level * 3.9)/1000 g
  '''
  def set_wake_on_motion_thread_for_accel(self, level):

  '''!
    @brief 当传感器INT引脚产生中断时，获取中断引脚的电平极性。
    @return INT 引脚触发中断时的电平信号。
    @n      GPIO.HIGH:  INT 引脚电平保持高电平直到中断状态被清除。
    @n      GPIO.LOW:   INT 引脚电平保持低电平直到中断状态被清除。
  '''
  def get_int_pin_motion_trigger_polarity(self):
  
  '''!
    @brief 读取中断状态寄存器，清除INT引脚的中断信号。
    @return 中断状态寄存器的值。
    @n  INT_STATUS register：addr:0x3A,acess:rw
    @n  ------------------------------------------------------------------------------------
    @n  |     b7    |    b6     |    b5     |      b4        | b3 | b2 | b1 |      b0      |
    @n  ------------------------------------------------------------------------------------
    @n  |             WOM_XYZ_INT           | FIFO_OFLOW_INT |      rsv     | DATA_RDY_INT |
    @n  ------------------------------------------------------------------------------------
    @n  DATA_RDY_INT  : 当产生数据就绪中断时，该位自动设置为 1。 读取寄存器后，该位清零。
    @n  rsv           : 保留
    @n  FIFO_OFLOW_INT: 当产生 FIFO 缓冲区溢出时，该位自动设置为 1。 读取寄存器后，该位清零。
    @n  WOM_XYZ_INT   : 当加速度计的 X 轴、Y 轴或 Z 轴触发 WOM（运动唤醒）中断时，这些位自动设置为非零数字。读取时清除。 
  '''
  def read_int_status(self):
  
```

## 兼容性

| 主板         | 通过 | 未通过 | 未测试 | 备注 |
| ------------ | :--: | :----: | :----: | :--: |
| RaspberryPi2 |      |        |   √    |      |
| RaspberryPi3 |      |        |   √    |      |
| RaspberryPi4 |  √   |        |        |      |

* Python 版本

| Python  | 通过 | 未通过 | 未测试 | 备注 |
| ------- | :--: | :----: | :----: | ---- |
| Python2 |  √   |        |        |      |
| Python3 |  √   |        |        |      |

## 历史

- 2021/06/01 - 1.0.0 版本

## 创作者

Written by Arya(xue.peng@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))






