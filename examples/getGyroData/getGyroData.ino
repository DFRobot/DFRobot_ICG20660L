/*!
 * @file getGyroData.ino
 * @brief 获取传感器的陀螺仪的x，y，z轴的数据(此demo不支持FIFO读取模式)，单位dps，并将其转换为角速度rad/s。
 *
 * @n connected table in SPI
 * -----------------------------------------------------------------------------------------------------
 *  sensor pin  |            MCU                    | ESP32 | ESP8266 |    M0   | micro:bit | Mega2560 |
 *    FSY       | not connected, floating           |   X   |    X    |    X    |     X     |     X    |
 *    INT       | not connected, floating           |   X   |    X    |    X    |     X     |     X    |
 *    CS        | connected to the IO pin of MCU    | 5/D8  |  5/D6   |    5    |     P8    |     5    |
 *    SDO       | connected to miso of mcu'spi      |19/MISO|  MISO   |  MISO   |  P14/MISO |  50/MISO |
 *    SDI       | connected to mosi of mcu'spi      |23/MOSI|  MOSI   |  MOSI   |  P15/MOSI |  51/MOSI |
 *    SCK       | connected to sck of mcu'spi       |18/SCK |  SCK    |  SCK    |  P13/SCK  |  52/SCK  |
 *    GND       | GND                               |  GND  |   GND   |   GND   |    GND    |    GND   |
 *    3V3/VCC   | 3V3/VCC                           |  3V3  |   3V3   |   3V3   |    3V3    |    5V    |
 * -----------------------------------------------------------------------------------------------------
 *
 * @n connected table in IIC
 * ---------------------------------------------------------------------------------------------------
 * sensor pin |            MCU                    | ESP32 | ESP8266 |    M0   | micro:bit | Mega2560 |
 *    FSY     | not connected, floating           |   X   |    X    |    X    |     X     |     X    |
 *    INT     | not connected, floating           |   X   |    X    |    X    |     X     |     X    |
 *    SDA     | connected to SDA of mcu'iic       | 21/SDA|   SDA   |   SDA   |  P20/SDA  |  20/SDA  |
 *    SCL     | connected to scl of mcu'iic       | 22/SCL|   SCL   |   SCL   |  P19/SCL  |  21/SCL  |
 *    GND     | GND                               |  GND  |   GND   |   GND   |    GND    |    GND   |
 *    3V3/VCC | 3V3/VCC                           |  3V3  |   3V3   |   3V3   |    3V3    |    5V    |
 * ---------------------------------------------------------------------------------------------------
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [Arya](xue.peng@dfrobot.com)
 * @version  V1.0
 * @data  2021-05-24
 * @get from https://www.dfrobot.com
 * @url https://github.com/DFRobot/DFRobot_ICG20660L
 */
#include "DFRobot_ICG20660L.h"

#ifdef ARDUINO_BBC_MICROBIT
#define CS_PIN      8                      //The CS pin of sensor which is connected to the 8 digital io pin of micro:bit,and also can connected to other pin.
#else
#define CS_PIN      5                      //The CS pin of sensor which is connected to the 5 digital io pin of MCU,and also can connected to other pin.
#endif
/**
 * @brief The constructor of the ICG20660L sensor using IIC communication.
 * @param addr:  7-bit IIC address, controlled by SDO pin.
 * @n     IIC_ADDR_SDO_H or 0x69:  SDO pull high.(default)
 * @n     IIC_ADDR_SDO_L or 0x68:  SDO pull down.
 * @param pWire:   TwoWire class pointer.
 */
DFRobot_ICG20660L_IIC icg(/*addr=*/IIC_ADDR_SDO_H, &Wire);
/**
 * @brief The constructor of the ICG20660L sensor using SPI communication.
 * @param csPin:  SPI chip select pin, connected to IO pin of MCU.
 * @param spi: SPIClass class pointer.
 */
//DFRobot_ICG20660L_SPI icg(/*csPin=*/CS_PIN, &SPI);

float DPS = 3.1415926/180.0; // unit: 1dps = 3.1415926/180.0rad/s
void setup() {
  Serial.begin(115200);
  while(!Serial){                                                     //Waiting for USB Serial COM port to open.
  }
  
  Serial.print("Initialization sensor...");
/**
 * @brief 初始化传感器，初始化后，所有传感器都被关闭，需通过enableSensor打开相应的配置. 
 * @param mode: Enum variable,from eDataReadMode_t,配置读取传感器数据是从FIFO还是从寄存器。
 * @n     eRegMode:  配置为从寄存器读取传感器数据
 * @n     eFIFOMode: 从512字节FIFO读取数据,注意：从FIFO读取，加速度，陀螺仪、温度必须全部使能，且将其内部采样率必须配置成一致。（此demo不支持）
 * @return status:
 * @n      0 :   Initialization sucess.
 * @n      -1:   Interface Initialization failed(IIC or SPI).
 * @n      -2:   读取设备ID失败，ID不是0x91
 */
  while(icg.begin(/*mode=*/icg.eRegMode) != 0){
      Serial.println("failed. Please check whether the hardware connection is wrong.");
      delay(1000);
      Serial.print("Initialization sensor...");
  }
  Serial.println("done.");
  
  Serial.print("ICG20660L Device ID: 0x");
  Serial.println(icg.readID(), HEX);
  
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
  icg.enableSensor(icg.eGyroAxisXYZ);
  //icg.enableSensor(icg.eGyroAxisX|icg.eGyroAxisY|icg.eGyroAxisZ);
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
  icg.configGyro(icg.eFSR_G_250DPS, icg.eGyro_DLPF_8173_32KHZ);
/**
 * @brief Set sample rate divider. 
 * @param div  Sample rate divider, the range is 0~255.
 * @n     采样率 = 内部采样率/(div+1)
 * @n Note: 如果陀螺仪配置为低功耗模式，即configAccel函数的形参lowPowerFlag为true，则采样率必须和configAccel的形参odr输出率相匹配，如下表所示：
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
  icg.setSampleDiv(19);
}

void loop() {
  sIcg20660SensorData_t  gyro;
  float t;
  gyro.x = icg.getGyroDataX();
  gyro.y = icg.getGyroDataY();
  gyro.z = icg.getGyroDataZ();
  t = icg.getTemperatureC();
  
  Serial.print("Gyro unit(dps): ");
  Serial.print("\tx: ");Serial.print(gyro.x);
  Serial.print(", \ty: ");Serial.print(gyro.y);
  Serial.print(", \tz: ");Serial.println(gyro.z);
  
  Serial.print("Gyro unit(rad/s): ");
  Serial.print("\tx: ");Serial.print(gyro.x*DPS);
  Serial.print(", \ty: ");Serial.print(gyro.y*DPS);
  Serial.print(", \tz: ");Serial.println(gyro.z*DPS);
  
  Serial.print("Temperature: \t");
  Serial.print(t);Serial.println("C\n");
  delay(1000);
}


