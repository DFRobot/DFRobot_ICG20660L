/*!
 * @file DFRobot_ICG20660L.h
 * @brief The ICG-20660 is a 6-axis motiontracking device that combines a 3-axis gyroscope and 3-axis accelerometer.
 * @n It supports two communication interfaces:
 * @n (1) IIC-->freq: 100~400Khz
 * @n (2) SPI-->freq: 100kHz~7MHz, only support mode0 or mode3
 * @n Two communication methods are switched by cs pin, 0:SPI, 1:IIC
 * @n 3-axis accelerometer feature:
 * @n (1) Support max ranging: ±2g, ±4g, ±8g, ±16g, g = 9.80665 m/s²
 * @n (2) 1g = 9.80665 m/s²
 * @n 3-axis gyroscope feature:
 * @n (1) Support max ranging: ±125dps, ±250dps, ±500dps
 * @n (2) 1dps = Π/180° rad/s, Π = 3.1415926
 * @n Motion threshold wake-up detection：
 * @n The motion threshold is the acceleration thresholds difference between the previous and next. If it is greater than or equal to the set threshold, an interrupt will be generated.
 * @n Support to read from register and FIFO
 * @n Read from FIFO. Accelerometer, gyroscope and temperature must all be enabled, and its internal sampling rate must be configured to be consistent.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [Arya](xue.peng@dfrobot.com)
 * @version  V1.0
 * @date  2021-05-25
 * @url https://github.com/DFRobot/DFRobot_ICG20660L
 */
#include <Arduino.h>
#include "DFRobot_ICG20660L.h"

#define REG_ICG20660L_SMPLRT_DIV     0x19  ///< SAMPLE RATE DIVIDER
#define REG_ICG20660L_ACCEL_CONFIG2  0x1D  ///< ACCELEROMETER CONFIGURATION 2
#define REG_ICG20660L_INT_PIN_CFG    0x37  ///< INT PIN / BYPASS ENABLE CONFIGURATION
#define REG_ICG20660L_INT_ENABLE     0x38  ///< INTERRUPT ENABLE
#define REG_ICG20660L_PWR_MGMT_1     0x6B  ///< POWER MANAGEMENT 1
#define REG_ICG20660L_PWR_MGMT_2     0x6C  ///< POWER MANAGEMENT 2
#define REG_ICG20660L_FIFO_COUNTH    0x72  ///< FIFO COUNT REGISTERS
#define REG_ICG20660L_FIFO_COUNTL    0x73  ///< FIFO COUNT REGISTERS
#define REG_ICG20660L_FIFO_R_W       0x74  ///< FIFO READ WRITE
 
#define REG_ICG20660L_SELF_TEST_X_GYRO              0x00  ///< GYROSCOPE SELF-TEST REGISTERS
#define REG_ICG20660L_SELF_TEST_Y_GYRO              0x01  ///< GYROSCOPE SELF-TEST REGISTERS
#define REG_ICG20660L_SELF_TEST_Z_GYRO              0x02  ///< GYROSCOPE SELF-TEST REGISTERS
#define REG_ICG20660L_XG_OFFS_TC_H                  0x04  ///< GYROSCOPE OFFSET TEMPERATURE COMPENSATION (TC) REGISTER
#define REG_ICG20660L_XG_OFFS_TC_L                  0x05  ///< GYROSCOPE OFFSET TEMPERATURE COMPENSATION (TC) REGISTER
#define REG_ICG20660L_YG_OFFS_TC_H                  0x07  ///< GYROSCOPE OFFSET TEMPERATURE COMPENSATION (TC) REGISTER
#define REG_ICG20660L_YG_OFFS_TC_L                  0x08  ///< GYROSCOPE OFFSET TEMPERATURE COMPENSATION (TC) REGISTER
#define REG_ICG20660L_ZG_OFFS_TC_H                  0x0A  ///< GYROSCOPE OFFSET TEMPERATURE COMPENSATION (TC) REGISTER
#define REG_ICG20660L_ZG_OFFS_TC_L                  0x0B  ///< IGYROSCOPE OFFSET TEMPERATURE COMPENSATION (TC) REGISTER
#define REG_ICG20660L_SELF_TEST_X_ACCEL             0x0D  ///< ACCELEROMETER SELF-TEST REGISTERS
#define REG_ICG20660L_SELF_TEST_Y_ACCEL             0x0E  ///< ACCELEROMETER SELF-TEST REGISTERS
#define REG_ICG20660L_SELF_TEST_Z_ACCEL             0x0F  ///< ACCELEROMETER SELF-TEST REGISTERS
#define REG_ICG20660L_XG_OFFS_USRH                  0x13  ///< GYRO OFFSET ADJUSTMENT REGISTER
#define REG_ICG20660L_XG_OFFS_USRL                  0x14  ///< GYRO OFFSET ADJUSTMENT REGISTER
#define REG_ICG20660L_YG_OFFS_USRH                  0x15  ///< GYRO OFFSET ADJUSTMENT REGISTER
#define REG_ICG20660L_YG_OFFS_USRL                  0x16  ///< GYRO OFFSET ADJUSTMENT REGISTER
#define REG_ICG20660L_ZG_OFFS_USRH                  0x17  ///< GYRO OFFSET ADJUSTMENT REGISTER
#define REG_ICG20660L_ZG_OFFS_USRL                  0x18  ///< GYRO OFFSET ADJUSTMENT REGISTER
#define REG_ICG20660L_CONFIG                        0x1A  ///< CONFIGURATION
#define REG_ICG20660L_GYRO_CONFIG                   0x1B  ///< GYROSCOPE CONFIGURATION
#define REG_ICG20660L_ACCEL_CONFIG                  0x1C  ///< ACCELEROMETER CONFIGURATION
#define REG_ICG20660L_ACCEL_CONFIG2                 0x1D  ///< ACCELEROMETER CONFIGURATION 2
#define REG_ICG20660L_LP_MODE_CFG                   0x1E  ///< LOW POWER MODE CONFIGURATION
#define REG_ICG20660L_ACCEL_WOM_THR                 0x1F  ///< WAKE-ON MOTION THRESHOLD (ACCELEROMETER)
#define REG_ICG20660L_FIFO_EN                       0x23  ///< FIFO ENABLE
#define REG_ICG20660L_FSYNC_INT                     0x36  ///< FSYNC INTERRUPT STATUS
#define REG_ICG20660L_INT_PIN_CFG                   0x37  ///< INT PIN / BYPASS ENABLE CONFIGURATION
#define REG_ICG20660L_INT_STATUS                    0x3A  ///< INTERRUPT STATUS
#define REG_ICG20660L_ACCEL_XOUT_H                  0x3B  ///< ACCELEROMETER MEASUREMENTS
#define REG_ICG20660L_ACCEL_XOUT_L                  0x3C  ///< ACCELEROMETER MEASUREMENTS
#define REG_ICG20660L_ACCEL_YOUT_H                  0x3D  ///< ACCELEROMETER MEASUREMENTS
#define REG_ICG20660L_ACCEL_YOUT_L                  0x3E  ///< ACCELEROMETER MEASUREMENTS
#define REG_ICG20660L_ACCEL_ZOUT_H                  0x3F  ///< ACCELEROMETER MEASUREMENTS
#define REG_ICG20660L_ACCEL_ZOUT_L                  0x40  ///< ACCELEROMETER MEASUREMENTS
#define REG_ICG20660L_TEMP_OUT_H                    0x41  ///< TEMPERATURE MEASUREMENT
#define REG_ICG20660L_TEMP_OUT_L                    0x42  ///< TEMPERATURE MEASUREMENT
#define REG_ICG20660L_GYRO_XOUT_H                   0x43  ///< GYROSCOPE MEASUREMENTS
#define REG_ICG20660L_GYRO_XOUT_L                   0x44  ///< GYROSCOPE MEASUREMENTS
#define REG_ICG20660L_GYRO_YOUT_H                   0x45  ///< GYROSCOPE MEASUREMENTS
#define REG_ICG20660L_GYRO_YOUT_L                   0x46  ///< GYROSCOPE MEASUREMENTS
#define REG_ICG20660L_GYRO_ZOUT_H                   0x47  ///< GYROSCOPE MEASUREMENTS
#define REG_ICG20660L_GYRO_ZOUT_L                   0x48  ///< GYROSCOPE MEASUREMENTS
#define REG_ICG20660L_SIGNAL_PATH_RESET             0x68  ///< SIGNAL PATH RESET
#define REG_ICG20660L_ACCEL_INTEL_CTRL              0x69  ///< ACCELEROMETER INTELLIGENCE CONTROL
#define REG_ICG20660L_USER_CTRL                     0x6A  ///< USER CONTROL
#define REG_ICG20660L_WHO_AM_I                      0x75  ///< WHO AM I
#define REG_ICG20660L_XA_OFFSET_H                   0x77  ///< ACCELEROMETER OFFSET REGISTERS
#define REG_ICG20660L_XA_OFFSET_L                   0x78  ///< ACCELEROMETER OFFSET REGISTERS
#define REG_ICG20660L_YA_OFFSET_H                   0x7A  ///< ACCELEROMETER OFFSET REGISTERS 
#define REG_ICG20660L_YA_OFFSET_L                   0x7B  ///< ACCELEROMETER OFFSET REGISTERS 
#define REG_ICG20660L_ZA_OFFSET_H                   0x7D  ///< ACCELEROMETER OFFSET REGISTERS 
#define REG_ICG20660L_ZA_OFFSET_L                   0x7E  ///< ACCELEROMETER OFFSET REGISTERS 

DFRobot_ICG20660L::DFRobot_ICG20660L(){
  _gyroScale = ADC_MAX_RANGE/GYRO_FULL_SCALE_500DPS;
  _accelScale = ADC_MAX_RANGE/ACCEL_FULL_SCALE_16G;
  _gyroRange = GYRO_FULL_SCALE_500DPS;
  _accelRange = ACCEL_FULL_SCALE_16G;
  _mode = eSixAxisLowNoiseMode;
  _dataMode = eRegMode;
  _fifoFrameSize = 14;
  _level = LOW;
  memset(&_rawData, 0, RAW_DATA_LENGTH);
  _update = 0;
}
DFRobot_ICG20660L::~DFRobot_ICG20660L(){
  
}


int DFRobot_ICG20660L::begin(eDataReadMode_t mode){
  if (init() != 0){
      return -1;
  }
  _dataMode = mode;
  _mode = eSleepMode;
  reset();
  delay(1000);
  
  outSleepMode();
  delay(1000);
  selectClockSource(CLOCK_SEL_PLL); //select clock source.
  _mode = eSixAxisLowNoiseMode;
 
  if(readID() != ICG20660L_DEVICE_ID){ // check the device ID, expected value is 0x91 (decimal 145)
      return -2;
  }
  disableSensor(eAxisAll);
  configAccel(eFSR_A_16G, eAccel_DLPF_218_1KHZ);
  configGyro(eFSR_G_500DPS, eGyro_DLPF_176_1KHZ);
  return 0;
}

uint8_t DFRobot_ICG20660L::readID(){
  uint8_t val;
  readReg(REG_ICG20660L_WHO_AM_I, &val, 1);
  DBGREG("REG_ICG20660L_WHO_AM_I", REG_ICG20660L_WHO_AM_I, val);
  return val;
}


void DFRobot_ICG20660L::enableSensor(uint8_t bit){
  uSensorEnable_t  sensor;
  sensor.value = (uint8_t)(bit&0x7F);
  bool temp, gz,gy, gx, a;
  uint8_t val1,val2;
  
  eFifoEnReg_t fifo;
  uPowerManagement1Reg_t reg1;
  uPowerManagement2Reg_t reg;
  readReg(REG_ICG20660L_FIFO_EN, &fifo, 1);
  readReg(REG_ICG20660L_PWR_MGMT_1, &reg1, 1);
  readReg(REG_ICG20660L_PWR_MGMT_2, &reg, 1);

  reg1.tempDis = (sensor.bitTemp)   ? 0 : reg1.tempDis;
  reg.STBY_ZG =  (sensor.bitGyroZ)  ? 0 : reg.STBY_ZG;
  reg.STBY_YG =  (sensor.bitGyroY)  ? 0 : reg.STBY_YG;
  reg.STBY_XG =  (sensor.bitGyroX) ? 0 : reg.STBY_XG;
  reg.STBY_ZA =  (sensor.bitAccelZ) ? 0 : reg.STBY_ZA;
  reg.STBY_YA =  (sensor.bitAccelY) ? 0 : reg.STBY_YA;
  reg.STBY_XA =  (sensor.bitAccelX) ? 0 : reg.STBY_XA;
  reg.FIFO_LP_EN = 0;
  temp = (sensor.bitTemp) ? true : fifo.TEMP_FIFO_EN;
  gz   = (sensor.bitGyroZ) ? true : fifo.ZG_FIFO_EN;
  gy   = (sensor.bitGyroY) ? true : fifo.YG_FIFO_EN;
  gx   = (sensor.bitGyroX) ? true : fifo.XG_FIFO_EN;
  a    = (sensor.bitAccelZ | sensor.bitAccelY | sensor.bitAccelX) ? true : fifo.ACCEL_FIFO_EN;

  if(_dataMode == eFIFOMode){
      enableFifo(temp, gx, gy, gz, a);
  }else{
      enableFifo(false, false, false, false, false);
  }
  
  writeReg(REG_ICG20660L_PWR_MGMT_2, &reg, 1);
  writeReg(REG_ICG20660L_PWR_MGMT_1, &reg1, 1);
}


void DFRobot_ICG20660L::disableSensor(uint8_t bit){
  uSensorEnable_t  sensor;
  sensor.value = (uint8_t)(bit&0x7F);
  bool temp, gz,gy, gx, a;
  
  eFifoEnReg_t fifo;
  uPowerManagement1Reg_t reg1;
  uPowerManagement2Reg_t reg;
  readReg(REG_ICG20660L_FIFO_EN, &fifo, 1);
  readReg(REG_ICG20660L_PWR_MGMT_1, &reg1, 1);
  readReg(REG_ICG20660L_PWR_MGMT_2, &reg, 1);
  
  reg1.tempDis = (sensor.bitTemp)   ? 1 : reg1.tempDis;
  reg.STBY_ZG =  (sensor.bitGyroZ)  ? 1 : reg.STBY_ZG;
  reg.STBY_YG =  (sensor.bitGyroY)  ? 1 : reg.STBY_YG;
  reg.STBY_XG =  (sensor.bitGyroX)  ? 1 : reg.STBY_XG;
  reg.STBY_ZA =  (sensor.bitAccelZ) ? 1 : reg.STBY_ZA;
  reg.STBY_YA =  (sensor.bitAccelY) ? 1 : reg.STBY_YA;
  reg.STBY_XA =  (sensor.bitAccelX) ? 1 : reg.STBY_XA;
  reg.FIFO_LP_EN = 0;
  temp = (sensor.bitTemp) ? true : fifo.TEMP_FIFO_EN;
  gz   = (sensor.bitGyroZ) ? true : fifo.ZG_FIFO_EN;
  gy   = (sensor.bitGyroY) ? true : fifo.YG_FIFO_EN;
  gx   = (sensor.bitGyroX) ? true : fifo.XG_FIFO_EN;
  a    = (sensor.bitAccelZ | sensor.bitAccelY | sensor.bitAccelX) ? true : fifo.ACCEL_FIFO_EN;
  reg1.tempDis = (reg.STBY_ZG | reg.STBY_YG | reg.STBY_ZG) ? 0 : reg1.tempDis;
  temp = (reg.STBY_ZG | reg.STBY_YG | reg.STBY_ZG) ? true : temp;

  if(_dataMode == eFIFOMode){
      enableFifo(temp, gx, gy, gz, a);
  }else{
      enableFifo(false, false, false, false, false);
  }
  writeReg(REG_ICG20660L_PWR_MGMT_2, &reg, 1);
  writeReg(REG_ICG20660L_PWR_MGMT_1, &reg1, 1);
}

void DFRobot_ICG20660L::configGyro(eGyroFSR_t scale, eGyroBandwidth_t  bd){
  configGyro((uint8_t)scale, (uint8_t)bd);
}
void DFRobot_ICG20660L::configGyro(uint8_t scale, uint8_t  bd){
  setFullScaleForGyro(scale);
  setBandwidthForGyro((eGyroBandwidth_t)bd);
}

void DFRobot_ICG20660L::configAccel(eAccelFSR_t scale, eAccelBandwidth_t bd, eODR_t odr, bool lowPowerFlag){
  configAccel((uint8_t)scale, (uint8_t)bd, (uint8_t)odr, lowPowerFlag);
}

void DFRobot_ICG20660L::configAccel(uint8_t scale, uint8_t bd, uint8_t odr, bool lowPowerFlag){
  if(lowPowerFlag){
      _mode = eAccelLowPowerMode;
  }else{
      _mode = eSixAxisLowNoiseMode;
  }
  setFullScaleForAccel(scale);
  setBandwidthForAccel((eAccelBandwidth_t)bd);
  writeReg(REG_ICG20660L_LP_MODE_CFG, &odr, 1);
}

void DFRobot_ICG20660L::setSampleDiv(uint8_t div){
  writeReg(REG_ICG20660L_SMPLRT_DIV, &div, 1);
}

void DFRobot_ICG20660L::sleep(){
  uPowerManagement1Reg_t pReg;
  readReg(REG_ICG20660L_PWR_MGMT_1, &pReg, 1);
  pReg.sleep = 1;
  writeReg(REG_ICG20660L_PWR_MGMT_1, &pReg, 1);
  delay(100);
}

void DFRobot_ICG20660L::wakeup(){
  uPowerManagement1Reg_t pReg;
  readReg(REG_ICG20660L_PWR_MGMT_1, &pReg, 1);
  pReg.sleep = 0;
  writeReg(REG_ICG20660L_PWR_MGMT_1, &pReg, 1);
  delay(1000);
}

void DFRobot_ICG20660L::reset(){
  uPowerManagement1Reg_t reg;
  int waitForTimeOutMs = 100;
  int waitForTimeOutInc = 5;
  readReg(REG_ICG20660L_PWR_MGMT_1, &reg, 1);
  DBGREG("before: REG_ICG20660L_PWR_MGMT_1", REG_ICG20660L_PWR_MGMT_1, reg.value);
  reg.deviceReset = 1;
  writeReg(REG_ICG20660L_PWR_MGMT_1, &reg, 1);
  for(int i = 0; i < waitForTimeOutMs; i++){
      delay(5);
      readReg(REG_ICG20660L_PWR_MGMT_1, &reg, 1);
      if (reg.deviceReset == 0) break;
  }
  DBGREG("after: REG_ICG20660L_PWR_MGMT_1", REG_ICG20660L_PWR_MGMT_1, reg.value);
}

void DFRobot_ICG20660L::setINTPinMotionTriggerPolarity(int polarity){
  uAccelConfig2Reg_t aclCfg2;
  readReg(REG_ICG20660L_ACCEL_CONFIG2, &aclCfg2, 1);
  aclCfg2.A_DLPF_CFG = 2;
  writeReg(REG_ICG20660L_ACCEL_CONFIG2, &aclCfg2, 1);
  _level = polarity;
  uIntPinCfgReg_t pinReg;
  readReg(REG_ICG20660L_INT_PIN_CFG, &pinReg, 1);
  if (polarity){
      pinReg.INT_LEVEL = 0;
  }else{
      pinReg.INT_LEVEL = 1;
  }
  pinReg.INT_OPEN = 0;
  pinReg.LATCH_INT_EN = 1;
  writeReg(REG_ICG20660L_INT_PIN_CFG, &pinReg, 1);
  
  //enable motion irq
  uIntEnableReg_t irqReg;
  readReg(REG_ICG20660L_INT_ENABLE, &irqReg, 1);
  irqReg.WOM_EN = 7;
  irqReg.DATA_RDY_INT_EN = 0;
  writeReg(REG_ICG20660L_INT_ENABLE, &irqReg, 1);
  delay(1000);
  uAccelIntelCtrlReg_t itel;
  uint8_t val;
  readReg(REG_ICG20660L_ACCEL_INTEL_CTRL, &itel, 1);
  DBGREG("before REG_ICG20660L_ACCEL_INTEL_CTRL = ", REG_ICG20660L_ACCEL_INTEL_CTRL, itel.value);
  itel.ACCEL_INTEL_EN = 1;
  itel.WOM_TH_MODE = 1;
  itel.ACCEL_INTEL_MODE = 1;
  for(int i = 0; i < 5; i++){
      delay(10);
      writeReg(REG_ICG20660L_ACCEL_INTEL_CTRL, &itel, 1);
      delay(10);
      readReg(REG_ICG20660L_ACCEL_INTEL_CTRL, &val, 1);
      if(val == itel.value) break;
  }
  
  readReg(REG_ICG20660L_ACCEL_INTEL_CTRL, &val, 1);
  //Serial.print("val=");Serial.println(val,HEX);
  //Serial.print("itel=");Serial.println(itel.value,HEX);
  DBGREG("after REG_ICG20660L_ACCEL_INTEL_CTRL = ", REG_ICG20660L_ACCEL_INTEL_CTRL, itel.value);
}

float DFRobot_ICG20660L::setWakeOnMotionThresholdForAccel(uint8_t level){
  float g = (level *  3.9)/1000.0;
  //DBGREG("after level = ", level, level);
  uint8_t level1;
  for(int i = 0; i < 5; i++){
      writeReg(REG_ICG20660L_ACCEL_WOM_THR,&level, 1);
      delay(2);
      readReg(REG_ICG20660L_ACCEL_WOM_THR,&level1, 1);
      if(level1 == level) break;
  }
  
  readReg(REG_ICG20660L_ACCEL_WOM_THR,&level1, 1);
  //Serial.print("level1=");Serial.println(level1);
  //DBGREG("after REG_ICG20660L_ACCEL_WOM_THR = ", REG_ICG20660L_ACCEL_WOM_THR, level);
  return g;
}

uint8_t DFRobot_ICG20660L::readINTStatus(){
  uIntStatusReg_t reg;
  readReg(REG_ICG20660L_INT_STATUS, &reg, 1);
  DBGREG("WOM_X_INT = ", REG_ICG20660L_INT_STATUS, reg.WOM_X_INT);
  DBGREG("WOM_Y_INT = ", REG_ICG20660L_INT_STATUS, reg.WOM_Y_INT);
  DBGREG("WOM_Z_INT = ", REG_ICG20660L_INT_STATUS, reg.WOM_Z_INT);
  DBGREG("FIFO_OFLOW_INT = ", REG_ICG20660L_INT_STATUS, reg.FIFO_OFLOW_INT);
  DBGREG("DATA_RDY_INT = ", REG_ICG20660L_INT_STATUS, reg.DATA_RDY_INT);
  DBGREG("DATA_RDY_INT vale ", REG_ICG20660L_INT_STATUS, reg.value);
  return reg.value;
  
}

int DFRobot_ICG20660L::getINTPinMotionTriggerPolarity(){
  return _level;
}


void DFRobot_ICG20660L::getSensorData(sIcg20660SensorData_t *accel, sIcg20660SensorData_t *gyro, float *t){
  getRawData(NULL);
  if(accel != NULL){
      accel->x = ((int16_t)(_rawData[RAW_DATA_AX_H_INDEX] << 8 | _rawData[RAW_DATA_AX_L_INDEX]))/_accelScale;
      accel->y = ((int16_t)(_rawData[RAW_DATA_AY_H_INDEX] << 8 | _rawData[RAW_DATA_AY_L_INDEX]))/_accelScale;
      accel->z = ((int16_t)(_rawData[RAW_DATA_AZ_H_INDEX] << 8 | _rawData[RAW_DATA_AZ_L_INDEX]))/_accelScale;
  }
  
  if(gyro != NULL){
      gyro->x = ((int16_t)(_rawData[RAW_DATA_GX_H_INDEX] << 8 | _rawData[RAW_DATA_GX_L_INDEX]))/_gyroScale;
      gyro->y = ((int16_t)(_rawData[RAW_DATA_GY_H_INDEX] << 8 | _rawData[RAW_DATA_GY_L_INDEX]))/_gyroScale;
      gyro->z = ((int16_t)(_rawData[RAW_DATA_GZ_H_INDEX] << 8 | _rawData[RAW_DATA_GZ_L_INDEX]))/_gyroScale;
  }
  
  if(t != NULL){
      *t = ((int16_t)(_rawData[RAW_DATA_T_H_INDEX] << 8 | _rawData[RAW_DATA_T_L_INDEX]))/326.8 + 23;
  }
  _update = 0;
}

float DFRobot_ICG20660L::getAccelDataX(){
  if(!(_update & 0x01)){
      getRawData(NULL);
  }
  _update &= 0xFE;
  return ((int16_t)(_rawData[RAW_DATA_AX_H_INDEX] << 8 | _rawData[RAW_DATA_AX_L_INDEX]))/_accelScale;
}
float DFRobot_ICG20660L::getAccelDataY(){
  if(!(_update & 0x02)){
      getRawData(NULL);
  }
  _update &= 0xFD;
  return ((int16_t)(_rawData[RAW_DATA_AY_H_INDEX] << 8 | _rawData[RAW_DATA_AY_L_INDEX]))/_accelScale;
}
float DFRobot_ICG20660L::getAccelDataZ(){
  if(!(_update & 0x04)){
      getRawData(NULL);
  }
  _update &= 0xFB;
  return ((int16_t)(_rawData[RAW_DATA_AZ_H_INDEX] << 8 | _rawData[RAW_DATA_AZ_L_INDEX]))/_accelScale;
}

float DFRobot_ICG20660L::getTemperatureC(){
  if(!(_update & 0x08)){
      getRawData(NULL);
  }
  _update &= 0xF7;
  return ((int16_t)(_rawData[RAW_DATA_T_H_INDEX] << 8 | _rawData[RAW_DATA_T_L_INDEX]))/326.8 + 23;
}

float DFRobot_ICG20660L::getGyroDataX(){
  if(!(_update & 0x10)){
      getRawData(NULL);
  }
  _update &= 0xEF;
  return ((int16_t)(_rawData[RAW_DATA_GX_H_INDEX] << 8 | _rawData[RAW_DATA_GX_L_INDEX]))/_gyroScale;
}
float DFRobot_ICG20660L::getGyroDataY(){
 if(!(_update & 0x20)){
      getRawData(NULL);
  }
  _update &= 0xDF;
  return ((int16_t)(_rawData[RAW_DATA_GY_H_INDEX] << 8 | _rawData[RAW_DATA_GY_L_INDEX]))/_gyroScale;
}
float DFRobot_ICG20660L::getGyroDataZ(){
   if(!(_update & 0x40)){
      getRawData(NULL);
  }
  _update &= 0xBF;
  return ((int16_t)(_rawData[RAW_DATA_GZ_H_INDEX] << 8 | _rawData[RAW_DATA_GZ_L_INDEX]))/_gyroScale;
}

void DFRobot_ICG20660L::getRawData(uint8_t *data, uint8_t len){
  memset(_rawData, 0, RAW_DATA_LENGTH);
  if(_dataMode == eFIFOMode){
      readDataFromFIFO();
  }else{
      readDataFromREG();
  }
  if(data != NULL){
      memcpy(data, _rawData, len);
  }
  _update = 0x7F;
}

void DFRobot_ICG20660L::setBandwidthForAccel(eAccelBandwidth_t  bd){
  uPowerManagement1Reg_t pReg;
  readReg(REG_ICG20660L_PWR_MGMT_1, &pReg, 1);
  switch((uint8_t)_mode){
      case eSleepMode:
      case eStandbyMode:
           break;
      case eAccelLowPowerMode:
      case eAccelLowNoiseMode:
           pReg.cycle = 1;
           setBandwidthForAccelInLowPowerMode(bd);
           break;
      case eSixAxisLowNoiseMode:
           pReg.cycle = 0;
           pReg.gyroStandby = 0;
           setBandwidthForAccelInOthersMode(bd);
           break;
  }
   writeReg(REG_ICG20660L_PWR_MGMT_1, &pReg, 1);
}

void DFRobot_ICG20660L::setBandwidthForGyro(eGyroBandwidth_t  bd){
  uGyroConfigReg_t gyroReg;
  uConfigReg_t reg;
  readReg(REG_ICG20660L_GYRO_CONFIG, &gyroReg, 1);
  readReg(REG_ICG20660L_CONFIG, &reg, 1);
  switch((uint8_t)bd){
      case eGyro_DLPF_8173_32KHZ:
           gyroReg.FCHOICE_B = 1;
           break;
      case eGyro_DLPF_3281_32KHZ:
           gyroReg.FCHOICE_B = 2;
           break;
      case eGyro_DLPF_250_8KHZ:
           gyroReg.FCHOICE_B = 0;
           reg.DLPF_CFG = 0;
           break;
      case eGyro_DLPF_176_1KHZ:
           gyroReg.FCHOICE_B = 0;
           reg.DLPF_CFG = 1;
           break;
      case eGyro_DLPF_92_1KHZ:
           gyroReg.FCHOICE_B = 0;
           reg.DLPF_CFG = 2;
           break;
      case eGyro_DLPF_3281_8KHZ:
           gyroReg.FCHOICE_B = 0;
           reg.DLPF_CFG = 7;
           break;
  }
  writeReg(REG_ICG20660L_GYRO_CONFIG, &gyroReg, 1);
  writeReg(REG_ICG20660L_CONFIG, &reg, 1);
}



void DFRobot_ICG20660L::setFullScaleForGyro(uint8_t scale){
  scale &= 0x03;
  uGyroConfigReg_t reg;
  readReg(REG_ICG20660L_GYRO_CONFIG, &reg, 1);
  DBGREG("before: REG_ICG20660L_GYRO_CONFIG", REG_ICG20660L_GYRO_CONFIG, reg.value);
  reg.FS_SEL = scale;
  reg.XG_ST = 0;
  reg.YG_ST = 0;
  reg.ZG_ST = 0;
  writeReg(REG_ICG20660L_GYRO_CONFIG, &reg, 1);
  readReg(REG_ICG20660L_GYRO_CONFIG, &reg, 1);
  DBGREG("after: REG_ICG20660L_GYRO_CONFIG", REG_ICG20660L_GYRO_CONFIG, reg.value);
  switch(scale){
      case eFSR_G_125DPS:
        _gyroRange = GYRO_FULL_SCALE_125DPS;
        break;
      case eFSR_G_250DPS:
        _gyroRange = GYRO_FULL_SCALE_250DPS;
        break;
      case eFSR_G_500DPS:
        _gyroRange = GYRO_FULL_SCALE_500DPS;
        break;
  }
  _gyroScale = ADC_MAX_RANGE/_gyroRange;
}

void DFRobot_ICG20660L::setFullScaleForAccel(uint8_t scale){
  uAccelConfigReg_t reg;
  scale &= 0x03;
  readReg(REG_ICG20660L_ACCEL_CONFIG, &reg, 1);
  DBGREG("before: REG_ICG20660L_ACCEL_CONFIG", REG_ICG20660L_ACCEL_CONFIG, reg.value);
  reg.ACCEL_FS_SEL = scale;
  reg.XA_ST = 0;
  reg.YA_ST = 0;
  reg.ZA_ST = 0;
  writeReg(REG_ICG20660L_ACCEL_CONFIG, &reg, 1);
  readReg(REG_ICG20660L_ACCEL_CONFIG, &reg, 1);
  DBGREG("after: REG_ICG20660L_ACCEL_CONFIG", REG_ICG20660L_ACCEL_CONFIG, reg.value);
  switch(scale){
      case eFSR_A_2G:
        _accelScale = ADC_MAX_RANGE/ACCEL_FULL_SCALE_2G;
        _accelRange = ACCEL_FULL_SCALE_2G;
        break;
      case eFSR_A_4G:
        _accelScale = ADC_MAX_RANGE/ACCEL_FULL_SCALE_4G;
        _accelRange = ACCEL_FULL_SCALE_4G;
        break;
      case eFSR_A_8G:
        _accelScale = ADC_MAX_RANGE/ACCEL_FULL_SCALE_8G;
        _accelRange = ACCEL_FULL_SCALE_8G;
        break;
      case eFSR_A_16G:
        _accelScale = ADC_MAX_RANGE/ACCEL_FULL_SCALE_16G;
        _accelRange = ACCEL_FULL_SCALE_16G;
        break;
  }
}

void DFRobot_ICG20660L::readDataFromREG(){
  uint8_t value[14] = {0};
  readReg(REG_ICG20660L_ACCEL_XOUT_H, &value, 14);
  
  // for(int i = 0; i < 14; i++){
      // if(value[i] < 16){
          // Serial.print("0");
      // }
      // Serial.print(value[i], HEX);
      // Serial.print(", ");
  // }
  // Serial.println();

  _rawData[RAW_DATA_AX_H_INDEX] = value[0];
  _rawData[RAW_DATA_AX_L_INDEX] = value[1];
  _rawData[RAW_DATA_AY_H_INDEX] = value[2];
  _rawData[RAW_DATA_AY_L_INDEX] = value[3];
  _rawData[RAW_DATA_AZ_H_INDEX] = value[4];
  _rawData[RAW_DATA_AZ_L_INDEX] = value[5];
  _rawData[RAW_DATA_T_H_INDEX] =  value[6];
  _rawData[RAW_DATA_T_L_INDEX] =  value[7];
  _rawData[RAW_DATA_GX_H_INDEX] = value[8];
  _rawData[RAW_DATA_GX_L_INDEX] = value[9];
  _rawData[RAW_DATA_GY_H_INDEX] = value[10];
  _rawData[RAW_DATA_GY_L_INDEX] = value[11];
  _rawData[RAW_DATA_GZ_H_INDEX] = value[12];
  _rawData[RAW_DATA_GZ_L_INDEX] = value[13];
}

void DFRobot_ICG20660L::readDataFromFIFO(){
  eFifoEnReg_t reg;
  uint8_t val[2] = {0};
  uint8_t rslt[_fifoFrameSize];
  uint16_t size = 0;
  uint8_t count = 0;
  
  memset(rslt, 0, sizeof(rslt));
  readReg(REG_ICG20660L_FIFO_EN, &reg, 1);
  
  readReg(REG_ICG20660L_FIFO_COUNTH, val, 2);
  size = ((val[0] & 0x1F) << 8) | val[1];

  if(size >= _fifoFrameSize){
      readReg(REG_ICG20660L_FIFO_R_W, rslt, _fifoFrameSize);
      // for(int i = 0; i < 14; i++){
          // if(rslt[i] < 16){
              // Serial.print("0");
          // }
          // Serial.print(rslt[i], HEX);
          // Serial.print(", ");
      // }
      // Serial.println();
      if(reg.TEMP_FIFO_EN && count < _fifoFrameSize){
          _rawData[RAW_DATA_T_H_INDEX] = rslt[count++];
          _rawData[RAW_DATA_T_L_INDEX] = rslt[count++];
      }
      
      if(reg.XG_FIFO_EN && count < _fifoFrameSize){
          _rawData[RAW_DATA_GX_H_INDEX] = rslt[count++];
          _rawData[RAW_DATA_GX_L_INDEX] = rslt[count++];
      }
      if(reg.YG_FIFO_EN && count < _fifoFrameSize){
          _rawData[RAW_DATA_GY_H_INDEX] = rslt[count++];
          _rawData[RAW_DATA_GY_L_INDEX] = rslt[count++];
      }
      if(reg.ZG_FIFO_EN && count < _fifoFrameSize){
          _rawData[RAW_DATA_GZ_H_INDEX] = rslt[count++];
          _rawData[RAW_DATA_GZ_L_INDEX] = rslt[count++];
      }
      if(reg.ACCEL_FIFO_EN && count < _fifoFrameSize){
          _rawData[RAW_DATA_AX_H_INDEX] = rslt[count++];
          _rawData[RAW_DATA_AX_L_INDEX] = rslt[count++];
          _rawData[RAW_DATA_AY_H_INDEX] = rslt[count++];
          _rawData[RAW_DATA_AY_L_INDEX] = rslt[count++];
          _rawData[RAW_DATA_AZ_H_INDEX] = rslt[count++];
          _rawData[RAW_DATA_AZ_L_INDEX] = rslt[count++];
      }

      // for(int i = 0; i < 14; i++){
          // if(_rawData[i] < 16){
              // Serial.print("0");
          // }
          // Serial.print(_rawData[i], HEX);
          // Serial.print(", ");
      // }
      // Serial.println();
  }
  
}


void DFRobot_ICG20660L::outSleepMode(){
  uPowerManagement1Reg_t reg;
  readReg(REG_ICG20660L_PWR_MGMT_1, &reg, 1);
  DBGREG("before: REG_ICG20660L_PWR_MGMT_1", REG_ICG20660L_PWR_MGMT_1, reg.value);
  reg.sleep = 0;
  writeReg(REG_ICG20660L_PWR_MGMT_1, &reg, 1);
  readReg(REG_ICG20660L_PWR_MGMT_1, &reg, 1);
  DBGREG("after: REG_ICG20660L_PWR_MGMT_1", REG_ICG20660L_PWR_MGMT_1, reg.value);
}

void DFRobot_ICG20660L::selectClockSource(uint8_t clksel){
  uPowerManagement1Reg_t reg;
  readReg(REG_ICG20660L_PWR_MGMT_1, &reg, 1);
  DBGREG("before: REG_ICG20660L_PWR_MGMT_1", REG_ICG20660L_PWR_MGMT_1, reg.value);
  reg.clkSel = clksel;
  writeReg(REG_ICG20660L_PWR_MGMT_1, &reg, 1);
  readReg(REG_ICG20660L_PWR_MGMT_1, &reg, 1);
  DBGREG("after: REG_ICG20660L_PWR_MGMT_1", REG_ICG20660L_PWR_MGMT_1, reg.value);
}

void DFRobot_ICG20660L::setBandwidthForAccelInOthersMode(eAccelBandwidth_t  bd){
  uAccelConfig2Reg_t reg;
  readReg(REG_ICG20660L_ACCEL_CONFIG2, &reg, 1);
  DBGREG("before: REG_ICG20660L_ACCEL_CONFIG2", REG_ICG20660L_ACCEL_CONFIG2, reg.value);
  reg.FIFO_SIZE = 0;
  reg.DEC2_CFG = 0;
  switch((uint8_t)bd){
      case eAccel_DLPF_5_1KHZ:
          reg.ACCEL_FCHOICE_B = 0;
          reg.A_DLPF_CFG = 6;
          break;
      case eAccel_DLPF_10_1KHZ:
          reg.ACCEL_FCHOICE_B = 0;
          reg.A_DLPF_CFG = 5;
          break;
      case eAccel_DLPF_21_1KHZ:
          reg.ACCEL_FCHOICE_B = 0;
          reg.A_DLPF_CFG = 4;
          break;
      case eAccel_DLPF_44_1KHZ:
          reg.ACCEL_FCHOICE_B = 0;
          reg.A_DLPF_CFG = 3;
          break;
      case eAccel_DLPF_99_1KHZ:
          reg.ACCEL_FCHOICE_B = 0;
          reg.A_DLPF_CFG = 2;
          break;
      case eAccel_DLPF_218_1KHZ:
          reg.ACCEL_FCHOICE_B = 0;
          reg.A_DLPF_CFG = 1;
          break;
      case eAccel_DLPF_420_1KHZ:
          reg.ACCEL_FCHOICE_B = 0;
          reg.A_DLPF_CFG = 7;
          break;
      case eAccel_DLPF_1046_4KHZ:
          reg.ACCEL_FCHOICE_B = 1;
          break;
  }
  writeReg(REG_ICG20660L_ACCEL_CONFIG2, &reg, 1);
  readReg(REG_ICG20660L_ACCEL_CONFIG2, &reg, 1);
  DBGREG("after: REG_ICG20660L_ACCEL_CONFIG2", REG_ICG20660L_ACCEL_CONFIG2, reg.value);
}
void DFRobot_ICG20660L::setBandwidthForAccelInLowPowerMode(eAccelBandwidth_t  bd){
  uAccelConfig2Reg_t reg;
  readReg(REG_ICG20660L_ACCEL_CONFIG2, &reg, 1);
  DBGREG("before: REG_ICG20660L_ACCEL_CONFIG2", REG_ICG20660L_ACCEL_CONFIG2, reg.value);
  reg.FIFO_SIZE = 0;
  reg.DEC2_CFG = 0;
  switch((uint8_t)bd){
      case eAccel_DLPF_218_1KHZ:
          reg.ACCEL_FCHOICE_B = 0;
          reg.A_DLPF_CFG = 7;
          reg.DEC2_CFG = 1;
          break;
      case eAccel_DLPF_420_1KHZ:
          reg.ACCEL_FCHOICE_B = 0;
          reg.A_DLPF_CFG = 7;
          reg.DEC2_CFG = 0;
          break;
      case eAccel_DLPF_1046_4KHZ:
          reg.ACCEL_FCHOICE_B = 1;
          break;
      case eAccel_DLPF_55_1KHZ:
          reg.ACCEL_FCHOICE_B = 0;
          reg.A_DLPF_CFG = 7;
          reg.DEC2_CFG = 3;
          break;
      case eAccel_DLPF_110_1KHZ:
          reg.ACCEL_FCHOICE_B = 0;
          reg.A_DLPF_CFG = 7;
          reg.DEC2_CFG = 2;
          break;
  }
  writeReg(REG_ICG20660L_ACCEL_CONFIG2, &reg, 1);
  readReg(REG_ICG20660L_ACCEL_CONFIG2, &reg, 1);
  DBGREG("after: REG_ICG20660L_ACCEL_CONFIG2", REG_ICG20660L_ACCEL_CONFIG2, reg.value);
}

void DFRobot_ICG20660L::enableFifo(bool temp, bool gx, bool gy, bool gz, bool accel){
  eFifoEnReg_t reg;
  uUsrCtrlReg_t usr;
  uConfigReg_t cfg;
  readReg(REG_ICG20660L_CONFIG, &cfg, 1);
  readReg(REG_ICG20660L_FIFO_EN, &reg, 1);
  readReg(REG_ICG20660L_USER_CTRL, &usr, 1);
  if(temp||gx||gy||gz||accel){
      usr.FIFO_EN = 1;
      usr.FIFO_RST = 1;
      usr.SIG_COND_RST = 1;
      cfg.FIFO_MODE = 0;
  }else{
      usr.FIFO_EN = 0;
      usr.FIFO_RST = 1;
      usr.SIG_COND_RST = 1;
      cfg.FIFO_MODE = 0;
  }
  reg.ACCEL_FIFO_EN = accel;
  reg.ZG_FIFO_EN = gz;
  reg.YG_FIFO_EN = gy;
  reg.XG_FIFO_EN = gx;
  reg.TEMP_FIFO_EN = temp;
  writeReg(REG_ICG20660L_FIFO_EN, &reg, 1);
  writeReg(REG_ICG20660L_CONFIG, &cfg, 1);
  writeReg(REG_ICG20660L_USER_CTRL, &usr, 1);
  readReg(REG_ICG20660L_FIFO_EN, &reg, 1);
  
  temp = reg.TEMP_FIFO_EN;
  gz = reg.ZG_FIFO_EN;
  gy = reg.YG_FIFO_EN;
  gx = reg.XG_FIFO_EN;
  accel = reg.ACCEL_FIFO_EN;
 
  _fifoFrameSize = accel*6 + (temp + gx + gy + gz)*2;
}


DFRobot_ICG20660L_IIC::DFRobot_ICG20660L_IIC(uint8_t addr, TwoWire *pWire)
  :DFRobot_ICG20660L(),_pWire(pWire),_addr(addr){
  
}
DFRobot_ICG20660L_IIC::~DFRobot_ICG20660L_IIC(){}

int DFRobot_ICG20660L_IIC::init(){
  if (_pWire == NULL) return -1;
  _pWire->begin();
  _pWire->setClock(100000);
  return 0;
}

void DFRobot_ICG20660L_IIC::writeReg(uint8_t reg, void* pBuf, size_t size){
  if(pBuf == NULL){
      DBG("pBuf ERROR!! : null pointer");
  }
  
  uint8_t * _pBuf = (uint8_t *)pBuf;

  _pWire->beginTransmission(_addr);
  _pWire->write(&reg,1);

  for(uint16_t i = 0; i < size; i++){
    _pWire->write(_pBuf[i]);
  }
  _pWire->endTransmission();
}

size_t DFRobot_ICG20660L_IIC::readReg(uint8_t reg, void* pBuf, size_t size){
  if(pBuf == NULL){
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;
  _pWire->beginTransmission(_addr);
  _pWire->write(&reg, 1);
  if( _pWire->endTransmission() != 0){
      return 0;
  }
  _pWire->requestFrom(_addr, (uint8_t) size);
  for(uint16_t i = 0; i < size; i++){
    _pBuf[i] = _pWire->read();
  }
  return size;
}
/*
typedef struct{
  String reg;     
  uint8_t value;
}sRegDecrisption_t;

void DFRobot_ICG20660L::test(){
  uint8_t val = 0;
  readReg(REG_ICG20660L_PWR_MGMT_1, &val, 1);
  DBGREG("REG_ICG20660L_PWR_MGMT_1", REG_ICG20660L_PWR_MGMT_1, val);
  readReg(REG_ICG20660L_PWR_MGMT_2, &val, 1);
  DBGREG("REG_ICG20660L_PWR_MGMT_2", REG_ICG20660L_PWR_MGMT_2, val);
  readReg(REG_ICG20660L_FIFO_COUNTH, &val, 1);
  DBGREG("REG_ICG20660L_FIFO_COUNTH", REG_ICG20660L_FIFO_COUNTH, val);
  readReg(REG_ICG20660L_FIFO_COUNTL, &val, 1);
  DBGREG("REG_ICG20660L_FIFO_COUNTL", REG_ICG20660L_FIFO_COUNTL, val);
  readReg(REG_ICG20660L_FIFO_R_W, &val, 1);
  DBGREG("REG_ICG20660L_FIFO_R_W", REG_ICG20660L_FIFO_R_W, val);
  
  readReg(REG_ICG20660L_INT_ENABLE, &val, 1);
  DBGREG("REG_ICG20660L_INT_ENABLE", REG_ICG20660L_INT_ENABLE, val);
  readReg(REG_ICG20660L_INT_PIN_CFG, &val, 1);
  DBGREG("REG_ICG20660L_INT_PIN_CFG", REG_ICG20660L_INT_PIN_CFG, val);
  readReg(REG_ICG20660L_ACCEL_CONFIG2, &val, 1);
  DBGREG("REG_ICG20660L_ACCEL_CONFIG2", REG_ICG20660L_ACCEL_CONFIG2, val);
  readReg(REG_ICG20660L_SMPLRT_DIV, &val, 1);
  DBGREG("REG_ICG20660L_SMPLRT_DIV", REG_ICG20660L_SMPLRT_DIV, val);
}



static sRegDecrisption_t REGMap[] = {
{"REG_ICG20660L_SELF_TEST_X_GYRO ",             0x00},
{"REG_ICG20660L_SELF_TEST_Y_GYRO ",             0x01},
{"REG_ICG20660L_SELF_TEST_Z_GYRO ",             0x02},
{"REG_ICG20660L_XG_OFFS_TC_H     ",             0x04},
{"REG_ICG20660L_XG_OFFS_TC_L     ",             0x05},
{"REG_ICG20660L_YG_OFFS_TC_H     ",             0x07},
{"REG_ICG20660L_YG_OFFS_TC_L     ",             0x08},
{"REG_ICG20660L_ZG_OFFS_TC_H     ",             0x0A},
{"REG_ICG20660L_ZG_OFFS_TC_L     ",             0x0B},
{"REG_ICG20660L_SELF_TEST_X_ACCEL",             0x0D},
{"REG_ICG20660L_SELF_TEST_Y_ACCEL",             0x0E},
{"REG_ICG20660L_SELF_TEST_Z_ACCEL",             0x0F},
{"REG_ICG20660L_XG_OFFS_USRH     ",             0x13},
{"REG_ICG20660L_XG_OFFS_USRL     ",             0x14},
{"REG_ICG20660L_YG_OFFS_USRH     ",             0x15},
{"REG_ICG20660L_YG_OFFS_USRL     ",             0x16},
{"REG_ICG20660L_ZG_OFFS_USRH     ",             0x17},
{"REG_ICG20660L_ZG_OFFS_USRL     ",             0x18},
{"REG_ICG20660L_CONFIG           ",             0x1A},
{"REG_ICG20660L_GYRO_CONFIG      ",             0x1B},
{"REG_ICG20660L_ACCEL_CONFIG     ",             0x1C},
{"REG_ICG20660L_ACCEL_CONFIG2    ",             0x1D},
{"REG_ICG20660L_LP_MODE_CFG      ",             0x1E},
{"REG_ICG20660L_ACCEL_WOM_THR    ",             0x1F},
{"REG_ICG20660L_FIFO_EN          ",             0x23},
{"REG_ICG20660L_FSYNC_INT        ",             0x36},
{"REG_ICG20660L_INT_PIN_CFG      ",             0x37},
{"REG_ICG20660L_INT_STATUS       ",             0x3A},
{"REG_ICG20660L_ACCEL_XOUT_H     ",             0x3B},
{"REG_ICG20660L_ACCEL_XOUT_L     ",             0x3C},
{"REG_ICG20660L_ACCEL_YOUT_H     ",             0x3D},
{"REG_ICG20660L_ACCEL_YOUT_L     ",             0x3E},
{"REG_ICG20660L_ACCEL_ZOUT_H     ",             0x3F},
{"REG_ICG20660L_ACCEL_ZOUT_L     ",             0x40},
{"REG_ICG20660L_TEMP_OUT_H       ",             0x41},
{"REG_ICG20660L_TEMP_OUT_L       ",             0x42},
{"REG_ICG20660L_GYRO_XOUT_H      ",             0x43},
{"REG_ICG20660L_GYRO_XOUT_L      ",             0x44},
{"REG_ICG20660L_GYRO_YOUT_H      ",             0x45},
{"REG_ICG20660L_GYRO_YOUT_L      ",             0x46},
{"REG_ICG20660L_GYRO_ZOUT_H      ",             0x47},
{"REG_ICG20660L_GYRO_ZOUT_L      ",             0x48},
{"REG_ICG20660L_SIGNAL_PATH_RESET",             0x68},
{"REG_ICG20660L_ACCEL_INTEL_CTRL ",             0x69},
{"REG_ICG20660L_USER_CTRL        ",             0x6A},
{"REG_ICG20660L_WHO_AM_I         ",             0x75},
{"REG_ICG20660L_XA_OFFSET_H      ",             0x77},
{"REG_ICG20660L_XA_OFFSET_L      ",             0x78},
{"REG_ICG20660L_YA_OFFSET_H      ",             0x7A},
{"REG_ICG20660L_YA_OFFSET_L      ",             0x7B},
{"REG_ICG20660L_ZA_OFFSET_H      ",             0x7D},
{"REG_ICG20660L_ZA_OFFSET_L      ",             0x7E},
};
void DFRobot_ICG20660L::test1(){
  uint8_t val;
  for (int i = 0; i < sizeof(REGMap)/sizeof(sRegDecrisption_t);i++){
      readReg(REGMap[i].value, &val, 1);
      DBGREG(REGMap[i].reg, REGMap[i].value, val);
      delay(2);
  }
  
}
*/

DFRobot_ICG20660L_SPI::DFRobot_ICG20660L_SPI(int csPin, SPIClass *spi)
  :DFRobot_ICG20660L(),_spi(spi),_cs(csPin){}

int DFRobot_ICG20660L_SPI::init(){
  if(_spi == NULL) return -1;
  _spi->begin();
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs,HIGH);
  return 0;
}

void DFRobot_ICG20660L_SPI::writeReg(uint8_t reg, void *pBuf, size_t len)
{
  if(pBuf == NULL){
     DBG("pBuf ERROR!! : null pointer");
  }
  setCSPinLow();
  uint8_t *pData = (uint8_t *)pBuf;
  _spi->transfer(reg);
  while(len--) {
    _spi->transfer(*pData);
    pData++;
  }
  setCSPinHigh();
}

size_t DFRobot_ICG20660L_SPI::readReg(uint8_t reg, void *pBuf, size_t len)
{
  if(pBuf == NULL){
      DBG("pBuf ERROR!! : null pointer");
  }
   uint8_t *pData = (uint8_t *)pBuf;
  size_t count = 0;
  setCSPinLow();
  _spi->transfer(reg | 0x80);
  while(len--) {
    *pData = _spi->transfer(0xff);
    pData++;
    count++;
  }
  setCSPinHigh();
  return count;
}

void DFRobot_ICG20660L_SPI::setCSPinLow(){
  _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs,LOW);
}
void DFRobot_ICG20660L_SPI::setCSPinHigh(){
  _spi->endTransaction();
  digitalWrite(_cs,HIGH);
}
