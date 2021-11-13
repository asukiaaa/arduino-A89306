#pragma once
#include <string_asukiaaa.h>

#include "./ValueTypes.hpp"
#include "./utils.hpp"

namespace A89306_asukiaaa {
namespace Registers {

enum class AddressControlEEPROM : uint8_t {
  Config = 161,
  Address = 162,
  Data = 163,
  Flag = 196,
};

uint16_t buildRatedSpeedRegisterValue(float ratedSpeed) {
  return ratedSpeed / 0.53;
}

class EEPROMBase {
 public:
  const uint8_t addressEEPROM;
  EEPROMBase(uint8_t address) : addressEEPROM(address) {}
  virtual ~EEPROMBase() = default;
  virtual void parse(uint8_t data[3]);
  virtual void print(Stream* serial);
  virtual uint32_t toU32() const;
};

class EEPROM8 : public EEPROMBase {
 public:
  EEPROM8() : EEPROMBase(8) {}
  ValueTypes::PWMinRange pwmRange = ValueTypes::PWMinRange::moreThan2p8kHz;
  ValueTypes::Direction direction = ValueTypes::Direction::ABC;
  ValueTypes::AccelerateRange accerationRange =
      ValueTypes::AccelerateRange::to816;
  ValueTypes::DigitalSpeedMode digitalSpeedMode =
      ValueTypes::DigitalSpeedMode::PWM;
  uint16_t ratedSpeedRegisterValue = 0b110101100;  // 226.84Hz

  void setRegisterByRatedSpeedHz(float ratedSpeedHz) {
    ratedSpeedRegisterValue = buildRatedSpeedRegisterValue(ratedSpeedHz);
  }

  uint32_t toU32() const {
    uint32_t v = 0;
    if (pwmRange == ValueTypes::PWMinRange::equalOrLessThan2p8kHz) {
      v |= utils::buildTrueBit(15);
    }
    if (direction == ValueTypes::Direction::ABC) {
      v |= utils::buildTrueBit(14);
    }
    if (accerationRange == ValueTypes::AccelerateRange::to12p75) {
      v |= utils::buildTrueBit(13);
    }
    v |= ((uint32_t)digitalSpeedMode) << 11;
    v |= ratedSpeedRegisterValue & (((uint16_t)0b111 << 8) | 0xff);
    return v;
  }

  void parse(uint8_t data[3]) {
    pwmRange = utils::isBitTrue(data[1], 7)
                   ? ValueTypes::PWMinRange::equalOrLessThan2p8kHz
                   : ValueTypes::PWMinRange::moreThan2p8kHz;
    direction = utils::isBitTrue(data[1], 6) ? ValueTypes::Direction::ABC
                                             : ValueTypes::Direction::ACB;
    accerationRange = utils::isBitTrue(data[1], 5)
                          ? ValueTypes::AccelerateRange::to12p75
                          : ValueTypes::AccelerateRange::to816;
    digitalSpeedMode = (ValueTypes::DigitalSpeedMode)((data[1] >> 3) & 0b11);
    ratedSpeedRegisterValue = (((uint16_t)data[1] & 0b111) << 8) | data[0];
  }

  void print(Stream* serial) {
    serial->print(
        "pwm range " +
        String(pwmRange == ValueTypes::PWMinRange::equalOrLessThan2p8kHz
                   ? "equal or less than"
                   : "more than 2.8kHz, "));
    serial->print("direction " +
                  String(direction == ValueTypes::Direction::ABC ? "A->B->C"
                                                                 : "A->C->B") +
                  ", ");
    serial->print("accerate range " +
                  String(accerationRange == ValueTypes::AccelerateRange::to12p75
                             ? "0to12.75"
                             : "0to816") +
                  "Hz/s, ");
    uint8_t rawSpeedMode = (uint8_t)digitalSpeedMode;
    String strSpeedMode = rawSpeedMode == 0   ? "PWM"
                          : rawSpeedMode == 1 ? "Clock on SPD"
                          : rawSpeedMode == 2 ? "Clock on DIR"
                                              : "Clock on BRAKE";
    serial->print("digital speed mode " + strSpeedMode + ", ");
    serial->print("rated speed " + String(ratedSpeedRegisterValue * 0.53) +
                  "Hz");
  }
};

class EEPROM9 : public EEPROMBase {
 public:
  EEPROM9() : EEPROMBase(9) {}
  uint8_t startupAcceleration = 202;
  void print(Stream* serial) {
    serial->print("startup acceleration ");
    serial->print(startupAcceleration);
  }

  void parse(uint8_t data[3]) { startupAcceleration = data[0]; }

  uint32_t toU32() const {
    uint32_t v = 0;
    v |= startupAcceleration;
    return v;
  }
};

uint16_t buildRatedCurrentRegisterValue(uint16_t ratedCurrent,
                                        uint8_t senseResistorRegisterValue) {
  uint16_t result = (uint16_t)ratedCurrent * senseResistorRegisterValue / 125;
  return result & (((uint16_t)0b111 << 8) | 0xff);
}

class EEPROM10 : public EEPROMBase {
 public:
  EEPROM10() : EEPROMBase(10) {}
  ValueTypes::StartupCurrent startupCurrent =
      ValueTypes::StartupCurrent::x4div8;
  ValueTypes::SPDMode spdMode = ValueTypes::SPDMode::digital;
  uint16_t ratedCurrentRegisterValue =
      0b00111011111;  // about 1.4A for 13mOhm sense resister

  uint32_t toU32() const {
    uint32_t v = 0;
    v |= ((uint16_t)startupCurrent) << 13;
    if (spdMode == ValueTypes::SPDMode::analog) {
      v |= utils::buildTrueBit(11);
    }
    v |= ((((uint16_t)0b111) << 8) | 0xff) & ratedCurrentRegisterValue;
    return v;
  }

  void parse(uint8_t data[3]) {
    startupCurrent = (ValueTypes::StartupCurrent)(data[1] >> 5);
    spdMode = utils::isBitTrue(data[1], 3) ? ValueTypes::SPDMode::analog
                                           : ValueTypes::SPDMode::digital;
    ratedCurrentRegisterValue = (((uint16_t)data[1] & 0b111) << 8) | data[0];
  }

  void print(Stream* serial) {
    serial->print("startup current rated_current*" +
                  String((uint8_t)startupCurrent + 1) + "/8, ");
    serial->print(
        "speed mode " +
        String(spdMode == ValueTypes::SPDMode::analog ? "Analog" : "Digital") +
        ", ");
    serial->print("rated current " + String(ratedCurrentRegisterValue) +
                  "*125/sense_resistor_rigister_value mA");
  }
};

class EEPROM11 : public EEPROMBase {
 public:
  EEPROM11() : EEPROMBase(11) {}
  ValueTypes::StartupMode startupMode = ValueTypes::StartupMode::pulse2;
  bool powerControlEnable = true;
  bool openPhProtect = true;
  bool openDrive = false;

  uint32_t toU32() const {
    uint32_t v = 0;
    v |= ((uint16_t)startupMode) << 10;
    if (powerControlEnable) {
      v |= utils::buildTrueBit(7);
    }
    if (openPhProtect) {
      v |= utils::buildTrueBit(4);
    }
    if (openDrive) {
      v |= utils::buildTrueBit(3);
    }
    return v;
  }

  void parse(uint8_t data[3]) {
    startupMode = ValueTypes::StartupMode((data[1] >> 2) & 0b11);
    powerControlEnable = utils::isBitTrue(data[0], 7);
    openPhProtect = utils::isBitTrue(data[0], 4);
    openDrive = utils::isBitTrue(data[0], 3);
  }

  void print(Stream* serial) {
    uint8_t rawStartupMode = (uint8_t)startupMode;
    String strStartupMode = rawStartupMode == 0   ? "6pulse"
                            : rawStartupMode == 1 ? "2pulse"
                            : rawStartupMode == 2 ? "slight move"
                                                  : "align&go";
    serial->print("startup mode " + strStartupMode + ", ");
    serial->print("power control enable " +
                  string_asukiaaa::trueFalse(powerControlEnable) + ", ");
    serial->print("open ph protect " +
                  string_asukiaaa::trueFalse(openPhProtect) + ", ");
    serial->print("open drive " + string_asukiaaa::trueFalse(openDrive));
  }
};

class EEPROM12 : public EEPROMBase {
 public:
  EEPROM12() : EEPROMBase(12) {}
  bool openWindow = false;
  uint8_t pidP = 15;

  uint32_t toU32() const {
    uint32_t v = 0;
    if (openWindow) {
      v |= utils::buildTrueBit(16);
    }
    v |= pidP & 0b11111;
    return v;
  }

  void parse(uint8_t data[3]) {
    openWindow = utils::isBitTrue(data[2], 0);
    pidP = data[0] & 0b11111;
  }

  void print(Stream* serial) {
    serial->print("open window " + string_asukiaaa::trueFalse(openWindow) +
                  ", ");
    serial->print("PID_P " + String(pidP));
  }
};

class EEPROM13 : public EEPROMBase {
 public:
  EEPROM13() : EEPROMBase(13) {}
  ValueTypes::DelayStart delayStart = ValueTypes::DelayStart::ms25;

  uint32_t toU32() const {
    uint32_t v = 0b100;  // unknown flags
    // uint32_t v = 0;
    if (delayStart == ValueTypes::DelayStart::ms100)
      v |= utils::buildTrueBit(14);
    return v;
  }
  void parse(uint8_t data[3]) {
    delayStart = utils::isBitTrue(data[1], 6) ? ValueTypes::DelayStart::ms100
                                              : ValueTypes::DelayStart::ms25;
  }
  void print(Stream* serial) {
    serial->print(
        "delay start " +
        String(delayStart == ValueTypes::DelayStart::ms100 ? "100ms" : "25ms"));
  }
};

class EEPROM14 : public EEPROMBase {
 public:
  EEPROM14() : EEPROMBase(14) {}
  bool disableFG = false;

  uint32_t toU32() const {
    uint32_t v = 0b1110 << 8;  // unknown flags
    if (disableFG) {
      v |= utils::buildTrueBit(4);
    }
    return v;
  }

  void parse(uint8_t data[3]) { disableFG = utils::isBitTrue(data[0], 4); }

  void print(Stream* serial) {
    serial->print("FG_pin_dis " +
                  String(disableFG ? "disabled for I2C mode" : "FG is used"));
  }
};

class EEPROM15 : public EEPROMBase {
 public:
  EEPROM15() : EEPROMBase(15) {}
  ValueTypes::SafeBrakeThrd safeBrakeThrd = ValueTypes::SafeBrakeThrd::div2;
  ValueTypes::DeadtimeSetting deadtimeSetting =
      ValueTypes::DeadtimeSetting::ns400;
  bool softOn = true;
  bool softOff = false;
  ValueTypes::AngleErrorLock angleErrorLock =
      ValueTypes::AngleErrorLock::degrees13;

  uint32_t toU32() const {
    // EEPROM15 00000000 01001001 10111101   0  73 189
    // safe brake 1/2*rated_current, deadtime setting 400ns, soft on true, soft
    // off false, angle error lock 13degrees
    uint32_t v = 0;
    v |= ((uint8_t)safeBrakeThrd << 14);
    v |= ((uint8_t)deadtimeSetting << 8);
    if (softOn) {
      v |= utils::buildTrueBit(7);
    }
    if (softOff) {
      v |= utils::buildTrueBit(6);
    }
    v |= ((uint8_t)angleErrorLock << 2);
    return v;
  }

  void parse(uint8_t data[3]) {
    safeBrakeThrd =
        (ValueTypes::SafeBrakeThrd)(((data[2] & 0b1) << 2) | (data[1] >> 6));
    deadtimeSetting = (ValueTypes::DeadtimeSetting)(data[1] & 0b1111);
    softOn = utils::isBitTrue(data[0], 7);
    softOff = utils::isBitTrue(data[0], 6);
    angleErrorLock = (ValueTypes::AngleErrorLock)((data[0] & 0b1100) >> 2);
  }

  void print(Stream* serial) {
    uint8_t rawBrake = (uint8_t)safeBrakeThrd;
    serial->print("safe brake 1/" + String((uint16_t)pow(2, rawBrake)) +
                  "*rated_current, ");
    uint8_t rawDeadTime = (uint8_t)deadtimeSetting;
    uint16_t deadTimeNs = rawDeadTime == 0 ? 640 : (rawDeadTime + 1) * 40;
    serial->print("deadtime setting " + String(deadTimeNs) + "ns, ");
    serial->print("soft on " + string_asukiaaa::trueFalse(softOn) + ", ");
    serial->print("soft off " + string_asukiaaa::trueFalse(softOff) + ", ");
    uint8_t rawAngleErrorLock = (uint8_t)angleErrorLock;
    String strAngleErrorLock = rawAngleErrorLock == 0   ? "disabled"
                               : rawAngleErrorLock == 1 ? "5degrees"
                               : rawAngleErrorLock == 2 ? "9degrees"
                                                        : "13degrees";
    serial->print("angle error lock " + strAngleErrorLock);
  }
};

class EEPROM16 : public EEPROMBase {
 public:
  EEPROM16() : EEPROMBase(16) {}
  ValueTypes::BEMFLockFilter bemfLockFilter =
      ValueTypes::BEMFLockFilter::medium;
  ValueTypes::DeclarateBuffer declarateBuffer =
      ValueTypes::DeclarateBuffer::mediumBuffer;
  ValueTypes::AccelerateBuffer accelerateBuffer =
      ValueTypes::AccelerateBuffer::mediumBuffer;
  ValueTypes::FirstCycleSpeed firstCycleSpeed =
      ValueTypes::FirstCycleSpeed::Hz1p1;
  ValueTypes::OCPMasking ocpMasking = ValueTypes::OCPMasking::mask640ns;
  ValueTypes::OCPResetMode ocpResetMode =
      ValueTypes::OCPResetMode::after5seconds;
  ValueTypes::OCPEnable ocpEnable =
      ValueTypes::OCPEnable::filter480ns_HS_LS_Enabled;

  uint32_t toU32() const {
    uint32_t v = 0;
    v |= (uint8_t)bemfLockFilter << 12;
    v |= (uint8_t)declarateBuffer << 10;
    v |= (uint8_t)accelerateBuffer << 8;
    v |= (uint8_t)firstCycleSpeed << 6;
    v |= (uint8_t)ocpMasking << 4;
    v |= (uint8_t)ocpResetMode << 3;
    v |= (uint8_t)ocpEnable;
    return v;
  }

  void parse(uint8_t data[3]) {
    bemfLockFilter = (ValueTypes::BEMFLockFilter)((data[1] >> 4) & 0b11);
    declarateBuffer = (ValueTypes::DeclarateBuffer)((data[1] >> 2) & 0b11);
    accelerateBuffer = (ValueTypes::AccelerateBuffer)(data[1] & 0b11);
    firstCycleSpeed = (ValueTypes::FirstCycleSpeed)((data[0] >> 6) & 0b11);
    ocpMasking = (ValueTypes::OCPMasking)((data[0] >> 4) & 0b11);
    ocpResetMode = utils::isBitTrue(data[0], 3)
                       ? ValueTypes::OCPResetMode::uponMotorRestart
                       : ValueTypes::OCPResetMode::after5seconds;
    ocpEnable = (ValueTypes::OCPEnable)(data[0] & 0b111);
  }

  void print(Stream* serial) {
    uint8_t rawBemfLockFilter = (uint8_t)bemfLockFilter;
    String strBEMFLockFilter = rawBemfLockFilter == 0   ? "disabled"
                               : rawBemfLockFilter == 1 ? "robust"
                               : rawBemfLockFilter == 2 ? "medium"
                                                        : "sensitive";
    serial->print("BEMF Lock filter " + strBEMFLockFilter + ", ");
    uint8_t rawDeclarateBuffer = (uint8_t)declarateBuffer;
    String strDeclarateBuffer = rawDeclarateBuffer == 0   ? "no buffer"
                                : rawDeclarateBuffer == 1 ? "fast buffer"
                                : rawDeclarateBuffer == 2 ? "medium buffer"
                                                          : "slow buffer";
    serial->print("declarete buffer " + strDeclarateBuffer + ", ");
    uint8_t rawAccelerateBuffer = (uint8_t)accelerateBuffer;
    String strAccelerateBuffer = rawAccelerateBuffer == 0   ? "no buffer"
                                 : rawAccelerateBuffer == 1 ? "fast buffer"
                                 : rawAccelerateBuffer == 2 ? "medium buffer"
                                                            : "slow buffer";
    serial->print("accelerete buffer " + strAccelerateBuffer + ", ");
    uint8_t rawFirstCycleSpeed = (uint8_t)firstCycleSpeed;
    String strFirstCycleSpeed = rawFirstCycleSpeed == 0   ? "0.55Hz1.18sec"
                                : rawFirstCycleSpeed == 1 ? "1.1Hz0.91sec"
                                : rawFirstCycleSpeed == 2 ? "2.2Hz0.45sec"
                                                          : "4.4Hz0.23sec";
    serial->print("first cycle speed " + strFirstCycleSpeed + ", ");
    uint8_t rawOcpMasking = (uint8_t)ocpMasking;
    String strOCPMasking = rawOcpMasking == 0   ? "no masking"
                           : rawOcpMasking == 1 ? "320ns mask"
                           : rawOcpMasking == 2 ? "640ns mask"
                                                : "1280ns mask";
    serial->print("OCP masking " + strOCPMasking + ", ");
    serial->print(
        "OCP reset mode " +
        String(ocpResetMode == ValueTypes::OCPResetMode::uponMotorRestart
                   ? "upon motor restart"
                   : "after 5 seconds") +
        ", ");
    uint8_t rawOcpEnable = (uint8_t)ocpEnable;
    String strOCPEnable =
        rawOcpEnable == 0   ? "no filter HS/LS OCP enabled"
        : rawOcpEnable == 1 ? "120ns HS/LS OCP enabled"
        : rawOcpEnable == 2 ? "240ns HS/LS OCP enabled"
        : rawOcpEnable == 3 ? "360ns HS/LS OCP enabled"
        : rawOcpEnable == 4 ? "480ns HS/LS OCP enabled"
        : rawOcpEnable == 5 ? "480ns LS OCP enabled HS OCP disabled"
        : rawOcpEnable == 6 ? "480ns LS OCP disabled HS OCP enabled"
                            : "480ns HS.LS OCP disabled";
    serial->print("OCP enable " + strOCPEnable);
  }
};

class EEPROM17 : public EEPROMBase {
 public:
  EEPROM17() : EEPROMBase(17) {}
  bool i2cSpeedMode = false;
  uint16_t speedDemand = 0x1ff;  // 100%

  uint32_t toU32() const {
    uint32_t v = (uint16_t)0b1 << 13;  // unkown flags
    if (i2cSpeedMode) {
      v |= utils::buildTrueBit(9);
    }
    v |= speedDemand & 0x1ff;
    return v;
  }

  void parse(uint8_t data[3]) {
    i2cSpeedMode = utils::isBitTrue(data[1], 1);
    speedDemand = (((uint16_t)data[1] & 0b1) << 8) | data[0];
  }

  void print(Stream* serial) {
    serial->print("isI2cMode ");
    serial->print(string_asukiaaa::trueFalse(i2cSpeedMode));
    serial->print(" speed demand ");
    float speedPercent = (float)speedDemand * 100 / 511;
    serial->print(speedPercent);
    serial->print('%');
  }
};

class EEPROM18 : public EEPROMBase {
 public:
  EEPROM18() : EEPROMBase(18) {}
  ValueTypes::DriveGateSlew driveGateSlew = ValueTypes::DriveGateSlew::level2;
  uint8_t ipdCurrentThresholdRegisterValue = 0b1110;

  uint32_t toU32() const {
    uint32_t v = 0b1101;  // unkown flags
    v |= (uint16_t)driveGateSlew << 14;
    v |= ((uint16_t)(ipdCurrentThresholdRegisterValue / 0.086) & 0b111111) << 8;
    return v;
  }
  void parse(uint8_t data[3]) {
    driveGateSlew = (ValueTypes::DriveGateSlew)((data[1] >> 6) & 0b11);
    ipdCurrentThresholdRegisterValue = data[1] & 0b11111;
  }
  void print(Stream* serial) {
    uint8_t rawDriveGateSlew = (uint8_t)driveGateSlew;
    String strDriveGateSlew =
        rawDriveGateSlew == 0   ? "level0 source15mA sink 30mA"
        : rawDriveGateSlew == 1 ? "level1 source30mA sink 60mA"
        : rawDriveGateSlew == 2 ? "level2 source55mA sink 105mA"
                                : "unknown";
    serial->print("drive gate slew " + strDriveGateSlew + ", ");
    serial->print("IPD current threshold " +
                  String((float)ipdCurrentThresholdRegisterValue * 0.086) +
                  "A");
  }
};

class EEPROM19 : public EEPROMBase {
 public:
  EEPROM19() : EEPROMBase(19) {}
  uint8_t mosfetCompFalling = 7;
  uint8_t mosfetCompRising = 8;

  uint32_t toU32() const {
    uint32_t v = 0;
    v |= (mosfetCompFalling & 0b1111) << 12;
    v |= (mosfetCompRising & 0b1111) << 8;
    return v;
  }

  void parse(uint8_t data[3]) {
    mosfetCompFalling = (data[1] >> 4) & 0b1111;
    mosfetCompRising = data[1] & 0b1111;
  }

  void print(Stream* serial) {
    serial->print("mosfet comp falling " + String(mosfetCompFalling) + ", ");
    serial->print("mosfet comp rising " + String(mosfetCompRising));
  }
};

uint8_t buildSenseResistorRegisterValue(uint8_t milliOhm) {
  return milliOhm * 3.7;
}

float calcSenseResisterByRegistorValue(uint8_t registorValue) {
  return (float)registorValue / 3.7;
}

uint8_t buildRatedVoltageRegistorValue(float volt) { return volt * 5; }

float calcRatedVoltageByRegistorValue(uint8_t registerValue) {
  return (float)registerValue / 5;
}

class EEPROM20 : public EEPROMBase {
 public:
  EEPROM20() : EEPROMBase(20) {}
  uint8_t senseResisterRegistorValue = 0b00101110;  // 12.43mOhm
  uint8_t ratedVoltageRegistorValue = 0b01111000;   // 24v

  void setResistorMililOhm(float milliOhm) {
    senseResisterRegistorValue = buildSenseResistorRegisterValue(milliOhm);
  }

  void setRatedVoltage(float volt) {
    ratedVoltageRegistorValue = buildRatedVoltageRegistorValue(volt);
  }

  uint32_t toU32() const {
    return ((uint16_t)senseResisterRegistorValue << 8) |
           ratedVoltageRegistorValue;
  }

  void parse(uint8_t data[3]) {
    senseResisterRegistorValue = data[1];
    ratedVoltageRegistorValue = data[0];
  }

  void print(Stream* serial) {
    serial->print(
        "sense resistor " +
        String(calcSenseResisterByRegistorValue(senseResisterRegistorValue)) +
        "mOhm, ");
    serial->print(
        "rated volt " +
        String(calcRatedVoltageByRegistorValue(ratedVoltageRegistorValue)) +
        "V");
  }
};

class EEPROM21 : public EEPROMBase {
 public:
  EEPROM21() : EEPROMBase(21) {}
  bool standbyDisable = false;
  ValueTypes::SpeedInputOffThreshold speedInputOffThreshold =
      ValueTypes::SpeedInputOffThreshold::percent5p8;
  uint8_t slightMvDemandRegisterValue = 0b011;  // 12%

  uint32_t toU32() const {
    uint32_t v = 0;
    if (standbyDisable) {
      v |= utils::buildTrueBit(15);
    }
    v |= (uint8_t)speedInputOffThreshold << 8;
    v |= (slightMvDemandRegisterValue & 0b111) << 5;
    return v;
  }

  void parse(uint8_t data[3]) {
    standbyDisable = utils::isBitTrue(data[1], 7);
    speedInputOffThreshold =
        (ValueTypes::SpeedInputOffThreshold)(data[1] & 0b11);
    slightMvDemandRegisterValue = (data[0] >> 5) & 0b111;
  }

  void print(Stream* serial) {
    serial->print("standby disable " +
                  string_asukiaaa::trueFalse(standbyDisable) + ", ");
    uint8_t rawSpeedInputOffThresh = (uint8_t)speedInputOffThreshold;
    String strSpeedInputOffThresh = rawSpeedInputOffThresh == 0   ? "9.7"
                                    : rawSpeedInputOffThresh == 1 ? "5.8"
                                    : rawSpeedInputOffThresh == 2 ? "12.8"
                                                                  : "19.5";
    serial->print("speed input off threshold " + strSpeedInputOffThresh +
                  "%, ");
    serial->print("slight mv demand " +
                  String(slightMvDemandRegisterValue * 3.2 + 2.4));
  }
};

class EEPROM22 : public EEPROMBase {
 public:
  EEPROM22() : EEPROMBase(22) {}
  ValueTypes::VdsThereshold vdsThreshold = ValueTypes::VdsThereshold::v1;
  bool deadtimeCompensation = true;
  ValueTypes::LockRestartSet lockRestartSet = ValueTypes::LockRestartSet::sec5;
  bool vibrationLock = true;
  bool softOff4Sec = false;
  bool brakeMode = false;
  ValueTypes::RestartAttempt restartAttempt =
      ValueTypes::RestartAttempt::always;
  uint8_t clockSpeedRatioRegisterValue = 0b11110;

  uint32_t toU32() const {
    uint32_t v = 0;
    return v;
  }

  void parse(uint8_t data[3]) {
    vdsThreshold = utils::isBitTrue(data[1], 7) ? ValueTypes::VdsThereshold::v2
                                                : ValueTypes::VdsThereshold::v1;
    deadtimeCompensation = utils::isBitTrue(data[1], 4);
    lockRestartSet = utils::isBitTrue(data[1], 3)
                         ? ValueTypes::LockRestartSet::sec10
                         : ValueTypes::LockRestartSet::sec5;
    vibrationLock = utils::isBitTrue(data[1], 2);
    softOff4Sec = utils::isBitTrue(data[1], 1);
    brakeMode = utils::isBitTrue(data[1], 0);
    restartAttempt = (ValueTypes::RestartAttempt)(data[0] >> 6);
    clockSpeedRatioRegisterValue = (data[0] & 0b111111);
  }

  void print(Stream* serial) {
    serial->print(
        "vds thresh " +
        String(vdsThreshold == ValueTypes::VdsThereshold::v2 ? 2 : 1) + "V, ");
    serial->print("deadtime comp " +
                  string_asukiaaa::trueFalse(deadtimeCompensation) + ", ");
    serial->print(
        "lock restart " +
        String(lockRestartSet == ValueTypes::LockRestartSet::sec10 ? 10 : 5) +
        "sec, ");
    serial->print("vibration lock " +
                  string_asukiaaa::trueFalse(vibrationLock) + ", ");
    serial->print("soft off 4sec " + string_asukiaaa::trueFalse(softOff4Sec) +
                  ", ");
    serial->print("brake mode " + String(brakeMode ? "always" : "see pin") +
                  ", ");
    uint8_t restartData = (uint8_t)restartAttempt;
    String restartStr = restartData == 0   ? "always"
                        : restartData == 1 ? "3times"
                        : restartData == 2 ? "5times"
                                           : "10times";
    serial->print("restart attempt " + restartStr + ", ");
    float clockSpeedRatio = (float)clockSpeedRatioRegisterValue * 0.25;
    serial->print("clock speed ratio " + String(clockSpeedRatio) + "rpm/Hz");
  }
};

class EEPROM23 : public EEPROMBase {
 public:
  EEPROM23() : EEPROMBase(23) {}
  bool checkCurve = false;
  bool speedCurBidir = false;
  ValueTypes::OperationMode operationMode = ValueTypes::OperationMode::openLoop;
  uint16_t parameterFull = 134;

  uint32_t toU32() const {
    // EEPROM23 00000000 00000000 10000110   0   0 134
    // check curve disabled, speed cur bidir input curve operates in a
    // unidirectional mode and the direction is set by the device terminal or
    // via I2C register, operation mode open loop operation output voltage
    // magnitude set by input demand, parameter full 134
    uint32_t v = 0;
    if (checkCurve) {
      v |= utils::buildTrueBit(17);
    }
    if (speedCurBidir) {
      v |= utils::buildTrueBit(16);
    }
    v |= ((uint8_t)operationMode << 11);
    v |= parameterFull & ((0b111 << 8) | 0xff);
    return v;
  }

  void parse(uint8_t data[3]) {
    checkCurve = utils::isBitTrue(data[2], 1);
    speedCurBidir = utils::isBitTrue(data[2], 0);
    operationMode = (ValueTypes::OperationMode)((data[1] >> 3) & 0b11);
    parameterFull = ((data[1] & 0b11) << 8) | data[0];
  }

  void print(Stream* serial) {
    serial->print("check curve " + string_asukiaaa::trueFalse(checkCurve) +
                  ", ");
    serial->print("speed cur bidir " +
                  string_asukiaaa::trueFalse(speedCurBidir) + ", ");
    uint8_t rawOperationMode = (uint8_t)operationMode;
    String strOperationMode =
        rawOperationMode == 0 ? "open loop operation output voltage "
                                "magnitude set by input demand"
        : rawOperationMode == 1 ? "constant torque by parameter full"
        : rawOperationMode == 2 ? "constant speed by parameter full"
                                : "constant power by parameter full";
    serial->print("operation mode " + strOperationMode + ", ");
    serial->print("parameter full " + String(parameterFull));
  }
};

class EEPROM24 : public EEPROMBase {
 public:
  EEPROM24() : EEPROMBase(24) {}
  uint8_t motorResistor = 186;
  uint32_t toU32() const { return motorResistor; }
  void parse(uint8_t data[3]) { motorResistor = data[0]; }
  void print(Stream* serial) {
    serial->print("motor resistor " + String(motorResistor));
  }
};

class EEPROM25 : public EEPROMBase {
 public:
  EEPROM25() : EEPROMBase(25) {}
  bool brkInput = false;
  bool brkFromReg = false;
  bool dirFromReg = false;
  ValueTypes::InductanceShift inductanceShift =
      ValueTypes::InductanceShift::x001;
  uint8_t inductance = 0xff;

  uint32_t toU32() const {
    uint32_t v = 0;
    if (brkInput) {
      v |= utils::buildTrueBit(15);
    }
    if (brkFromReg) {
      v |= utils::buildTrueBit(14);
    }
    if (dirFromReg) {
      v |= utils::buildTrueBit(13);
    }
    v |= ((uint16_t)inductanceShift << 8) & 0b111;
    v |= inductance;
    return v;
  }

  void parse(uint8_t data[3]) {
    brkInput = utils::isBitTrue(data[1], 7);
    brkFromReg = utils::isBitTrue(data[1], 6);
    dirFromReg = utils::isBitTrue(data[1], 5);
    inductanceShift = (ValueTypes::InductanceShift)(data[1] & 0b111);
    inductance = data[0];
  }

  void print(Stream* serial) {
    serial->print("BRK_input " + string_asukiaaa::trueFalse(brkInput) + ", ");
    serial->print("BRK_from_reg " +
                  String(brkFromReg ? "see BRK_input" : "see BRK pin") + ", ");
    serial->print("DIR_from_reg " + string_asukiaaa::trueFalse(dirFromReg) +
                  ", ");
    serial->print("inductance shift " +
                  String((uint8_t)pow(2, (uint8_t)inductanceShift)) + ", ");
    serial->print("inductance " + String(inductance) + ", ");
  }
};

class EEPROM26 : public EEPROMBase {
 public:
  EEPROM26() : EEPROMBase(26) {}
  uint8_t ktSet = 55;
  uint32_t toU32() const { return ktSet; }
  void parse(uint8_t data[3]) { ktSet = data[0]; }
  void print(Stream* serial) { serial->print("Kt set " + String(ktSet)); }
};

class EEPROM28 : public EEPROMBase {
 public:
  EEPROM28() : EEPROMBase(28) {}
  uint8_t currentLoopL = 3;
  uint8_t powerLoopP = 63;
  uint8_t speedLoopP = 14;
  uint8_t currentLoopP = 6;

  uint32_t toU32() const {
    uint32_t v = 0;
    v |= (currentLoopL & 0b111) << 15;
    v |= (powerLoopP & 0b11111) << 10;
    v |= (speedLoopP & 0b11111) << 5;
    v |= (currentLoopP & 0b11111);
    return v;
  }

  void parse(uint8_t data[3]) {
    currentLoopL = ((data[2] & 0b11) << 1) | ((data[1] >> 7));
    powerLoopP = ((data[1] >> 2) | 0b11111);
    speedLoopP = ((data[1] & 0b11) << 3) | (data[0] >> 5);
    currentLoopP = (data[0] & 0b1111);
  }

  void print(Stream* serial) {
    serial->print("current loop l " + String(currentLoopL) + ", ");
    serial->print("power loop p " + String(powerLoopP) + ", ");
    serial->print("speed loop p " + String(speedLoopP) + ", ");
    serial->print("current loop p " + String(currentLoopP));
  }
};

class EEPROM29 : public EEPROMBase {
 public:
  EEPROM29() : EEPROMBase(29) {}
  uint8_t powerLoopL = 2;
  uint8_t speedLoopL = 3;

  uint32_t toU32() const {
    uint32_t v = 0;
    v |= (powerLoopL & 0b111) << 3;
    v |= (speedLoopL & 0b111);
    return v;
  }

  void parse(uint8_t data[3]) {
    powerLoopL = ((data[0] >> 3) & 0b111);
    speedLoopL = data[0] & 0b111;
  }

  void print(Stream* serial) {
    serial->print("power loop l " + String(powerLoopL) + ", ");
    serial->print("speed loop l " + String(speedLoopL));
  }
};

class EEPROM31 : public EEPROMBase {
 public:
  EEPROM31() : EEPROMBase(31) {}
  uint16_t outputArrayTrim;
  uint32_t toU32() const { return outputArrayTrim; }

  void parse(uint8_t data[3]) {
    outputArrayTrim = (uint16_t)(data[1] << 8) | data[0];
  }

  void print(Stream* serial) {
    serial->print("output array trim " + String(outputArrayTrim));
  }
};

}  // namespace Registers
}  // namespace A89306_asukiaaa
