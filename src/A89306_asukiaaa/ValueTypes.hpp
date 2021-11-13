#pragma once

namespace A89306_asukiaaa {
namespace ValueTypes {

// eeprom8
enum class PWMinRange : uint8_t {
  moreThan2p8kHz = 0,
  equalOrLessThan2p8kHz = 1,
};
enum class Direction : uint8_t {
  ACB = 0,
  ABC = 1,
};
enum class AccelerateRange : uint8_t {
  to816 = 0,
  to12p75 = 1,
};
enum class DigitalSpeedMode : uint8_t {
  PWM = 0,
  ClockSPD = 1,
  ClockDIR = 2,
  ClockBRAME = 3,
};
// eeprom10
enum class StartupCurrent : uint8_t {
  x1div8 = 0,
  x2div8 = 1,
  x3div8 = 2,
  x4div8 = 3,
  x5div8 = 4,
  x6div8 = 5,
  x7div8 = 6,
  x8div8 = 7,
};
enum class SPDMode {
  digital = 0,
  analog = 1,
};
// eeprom11
enum class StartupMode : uint8_t {
  pulse6 = 0,
  pulse2 = 1,
  slightMove = 2,
  alighAndGo = 3,
};
// eeprom13
enum class DelayStart : uint8_t {
  ms25 = 0,
  ms100 = 1,
};
// eeprom15
enum class SafeBrakeThrd : uint8_t {
  div1 = 0,
  div2 = 1,
  div4 = 2,
  div8 = 3,
  div16 = 4,
  div32 = 5,
  div64 = 6,
  div128 = 7,
};
enum class DeadtimeSetting : uint8_t {
  // ns640 = 0,
  ns80 = 1,
  ns120 = 2,
  ns160 = 3,
  ns200 = 4,
  ns240 = 5,
  ns280 = 6,
  ns320 = 7,
  ns360 = 8,
  ns400 = 9,
  ns440 = 10,
  ns480 = 11,
  ns520 = 12,
  ns560 = 13,
  ns600 = 14,
  ns640 = 15,
};
enum class AngleErrorLock : uint8_t {
  disabled = 0,
  degrees5 = 1,
  degrees9 = 2,
  degrees13 = 3,
};
// eeprom16
enum class BEMFLockFilter : uint8_t {
  disabled = 0,
  robust = 1,
  medium = 2,
  sensitive = 3,
};
enum class DeclarateBuffer : uint8_t {
  noBuffer = 0,
  fastBuffer = 1,
  mediumBuffer = 2,
  slowBuffer = 3,
};
enum class AccelerateBuffer : uint8_t {
  noBuffer = 0,
  fastBuffer = 1,
  mediumBuffer = 2,
  slowBuffer = 3,
};
enum class FirstCycleSpeed : uint8_t {
  Hz0p55 = 0,
  Hz1p1 = 1,
  Hz2p2 = 2,
  Hz4p4 = 3,
};
enum class OCPMasking : uint8_t {
  noMask = 0,
  mask320ns = 1,
  mask640ns = 2,
  mask1280ns = 3,
};
enum class OCPResetMode : uint8_t {
  uponMotorRestart = 0,
  after5seconds = 1,
};
enum class OCPEnable : uint8_t {
  noFilter = 0,
  filter120ns_HS_LS_Enabled = 1,
  filter240ns_HS_LS_Enabled = 2,
  filter360ns_HS_LS_Enabled = 3,
  filter480ns_HS_LS_Enabled = 4,
  filter480ns_HS_Disabled_LS_Enabled = 5,
  filter480ns_HS_Enabled_LS_Disabled = 6,
  filter480ns_HS_LS_Disabled = 7,
};
// eeprom18
enum class DriveGateSlew : uint8_t {
  level0 = 0,
  level1 = 1,
  level2 = 2,
};
// eeprom21
enum class SpeedInputOffThreshold : uint8_t {
  percent9p7 = 0,
  percent5p8 = 1,
  percent12p8 = 2,
  percent19p5 = 3,
};
// eeprom22
enum class VdsThereshold : uint8_t {
  v1 = 0,
  v2 = 1,
};
enum class LockRestartSet : uint8_t {
  sec5 = 0,
  sec10 = 1,
};
enum class RestartAttempt: uint8_t {
  always = 0,
  times3 = 1,
  times5 = 2,
  times10 = 3,
};
// eeprom23
enum class OperationMode : uint8_t {
  openLoop = 0,
  constantTorque = 1,
  constantSpeed = 2,
  constantPower = 3,
};
// eeprom25
enum class InductanceShift : uint8_t {
  x001 = 0,
  x002 = 1,
  x004 = 2,
  x008 = 3,
  x016 = 4,
  x032 = 5,
  x064 = 6,
  x128 = 7,
};

}  // namespace ValueTypes
}  // namespace A89306_asukiaaa
