#pragma once

namespace A89306_asukiaaa {
namespace utils {

uint32_t buildTrueBit(uint8_t position) { return ((uint32_t)0b1) << position; }

bool isBitTrue(uint8_t byte, uint8_t bitPosition) {
  uint8_t targetBit = (0b1 << bitPosition);
  return (byte & targetBit) != 0;
}

uint8_t buildAddressRegisterFromEPROM(uint8_t addressEEPROM) {
  return addressEEPROM + 64;
};

uint8_t buildAddressEEPROMFromRegister(uint8_t addressRegister) {
  return addressRegister - 64;
};

}  // namespace utils
}  // namespace A89306_asukiaaa
