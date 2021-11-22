#pragma once
#include <wire_asukiaaa.h>

#include "./A89306_asukiaaa/Registers.hpp"

namespace A89306_asukiaaa {

static const uint8_t deviceAddress = 0x55;

class Core {
 public:
  void setWire(TwoWire* wire) { this->wire = wire; }

  void begin() {
    if (wire == NULL) {
      wire = &Wire;
      Wire.begin();
    }
  }

  void printRegister(Stream* serial, uint8_t registerAddress) {
    uint8_t data[3];
    serial->print("read" + String(registerAddress) + ": ");
    auto result = readRegister(registerAddress, data);
    if (result != 0) {
      serial->println("Failed to read. Error:" + String(result));
      return;
    }
    printDataBytes(serial, data);
    serial->println();
  }

  int updateEEPROM(uint8_t eepromAddress, uint32_t data32t) {
    int result;
    using AddressControl = Registers::AddressControlEEPROM;

    // erase existing data
    result = writeRegister((uint8_t)AddressControl::Address, eepromAddress);
    if (result != 0) return result;
    result = writeRegister((uint8_t)AddressControl::Data, (uint32_t)0);
    if (result != 0) return result;
    result = writeRegister((uint8_t)AddressControl::Config, (uint32_t)3);
    if (result != 0) return result;
    delay(15);
    result = writeRegister((uint8_t)AddressControl::Config, (uint32_t)3);
    if (result != 0) return result;
    delay(15);

    // write new data
    result = writeRegister((uint8_t)AddressControl::Flag, (uint32_t)1);
    if (result != 0) return result;
    result = writeRegister((uint8_t)AddressControl::Address, eepromAddress);
    if (result != 0) return result;
    result = writeRegister((uint8_t)AddressControl::Data, data32t);
    if (result != 0) return result;
    result = writeRegister((uint8_t)AddressControl::Config, (uint32_t)5);
    if (result != 0) return result;
    delay(15);
    result = writeRegister((uint8_t)AddressControl::Flag, (uint32_t)0);
    return result;
  }

  int writeRegister(const Registers::EEPROMBase& eeprom) {
    return writeRegister(
        utils::buildAddressRegisterFromEPROM(eeprom.addressEEPROM),
        eeprom.toU32());
  }

  int writeRegister(uint8_t registerAdresses, uint32_t data32t) {
    // Serial.print("Wrirte" + String(registerAdresses));
    uint8_t data[] = {0, 0, 0};
    data[0] = data32t & 0xff;
    data[1] = (data32t >> 8) & 0xff;
    data[2] = (data32t >> 16) & 0xff;
    // for (uint8_t i = 0; i < 3; ++i) {
    //   auto index = 2 - i;
    //   Serial.print(' ');
    //   Serial.print(string_asukiaaa::padStart(String(data[index], BIN), 8,
    //   '0'));
    // }
    auto result = writeRegister(registerAdresses, data);
    // Serial.println(result == 0 ? " succeeded" : " failed");
    return result;
  }

  int printInfo(Stream* serial) {
    return handleShadowRegisters(6, 29, serial, &Core::printInfoEEPROM);
  }

  int printInterOperation(Stream* serial) {
    return handleShadowRegisters(32, 63, serial,
                                 &Core::printlnEEPROMAddressAndData);
  }

 private:
  TwoWire* wire = NULL;

  int handleShadowRegisters(uint8_t addressFrom, uint8_t addressLast,
                            Stream* serial,
                            void (Core::*handleData)(Stream* serial,
                                                     uint8_t addressEEPROM,
                                                     uint8_t data[3])) {
    uint8_t data[3];
    int result;
    for (int i = addressFrom; i <= addressLast; ++i) {
      uint8_t addressEEPROM = i;
      uint8_t addressShadowRegister =
          utils::buildAddressRegisterFromEPROM(addressEEPROM);
      result = readRegister(addressShadowRegister, data);
      if (result != 0) {
        if (serial != NULL) {
          serial->println("Failed to read info. Error" + String(result));
        }
        return result;
      }
      (this->*handleData)(serial, addressEEPROM, data);
    }
    return result;
  }

  void printInfoEEPROM(Stream* serial, uint8_t addressEEPROM, uint8_t data[3]) {
    serial->print("EEPROM");
    serial->print(string_asukiaaa::padNumStart(addressEEPROM, 2, '0'));
    serial->print(" ");
    printDataBytes(serial, data);
    serial->print(" ");
    Registers::EEPROMBase* eeprom = NULL;
    switch (addressEEPROM) {
      case 8:
        eeprom = new Registers::EEPROM8;
        break;
      case 9:
        eeprom = new Registers::EEPROM9;
        break;
      case 10:
        eeprom = new Registers::EEPROM10;
        break;
      case 11:
        eeprom = new Registers::EEPROM11;
        break;
      case 12:
        eeprom = new Registers::EEPROM12;
        break;
      case 13:
        eeprom = new Registers::EEPROM13;
        break;
      case 14:
        eeprom = new Registers::EEPROM14;
        break;
      case 15:
        eeprom = new Registers::EEPROM15;
        break;
      case 16:
        eeprom = new Registers::EEPROM16;
        break;
      case 17:
        eeprom = new Registers::EEPROM17;
        break;
      case 18:
        eeprom = new Registers::EEPROM18;
        break;
      case 19:
        eeprom = new Registers::EEPROM19;
        break;
      case 20:
        eeprom = new Registers::EEPROM20;
        break;
      case 21:
        eeprom = new Registers::EEPROM21;
        break;
      case 22:
        eeprom = new Registers::EEPROM22;
        break;
      case 23:
        eeprom = new Registers::EEPROM23;
        break;
      case 24:
        eeprom = new Registers::EEPROM24;
        break;
      case 25:
        eeprom = new Registers::EEPROM25;
        break;
      case 26:
        eeprom = new Registers::EEPROM26;
        break;
      // no data for eeprom 27
      case 28:
        eeprom = new Registers::EEPROM28;
        break;
      case 29:
        eeprom = new Registers::EEPROM29;
        break;
      case 31:
        eeprom = new Registers::EEPROM31;
        break;
    }
    if (eeprom != NULL) {
      eeprom->parse(data);
      eeprom->print(serial);
      delete eeprom;
    } else {
      serial->print("Parser unavairable");
    }
    serial->println();
  }

  void printlnEEPROMAddressAndData(Stream* serial, uint8_t addressEEPROM,
                                   uint8_t data[3]) {
    serial->print("EEPROM" +
                  string_asukiaaa::padNumStart(addressEEPROM, 2, '0') + " ");
    printDataBytes(serial, data);
    serial->println();
  }

  void printDataBytes(Stream* serial, uint8_t data[3]) {
    uint8_t index;
    for (uint8_t i = 0; i < 3; ++i) {
      if (i != 0) {
        serial->print(' ');
      }
      index = 2 - i;
      serial->print(
          string_asukiaaa::padStart(String(data[index], BIN), 8, '0'));
    }
    for (uint8_t i = 0; i < 3; ++i) {
      index = 2 - i;
      serial->print(' ');
      serial->print(string_asukiaaa::padNumStart(data[index], 3, ' '));
    }
  }

  int readRegister(uint8_t registerAddress, uint8_t data[3]) {
    auto result =
        wire_asukiaaa::readBytes(wire, deviceAddress, registerAddress, data, 3);
    auto tmp = data[0];
    data[0] = data[2];
    data[2] = tmp;
    return result;
  }

  int writeRegister(uint8_t registerAddress, uint8_t data[3]) {
    uint8_t dataToWrite[3];
    dataToWrite[0] = data[2];
    dataToWrite[1] = data[1];
    dataToWrite[2] = data[0];
    // Serial.print(
    //     "write EEPROM" +
    //     String(utils::buildAddressEEPROMFromRegister(registerAddress)));
    // for (int i = 0; i < 3; ++i) {
    //   Serial.print(' ');
    //   Serial.print(string_asukiaaa::padStart(String(data[i], BIN), 8, '0'));
    // }
    // Serial.println();
    auto result = wire_asukiaaa::writeBytes(wire, deviceAddress,
                                            registerAddress, dataToWrite, 3);
    return result;
  }
};
}  // namespace A89306_asukiaaa
