#include "IP5306.h"

IP5306::IP5306(TwoWire &wire, uint8_t address) : wire_(&wire), address_(address) {}

bool IP5306::begin() {
  uint8_t value = 0;
  return readRegister(REG_SYS_0, value);
}

uint8_t IP5306::address() const {
  return address_;
}

bool IP5306::getPowerMode(PowerMode &mode) const {
  uint8_t keyOff = 0;
  uint8_t boostOutput = 0;
  uint8_t powerOnLoad = 0;
  uint8_t charger = 0;
  uint8_t boost = 0;

  if (!readBits(REG_SYS_0, 0, 1, keyOff) ||
      !readBits(REG_SYS_0, 1, 1, boostOutput) ||
      !readBits(REG_SYS_0, 2, 1, powerOnLoad) ||
      !readBits(REG_SYS_0, 4, 1, charger) ||
      !readBits(REG_SYS_0, 5, 1, boost)) {
    return false;
  }

  if (boostOutput == 1 && powerOnLoad == 1 && charger == 1 && boost == 1) {
    mode = PowerMode::Normal;
  } else if (boostOutput == 0 && powerOnLoad == 0 && charger == 1 && boost == 0) {
    mode = PowerMode::Standby;
  } else if (boostOutput == 0 && powerOnLoad == 0 && charger == 0 && boost == 0) {
    mode = PowerMode::PowerDown;
  } else {
    mode = PowerMode::Unknown;
  }

  (void)keyOff;
  return true;
}

bool IP5306::setPowerMode(PowerMode mode) {
  switch (mode) {
    case PowerMode::Normal:
      return setNormalMode();
    case PowerMode::Standby:
      return setStandbyMode();
    case PowerMode::PowerDown:
      return setPowerDownMode();
    default:
      return false;
  }
}

bool IP5306::setNormalMode() {
  return updateBits(REG_SYS_0, 0, 1, 0) &&
         updateBits(REG_SYS_0, 1, 1, 1) &&
         updateBits(REG_SYS_0, 2, 1, 1) &&
         updateBits(REG_SYS_0, 4, 1, 1) &&
         updateBits(REG_SYS_0, 5, 1, 1);
}

bool IP5306::setStandbyMode() {
  return updateBits(REG_SYS_0, 0, 1, 0) &&
         updateBits(REG_SYS_0, 1, 1, 0) &&
         updateBits(REG_SYS_0, 2, 1, 0) &&
         updateBits(REG_SYS_0, 4, 1, 1) &&
         updateBits(REG_SYS_0, 5, 1, 0);
}

bool IP5306::setPowerDownMode() {
  return updateBits(REG_SYS_0, 0, 1, 1) &&
         updateBits(REG_SYS_0, 1, 1, 0) &&
         updateBits(REG_SYS_0, 2, 1, 0) &&
         updateBits(REG_SYS_0, 4, 1, 0) &&
         updateBits(REG_SYS_0, 5, 1, 0);
}

bool IP5306::setLongPressTime(bool threeSeconds) {
  return updateBits(REG_SYS_2, 4, 1, threeSeconds ? 1 : 0);
}

bool IP5306::getLongPressTime(bool &threeSeconds) const {
  uint8_t value = 0;
  if (!readBits(REG_SYS_2, 4, 1, value)) {
    return false;
  }
  threeSeconds = (value != 0);
  return true;
}

bool IP5306::setFlashlightOnLongPress(bool enable) {
  return updateBits(REG_SYS_1, 6, 1, enable ? 1 : 0);
}

bool IP5306::getFlashlightOnLongPress(bool &enable) const {
  uint8_t value = 0;
  if (!readBits(REG_SYS_1, 6, 1, value)) {
    return false;
  }
  enable = (value != 0);
  return true;
}

bool IP5306::setBoostOffOnLongPress(bool enable) {
  return updateBits(REG_SYS_1, 7, 1, enable ? 0 : 1);
}

bool IP5306::getBoostOffOnLongPress(bool &enable) const {
  uint8_t value = 0;
  if (!readBits(REG_SYS_1, 7, 1, value)) {
    return false;
  }
  enable = (value == 0);
  return true;
}

bool IP5306::disableLightLoadShutdown() {
  return setLightLoadShutdownTime(0);
}

bool IP5306::setLightLoadShutdownTime(uint8_t value) {
  return updateBits(REG_SYS_2, 2, 2, value);
}

bool IP5306::getLightLoadShutdownTime(uint8_t &value) const {
  return readBits(REG_SYS_2, 2, 2, value);
}

bool IP5306::readRegister(uint8_t reg, uint8_t &value) const {
  wire_->beginTransmission(address_);
  wire_->write(reg);

  if (wire_->endTransmission(false) != 0) {
    return false;
  }

  if (wire_->requestFrom(static_cast<int>(address_), 1) != 1) {
    return false;
  }

  value = wire_->read();
  return true;
}

bool IP5306::writeRegister(uint8_t reg, uint8_t value, bool verify) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(value);
  if (wire_->endTransmission(true) != 0) {
    return false;
  }

  if (!verify) {
    return true;
  }

  uint8_t readBack = 0;
  return readRegister(reg, readBack) && readBack == value;
}

bool IP5306::readBits(uint8_t reg, uint8_t bitIndex, uint8_t bitCount, uint8_t &value) const {
  if (bitCount == 0 || bitCount > 8 || bitIndex > 7 || (bitIndex + bitCount) > 8) {
    return false;
  }

  uint8_t regValue = 0;
  if (!readRegister(reg, regValue)) {
    return false;
  }

  const uint8_t mask = static_cast<uint8_t>((1U << bitCount) - 1U);
  value = static_cast<uint8_t>((regValue >> bitIndex) & mask);
  return true;
}

bool IP5306::updateBits(uint8_t reg, uint8_t bitIndex, uint8_t bitCount, uint8_t value) {
  if (bitCount == 0 || bitCount > 8 || bitIndex > 7 || (bitIndex + bitCount) > 8) {
    return false;
  }

  uint8_t regValue = 0;
  if (!readRegister(reg, regValue)) {
    return false;
  }

  const uint8_t mask = static_cast<uint8_t>(((1U << bitCount) - 1U) << bitIndex);
  regValue = static_cast<uint8_t>((regValue & ~mask) | ((value << bitIndex) & mask));
  return writeRegister(reg, regValue, true);
}
