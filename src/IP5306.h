#pragma once

#include <Arduino.h>
#include <Wire.h>

class IP5306 {
public:
  static constexpr uint8_t kDefaultAddress = 0x75;

  enum class PowerMode : uint8_t {
    Normal = 0,
    Standby = 1,
    PowerDown = 2,
    Unknown = 255,
  };

  static constexpr uint8_t REG_SYS_0 = 0x00;
  static constexpr uint8_t REG_SYS_1 = 0x01;
  static constexpr uint8_t REG_SYS_2 = 0x02;
  static constexpr uint8_t REG_CHG_0 = 0x20;
  static constexpr uint8_t REG_CHG_1 = 0x21;
  static constexpr uint8_t REG_CHG_2 = 0x22;
  static constexpr uint8_t REG_CHG_3 = 0x23;
  static constexpr uint8_t REG_CHG_4 = 0x24;
  static constexpr uint8_t REG_READ_0 = 0x70;
  static constexpr uint8_t REG_READ_1 = 0x71;
  static constexpr uint8_t REG_READ_2 = 0x72;
  static constexpr uint8_t REG_READ_3 = 0x77;
  static constexpr uint8_t REG_READ_4 = 0x78;

  explicit IP5306(TwoWire &wire = Wire, uint8_t address = kDefaultAddress);

  bool begin();
  uint8_t address() const;

  bool getPowerMode(PowerMode &mode) const;
  bool setPowerMode(PowerMode mode);
  bool setNormalMode();
  bool setStandbyMode();
  bool setPowerDownMode();
  bool setLongPressTime(bool threeSeconds);
  bool getLongPressTime(bool &threeSeconds) const;
  bool setFlashlightOnLongPress(bool enable);
  bool getFlashlightOnLongPress(bool &enable) const;
  bool setBoostOffOnLongPress(bool enable);
  bool getBoostOffOnLongPress(bool &enable) const;
  bool disableLightLoadShutdown();
  bool setLightLoadShutdownTime(uint8_t value);
  bool getLightLoadShutdownTime(uint8_t &value) const;

  bool readRegister(uint8_t reg, uint8_t &value) const;
  bool writeRegister(uint8_t reg, uint8_t value, bool verify = true);
  bool readBits(uint8_t reg, uint8_t bitIndex, uint8_t bitCount, uint8_t &value) const;
  bool updateBits(uint8_t reg, uint8_t bitIndex, uint8_t bitCount, uint8_t value);

private:
  TwoWire *wire_;
  uint8_t address_;
};
