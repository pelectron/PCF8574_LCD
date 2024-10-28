/// \file PCF8574_LCD.hpp
/// \brief LiquidCrystal library with PCF8574 I2C adapter.
///
/// \author Matthias Hertel, http://www.mathertel.de
/// \author Pelé Constam, pelectron@github.com
///
/// \copyright Copyright (c) 2019 by Matthias Hertel.\n
///
/// The library work is licensed under a BSD style license.\n
/// See http://www.mathertel.de/License.aspx
///
/// \details
/// This library can drive a Liquid Crystal Display (LCD) based on the Hitachi
/// HD44780 chip that is connected through a PCF8574 I2C adapter. It uses the
/// original Wire library for communication. The API if common to many LCD
/// libraries and documented in
/// https://www.arduino.cc/en/Reference/LiquidCrystal. and partially functions
/// from https://playground.arduino.cc/Code/LCDAPI/.

///
/// ChangeLog:
/// --------
/// * 19.10.2013 created.
/// * 05.06.2019 rewrite from scratch.
/// * 26.05.2022 8-bit datatypes in interfaces and compatibility topics.
/// * 26.05.2022 createChar with PROGMEM character data for AVR processors.
/// * 26.05.2022 constructor with pin assignments. Thanks to @markisch.
/// * 28.10.2024 Implemented the C++17 version.

#ifndef PCF8574_LCD_HPP
#define PCF8574_LCD_HPP

#include <cstddef>
#include <cstdint>

namespace PCF8574 {

template <size_t NumRows, size_t NumCols> class LCD {
public:
  /// The function used to write data to an ic peripheral with address addr.
  using I2C_Write_t = void (*)(uint8_t addr, uint8_t data);
  /// The function used to delay for a certain amount of microseconds
  using Delay_t = void (*)(unsigned microSeconds);

  /**
   * Construct an LCD with default pinout.
   * @param write_func the function used to transmit I2C data
   * @param delay_microseconds the function used to delay for a certain duration
   * in microseconds
   * @param i2cAddr the i2c address of the PCF8574. This is passed to write_func
   * as is.
   */
  LCD(I2C_Write_t write_func, Delay_t delay_microseconds, uint8_t i2cAddr);

  /**
   * Construct an LCD with default pinout.
   * @param write_func the function used to transmit I2C data
   * @param delay_microseconds the function used to delay for a certain duration
   * in microseconds
   * @param i2cAddr the i2c address of the PCF8574. This is passed to write_func
   * as is.
   * @param backlight the backlight intensity (0-255)
   * @param rest the PCF8574 output bit to which the corresponding pin is
   * connected, i.e. if enable is output on bit 1 then enable should be set
   * to 1.
   */
  LCD(I2C_Write_t i2c_write_func, Delay_t delay_microseconds, uint8_t i2cAddr,
      uint8_t rs, uint8_t rw, uint8_t enable, uint8_t d4, uint8_t d5,
      uint8_t d6, uint8_t d7, uint8_t backlight);

  /**
   * Initializes the LCD. Must be called before using any other LCD function.
   */
  void init();

  /// clear the display
  void clear();

  /// set the cursor to the home position
  void home();

  /// set the cursor to the specified column and row.
  void set_cursor(uint8_t col, uint8_t row);

  /// show the cursor
  void enable_cursor();

  /// hide the cursor
  void disable_cursor();

  /// enable the cursor blinking
  void enable_blinking();

  /// disable the cursor blinking
  void disable_blinking();

  /// show display text
  void display_on();

  /// hide display text
  void display_off();

  void scroll_display_left();

  void scroll_display_right();

  void enable_autoscroll();

  void disable_autoscroll();

  void left_to_right();

  void right_to_left();

  // plus functions from LCDAPI:
  void setBacklight(uint8_t brightness);

  /// write a single character
  void write(uint8_t c);
  /// write a single character
  void write(char c);

  /// write a null terminated string
  void write(const uint8_t *str);
  /// write a null terminated string
  void write(const char *str);

  /// write a (possibly null terminated) string of length size.
  void write(const uint8_t *str, size_t size);
  /// write a (possibly null terminated) string of length size.
  void write(const char *str, size_t size);

private:
  static constexpr uint8_t row_offsets[4]{0x00, 0x40, 0x00 + NumCols,
                                          0x40 + NumCols};

  // instance variables
  I2C_Write_t i2c_write; //< pointer to function which writes to the i2c bus
  Delay_t delay_microseconds;
  uint8_t i2c_addr_;       ///< Wire Address of the LCD
  uint8_t backlight_;      ///< the backlight intensity
  uint8_t entrymode_;      ///< flags from entrymode
  uint8_t displaycontrol_; ///< flags from displaycontrol

  // variables describing how the PCF8574 is connected to the LCD
  uint8_t rs_mask_;
  uint8_t rw_mask_;
  uint8_t enable_mask_;
  uint8_t backlight_mask_;
  // these are used for 4-bit data to the display.
  uint8_t data_mask_[4];

  // low level functions
  /// write either command or data
  void _send(uint8_t value, bool isData = false);

  /// write a nibble / halfByte with handshake
  void _sendNibble(uint8_t halfByte, bool isData = false);

  /// write a nibble / halfByte with handshake
  void _writeNibble(uint8_t halfByte, bool isData);

  /// private function to change the PCF8574 pins to the given value
  void _write2Wire(uint8_t data, bool isData, bool enable);
};

template <size_t NumRows, size_t NumCols>
LCD<NumRows, NumCols>::LCD(I2C_Write_t write_func, Delay_t delay_microseconds,
                           uint8_t i2cAddr)
    : LCD(write_func, delay_microseconds, i2cAddr, 0, 1, 2, 4, 5, 6, 7, 3) {}

template <size_t NumRows, size_t NumCols>
LCD<NumRows, NumCols>::LCD(I2C_Write_t i2c_write_func,
                           Delay_t delay_microseconds, uint8_t i2cAddr,
                           uint8_t rs, uint8_t rw, uint8_t enable, uint8_t d4,
                           uint8_t d5, uint8_t d6, uint8_t d7,
                           uint8_t backlight)
    : i2c_write(i2c_write_func), delay_microseconds(delay_microseconds),
      i2c_addr_(i2cAddr) {
  backlight_ = backlight;

  entrymode_ = 0x02; // like Initializing by Internal Reset Circuit
  displaycontrol_ = 0x04;

  rs_mask_ = 0x01 << rs;
  if (rw != 255)
    rw_mask_ = 0x01 << rw;
  else
    rw_mask_ = 0;
  enable_mask_ = 0x01 << enable;
  data_mask_[0] = 0x01 << d4;
  data_mask_[1] = 0x01 << d5;
  data_mask_[2] = 0x01 << d6;
  data_mask_[3] = 0x01 << d7;

  if (backlight != 255)
    backlight_mask_ = 0x01 << backlight;
  else
    backlight_mask_ = 0;
}

template <size_t NumRows, size_t NumCols> void LCD<NumRows, NumCols>::init() {
  uint8_t functionFlags = 0;

  if (NumRows > 1) {
    functionFlags |= 0x08;
  }

  // initializing the display
  _write2Wire(0x00, 0, false);
  delay_microseconds(50000);

  // after reset the mode is this
  displaycontrol_ = 0x04;
  entrymode_ = 0x02;

  // sequence to reset. see "Initializing by Instruction" in datasheet
  _sendNibble(0x03);
  delay_microseconds(4500);
  _sendNibble(0x03);
  delay_microseconds(200);
  _sendNibble(0x03);
  delay_microseconds(200);
  _sendNibble(0x02); // finally, set to 4-bit interface

  // Instruction: Function set = 0x20
  _send(0x20 | functionFlags);

  display();
  clear();
  leftToRight();
}

template <size_t NumRows, size_t NumCols> void LCD<NumRows, NumCols>::clear() {
  // Instruction: Clear display = 0x01
  _send(0x01);
  delay_microseconds(1600); // this command takes 1.5ms!
}

template <size_t NumRows, size_t NumCols> void LCD<NumRows, NumCols>::home() {
  // Instruction: Return home = 0x02
  _send(0x02);
  delay_microseconds(1600); // this command takes 1.5ms!
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::set_cursor(uint8_t col, uint8_t row) {
  // check boundaries
  if ((col < NumCols) && (row < NumRows)) {
    // Instruction: Set DDRAM address = 0x80
    _send(0x80 | (row_offsets[row] + col));
  }
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::enable_cursor() {
  // Instruction: Display on/off control = 0x08
  displaycontrol_ |= 0x02; // cursor
  _send(0x08 | displaycontrol_);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::disable_cursor() {
  // Instruction: Display on/off control = 0x08
  displaycontrol_ &= ~0x02; // cursor
  _send(0x08 | displaycontrol_);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::enable_blinking() {
  // Instruction: Display on/off control = 0x08
  displaycontrol_ |= 0x01; // blink
  _send(0x08 | displaycontrol_);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::disable_blinking() {
  // Instruction: Display on/off control = 0x08
  displaycontrol_ &= ~0x01; // blink
  _send(0x08 | displaycontrol_);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::display_on() {
  // Instruction: Display on/off control = 0x08
  displaycontrol_ |= 0x04; // display
  _send(0x08 | displaycontrol_);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::display_off() {
  // Instruction: Display on/off control = 0x08
  displaycontrol_ &= ~0x04; // display
  _send(0x08 | displaycontrol_);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::scroll_display_left() {
  // Instruction: Cursor or display shift = 0x10
  // shift: 0x08, left: 0x00
  _send(0x10 | 0x08 | 0x00);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::scroll_display_right() {
  // Instruction: Cursor or display shift = 0x10
  // shift: 0x08, right: 0x04
  _send(0x10 | 0x08 | 0x04);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::enable_autoscroll() {
  // Instruction: Entry mode set, set shift S=0x01
  entrymode_ |= 0x01;
  _send(0x04 | entrymode_);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::disable_autoscroll() {
  // Instruction: Entry mode set, clear shift S=0x01
  entrymode_ &= ~0x01;
  _send(0x04 | entrymode_);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::left_to_right() {
  // Instruction: Entry mode set, set increment/decrement =0x02
  entrymode_ |= 0x02;
  _send(0x04 | entrymode_);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::right_to_left() {
  // Instruction: Entry mode set, clear increment/decrement =0x02
  entrymode_ &= ~0x02;
  _send(0x04 | entrymode_);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::setBacklight(uint8_t brightness) {
  backlight_ = brightness;
  // send no data but set the background-pin right;
  _write2Wire(0x00, true, false);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::write(uint8_t c) {
  _send(c, true);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::write(char c) {
  this->write(static_cast<uint8_t>(c));
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::write(const uint8_t *str) {
  while (*str != 0) {
    write(*str);
    ++str;
  }
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::write(const char *str) {
  while (*str != 0) {
    write(*str);
    ++str;
  }
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::write(const uint8_t *str, size_t size) {
  for (size_t i = 0; i < size; ++i) {
    write(str[i]);
  }
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::write(const char *str, size_t size) {
  for (size_t i = 0; i < size; ++i) {
    write(str[i]);
  }
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::_send(uint8_t value, bool isData = false) {
  // An I2C transmission has a significant overhead of ~10+1 I2C clock
  // cycles. We consequently only perform it only once per _send().

  // write high 4 bits
  _writeNibble(((value >> 4) & 0x0F), isData);
  // write low 4 bits
  _writeNibble((value & 0x0F), isData);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::_sendNibble(uint8_t halfByte, bool isData = false) {
  _writeNibble(halfByte, isData);
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::_writeNibble(uint8_t halfByte, bool isData) {
  // map the data to the given pin connections
  uint8_t data = isData ? rs_mask_ : 0;
  // _rw_mask is not used here.
  if (backlight_ > 0)
    data |= backlight_mask_;

  // allow for arbitrary pin configuration
  if (halfByte & 0x01)
    data |= data_mask_[0];
  if (halfByte & 0x02)
    data |= data_mask_[1];
  if (halfByte & 0x04)
    data |= data_mask_[2];
  if (halfByte & 0x08)
    data |= data_mask_[3];

  // Note that the specified speed of the PCF8574 chip is 100KHz.
  // Transmitting a single byte takes 9 clock ticks at 100kHz -> 90us.
  // The 37us delay is only necessary after sending the second nibble.
  // But in that case we have to restart the transfer using additional
  // >10 clock cycles. Hence, no additional delays are necessary even
  // when the I2C bus is operated beyond the chip's spec in fast mode
  // at 400 kHz.

  i2c_write(i2c_addr_, data | enable_mask_);
  // delay_microseconds(1); // enable pulse must be >450ns
  i2c_write(i2c_addr_, data);
  // delay_microseconds(37); // commands need > 37us to settle
}

template <size_t NumRows, size_t NumCols>
void LCD<NumRows, NumCols>::_write2Wire(uint8_t data, bool isData,
                                        bool enable) {
  if (isData)
    data |= rs_mask_;
  // _rw_mask is not used here.
  if (enable)
    data |= enable_mask_;
  if (backlight_ > 0)
    data |= backlight_mask_;

  i2c_write(i2c_addr_, data);
}

} // namespace PCF8574
#endif /* INC_LCD_PCF8574_HPP_ */
