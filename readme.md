# PCF8574 LCD Library

This is a single header library for character based displays using the HD44780 (or compatible) LCD driver in 4 bit mode in combination with the PCF8574 I2C expander. It is based on [Matthias Hertels Arduino library](https://github.com/mathertel/LiquidCrystal_PCF8574). The code from the arduino library has been modified to use C++17 and to be MCU agnostic.

## How to use

Simply download the single header file and incorporate it into your project. Afterwards, include the header where you want to use it.


To use the driver, two functions must be implemented:
- one to write over the I2C bus
- one to delay for a certain amount of microseconds

### I2C write function

An I2C write function must be implemented, which transmits a single data byte over I2C to the specified address. Given below is an implementation using the STM32 I2C HAL.

```{.cpp}
void lcd_i2c_write(uint8_t addr, uint8_t data){
    // hi2c1 is the handle used, addr must be shifted to the
    // left by one because that is what the HAL expects
    HAL_I2C_Master_Transmit(&hi2c1, addr<<1, &data, 1, 1000);
}
```

### Delay function

The delay function takes a single unsigned integer as it's only parameter, which is the amount of microsends to delay.
The example below uses the STM32 HAL. If your HAL cannot delay microseconds, as is the case for the STM32 HAL_Delay which only works with milliseconds, simply round up the the smallest delay possible which  fullfills the time requirements.

```{.cpp}
void lcd_delay_microseconds(unsigned us){
    HAL_Delay(us/1000 + 1);
}
```

### Creating the display driver

To contruct a display driver, simply include the header and pass the I2C write and delay function, as well as the i2c address of the PCF8574 to the constructor. If your PCF8574 has a different connection diagram, use the second constructor which exposes all bit position parameters. The example below shows how to contruct a driver for a 4x20 Display using the write and delay functions from above.

```{.cpp}
#include "PCF8574_LCD.hpp"

static PCF8574::LCD<4,20> lcd {&lcd_i2c_write,&lcd_delay_microseconds, 0x27};
```

### Initialization

To initialize the driver properly, its ``init()`` function must be called before using any other method, i.e. somewhere in main, after your I2C peripheral has been initialized, you should call

```{.cpp}
    lcd.init();
```

### Printing to the screen

Writing character data is simple with the various ``write()`` overloads.

``display_on/off()`` control if text is shown or hidden.

All text can be cleared with ``clear()``.

Control of the cursors position is done through the ``home()`` and ``set_cursor()`` methods. Cursor blinking is controlled with ``enable/disable_blinking()``.

Example:

```{.cpp}
lcd.init();
lcd.write("Hello World"); // lcd shows "Hello World" on the first row
lcd.display_off(); // screen no shows nothing
lcd.display_on(); // scrren show "Hello World" again
lcd.clear(); // screen is cleared -> shows nothing

lcd.set_cursor(0, 1); // cursor is now on column 0 on the second row
lcd.enable_blinking(); // cursor now blinks
lcd.write("Hello World 2"); // lcd now shows "Hello World 2" on the second row
lcd.disable_blinking(); // cursor blinking stopped
```