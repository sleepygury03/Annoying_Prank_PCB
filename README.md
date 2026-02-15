# Annoying PCB

**Project status: Archived / Not actively maintained**

This was a personal experiment project.
I will likely not answer issues or pull requests.
Feel free to fork and modify.

---

## Description

A tiny ultra-low-power ATtiny814 based board that randomly beeps every 5–30 minutes… forever.

Press the button → short beep
Wait long enough → it beeps again

The project was mainly created as a low-power experiment (and as a harmless prank device).

---

## Features

* ATtiny814 (megaTinyCore)
* Random interval beeps (5–30 min, LFSR based)
* Button wake-up interrupt
* RTC standby sleep
* Very low standby current

---

## Hardware

Designed in **EasyEDA**

The circuit uses:

* ATtiny814
* Passive piezo buzzer
* Push button (pull-up)
* CR2032 battery
* Minimal passives

The MCU stays in standby sleep almost all the time and wakes via:

* RTC overflow
* Button interrupt

Unused pins are clamped low to reduce leakage current.

All hardware design files are released under the MIT license.

---

## Enclosure & Safety

The PCB is intended to be used inside a simple 3D-printed enclosure.

The PCB contains hole trough components, that can short the power supply if placed on metal object. 

This is mainly a precaution — the device runs from a CR2032 cell and is inherently current-limited, but the mechanical design reduces accidental shorting and improves robustness.

---

## Usage

* **LED** = power indicator
  The LED lights once at startup to confirm the device is running and the battery is OK.

* **Button** = test beep
  Press the button to manually trigger a short beep.

Typical use:
If you plan to hide the device somewhere, you only need to check that the LED flashes on power-up.
No need to press the button and reveal the sound location.

---

## Programming

Programmed using **Arduino + megaTinyCore**
UPDI via **jtag2updi (Arduino Nano programmer)**

### Required tools

* Arduino IDE
* megaTinyCore
* jtag2updi

### Board settings

Chip: ATtiny814
Clock: Internal (default)
Programmer: jtag2updi

### Required fuse configuration (important)

After selecting the board you must run Tools → Burn Bootloader once.

Used settings:

Clock: 1 MHz internal

BOD: Disabled (active and sleep)

WDT: Disabled
---

## Dependencies

This project uses external tools:

megaTinyCore
https://github.com/SpenceKonde/megaTinyCore

jtag2updi programmer
https://github.com/ElTangas/jtag2updi

They are not included in this repository.

Tested with:
megaTinyCore 2.6.12
Arduino IDE 2.3.7

---

## About the firmware

Large parts of the firmware were generated with the help of AI tools and then tested and adjusted on real hardware.

---

## License

MIT License — do whatever you want with it.
No warranty. Use at your own risk.
