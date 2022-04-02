/*
 * Smart Water Pump
 *
 * Smart water pump controller.
 *
 * This sketch has been implemented and tested on an ATMega328P based Arduino Pro Mini
 * compatible board running on 3.3V/8MHz.
 *
 * It is recommended to activate the watchdog support on the Arduino bootloader
 * by defining the WATCHDOG_MODS macro. This will reduce the bootloader's power-up
 * delay, thus invalidating the need to hold the power button for around 2 seconds for
 * the system to turn on.
 *
 * This source file is part of the follwoing repository:
 * http://www.github.com/microfarad-de/smart-pump
 *
 * Please visit:
 *   http://www.microfarad.de
 *   http://www.github.com/microfarad-de
 *
 * Copyright (C) 2022 Karim Hraibi (khraibi@gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Version: 1.0.0
 * Date:    March 2022
 */
#define VERSION_MAJOR 1  // Major version
#define VERSION_MINOR 0  // Minor version
#define VERSION_MAINT 0  // Maintenance version

#include <Arduino.h>
#include "src/Adc/Adc.h"
#include "src/Cli/Cli.h"
#include "src/Nvm/Nvm.h"
#include "src/Button/Button.h"


/*
 * Pin assignment
 */
#define NUM_APINS           1  // Number of analog pins in use
#define CURRENT_APIN ADC_PIN2  // Analog pin for sensing the pump current draw
#define OUTPUT_PIN          9  // Digital output pin controlling the gate of the power MOSFET
#define LED_PIN            10  // Digital output pin controlling the status LED
#define BUTTON_PIN         11  // Digital input pin reading the push button


/*
 * Configuration parameters
 */
#define SERIAL_BAUD      115200  // Serial communication baud rate
#define ADC_AVG_SAMPLES       1  // Number of ADC samples to be averaged


/*
 * Global variables
 */
struct {
  uint32_t reserved;
} G;


/*
 * Parameters stored in EEPROM (non-volatile memory)
 */
struct {
  uint16_t dryRunCurrent;     // ADC reading when the pump is running dry
  uint16_t pumpingCurrent;    // ADC reading for regular pumping operation
  uint16_t fullCurrent;       // ADC reading when the pump water output is blocked
} Nvm;


/*
 * Function declarations
 */
void nvmRead      (void);
void nvmWrite     (void);


/*
 * Initialization routine
 */
void setup () {
  // clear MCU status register following a watchdog reset
  MCUSR = 0;

  pinMode      (OUTPUT_PIN, OUTPUT);
  digitalWrite (OUTPUT_PIN, LOW);
  pinMode      (LED_PIN, OUTPUT);
  digitalWrite (LED_PIN, LOW);
  pinMode      (BUTTON_PIN, INPUT);
  //pinMode      (LED_BUILTIN, OUTPUT);
  //digitalWrite (LED_BUILTIN, HIGH);

  Cli.init ( SERIAL_BAUD );
  Serial.println ("");
  Serial.println (F("+ + +  S M A R T  P U M P  + + +"));
  Serial.println ("");
  Cli.xprintf    ("V %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  Serial.println ("");
  Serial.println (F("'h' for help"));
  //Cli.newCmd     ("cal" , "Calibrate (arg: <0|25|50|75|100> [value])", cmdCalibrate);
  //Cli.newCmd     ("s"   , "Show real time readings"                  , cmdShow);
  //Cli.newCmd     ("."   , ""                                         , cmdShow);
  //Cli.newCmd     ("r"   , "Show the calibration data"                , cmdRom);

  AdcPin_t adcPins[NUM_APINS] = {CURRENT_APIN};
  Adc.initialize (ADC_PRESCALER_128, ADC_INTERNAL, ADC_AVG_SAMPLES, NUM_APINS, adcPins);

  nvmRead ();
}


/*
 * Main loop
 */
void loop () {

  Cli.getCmd ();

}


/*
 * Read EEPROM data
 */
void nvmRead (void) {
  eepromRead (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
}


/*
 * Write EEPROM data
 */
void nvmWrite (void) {
  eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
}

