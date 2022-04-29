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
 * Version: 3.0.0
 * Date:    April 23, 2022
 */

#define VERSION_MAJOR 3  // Major version
#define VERSION_MINOR 0  // Minor version
#define VERSION_MAINT 0  // Maintenance version


#include <Arduino.h>
#include "src/Adc/Adc.h"
#include "src/Cli/Cli.h"
#include "src/Led/Led.h"
#include "src/Nvm/Nvm.h"
#include "src/Button/Button.h"


/*
 * Pin assignment
 */
#define NUM_APINS           2  // Number of analog pins in use
#define I_APIN       ADC_PIN2  // Analog pin for sensing the pump current draw
#define LEVEL_APIN   ADC_PIN3  // Analog pin connected to the onboard tank water level probe
#define MOSFET_PIN          9  // Digital output pin controlling the gate of the power MOSFET
#define LED_PIN            10  // Digital output pin controlling the status LED
#define BUTTON_PIN         11  // Digital input pin reading the push button


/*
 * Configuration parameters
 */
#define SAVE_LAST_VALUES            // Save last read current and level values to EEPROM
#define SERIAL_BAUD         115200  // Serial communication baud rate
#define ADC_AVG_SAMPLES        128  // Number of ADC samples to be averaged
#define ADC_I_FROST_THR       1000  // ADC current reading threshold for triggering frost protection
#define ADC_LEVEL_FULL_THR     500  // ADC reading threshold for detecting a full onboard tank
#define DEBOUNCE_SAMPLES        20  // Button debouncing level (ms until a button press is detected)
#define CAL_PRESS_DURATION    5000  // Long button press duration in ms to enter the calibration mode
#define APPLY_PRESS_DURATION  1000  // Long button press duration in ms to apply the calibration setting
#define MEAS_DURATION         1000  // Pump current measurement duration in ms
#define PUMP_TIMEOUT            10  // Time in minutes to exit the calibration mode if no button was pressed


/*
 * Global variables
 */
struct {
  enum {
    ERROR_E, ERROR,
    OFF_E, OFF,
    STANDBY_E, STANDBY,
    PUMP_E, PUMP,
    CAL_E, CAL_I_PUMP, CAL_I_DRY} state = OFF_E;  // Main state
  uint16_t iAdcVal;      // ADC pump current reading value
  uint16_t levelAdcVal;  // ADC water level probe reading value
  uint16_t mosfet;       // MOSFET switch state
  uint16_t iThrDry;      // Current threshold for dry run detection
} G;


/*
 * Parameters stored in EEPROM (non-volatile memory)
 */
struct {
  uint16_t iDry;       // ADC reading when the pump is running dry
  uint16_t iPump;      // ADC reading for regular pumping operation
                       // Note: iDry < iPump
  uint16_t iLast;      // Last registered current before switching to standy state
  uint16_t levelLast;  // Last registered onboard tank level value
} Nvm;


/*
 * Objects
 */
ButtonClass Button;
LedClass    Led;


/*
 * Function declarations
 */
void saveLastVal (void);
void nvmRead     (void);
void nvmWrite    (void);
void nvmValidate (void);
void mosfetOff   (void);
void mosfetOn    (void);
int  cmdShow     (int argc, char **argv);
int  cmdRom      (int argc, char **argv);
int  cmdSetNvm   (int argc, char **argv);



/*
 * Initialization routine
 */
void setup () {
  // clear MCU status register following a watchdog reset
  MCUSR = 0;

  pinMode      (MOSFET_PIN, OUTPUT);
  digitalWrite (MOSFET_PIN, LOW);
  pinMode      (LED_PIN, OUTPUT);
  digitalWrite (LED_PIN, LOW);
  pinMode      (BUTTON_PIN, INPUT_PULLUP);

  Cli.init ( SERIAL_BAUD );
  Serial.println ("");
  Serial.println (F("+ + +  S M A R T  P U M P  + + +"));
  Serial.println ("");
  Cli.xprintf    ("V %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  Serial.println ("");
  Serial.println (F("'h' for help"));
  Cli.newCmd     ("s"   , "Show real time readings"                  , cmdShow);
  Cli.newCmd     ("."   , ""                                         , cmdShow);
  Cli.newCmd     ("r"   , "Show the calibration data"                , cmdRom);
  Cli.newCmd     ("n"   , "Set NVM value (arg: <idx> <value>)"       , cmdSetNvm);

  AdcPin_t adcPins[NUM_APINS] = {I_APIN, LEVEL_APIN};
  Adc.initialize (ADC_PRESCALER_128, ADC_INTERNAL, ADC_AVG_SAMPLES, NUM_APINS, adcPins);
  Button.initialize (BUTTON_PIN, LOW, DEBOUNCE_SAMPLES);
  Led.initialize (LED_PIN);

  nvmRead ();
}


/*
 * Main loop
 */
void loop () {
  static uint32_t dryMeasTs      = 0;
  static uint32_t levelMeasTs    = 0;
  static uint32_t calTs          = 0;
  static uint32_t pumpTs         = 0;
  static uint32_t frostTs        = 0;
  uint32_t ts = millis ();

  Cli.getCmd ();
  Button.read ();
  Button.rising ();
  Led.loopHandler ();

  if ( G.state < G.CAL_E) {
    if (Button.longPress (CAL_PRESS_DURATION)) {
      G.state = G.CAL_E;
    }
  }
  else {
    if (Button.falling ()) {
      G.state = G.OFF_E;
      nvmValidate ();
    }
    if (ts - calTs > PUMP_TIMEOUT * 60000) {
      G.state = G.OFF_E;
      nvmValidate ();
    }
  }

  if (Adc.readAll ()) {
    G.iAdcVal = Adc.result[I_APIN];
    G.levelAdcVal = Adc.result[LEVEL_APIN];
  }

  // If iAdcVal exceeds ADC_I_FROST_THR for MEAS_DURATION,
  // then turn of the pump.
  if (G.iAdcVal < ADC_I_FROST_THR) {
    frostTs = ts;
  }
  else if (ts - frostTs > MEAS_DURATION) {
    G.state = G.OFF_E;
  }

  switch (G.state) {

    // ERROR: Wrong calibration data, please calibrate
    case G.ERROR_E:
      mosfetOff ();
      Led.blink (-1, 500, 100);
      G.state = G.ERROR;
    case G.ERROR:
      break;

    // OFF: No pumping until button is pressed
    case G.OFF_E:
      mosfetOff ();
      Led.turnOff ();
      Led.blink (1, 100, 0);
      G.state = G.OFF;
    case G.OFF:
      if (Button.falling()) {
        G.state = G.PUMP_E;
      }
      break;

    // STANDBY: Periodically activate pump to top-up the water tank
    case G.STANDBY_E:
      mosfetOff ();
      Led.blink (-1, 100, 1900);
      levelMeasTs = ts;
      G.state     = G.STANDBY;
    case G.STANDBY:
      if (Button.falling()) {
        G.state = G.OFF_E;
      }

      // If levelAdcVal exceeds ADC_LEVEL_FULL_THR for more than MEAS duration,
      // then start pumping. levelAdcVal increases with decreasing water level.
      if (G.levelAdcVal < ADC_LEVEL_FULL_THR) {
        levelMeasTs = ts;
      }
      else if (ts - levelMeasTs > MEAS_DURATION) {
        G.state = G.PUMP_E;
      }
      break;

    // PUMP: Pumping water
    case G.PUMP_E:
      mosfetOn ();
      Led.turnOn ();
      dryMeasTs   = ts;
      levelMeasTs = ts;
      pumpTs      = ts;
      G.state     = G.PUMP;
    case G.PUMP:
      if (Button.falling ()) {
        saveLastVal ();
        G.state = G.OFF_E;
      }

      if (G.iAdcVal > G.iThrDry) {
        dryMeasTs = ts;
      }
      else if (ts - dryMeasTs > MEAS_DURATION) {
        saveLastVal ();
        G.state = G.OFF_E;
      }

      if (G.levelAdcVal > ADC_LEVEL_FULL_THR) {
        levelMeasTs = ts;
      }
      else if (ts - levelMeasTs > MEAS_DURATION) {
        saveLastVal ();
        G.state = G.STANDBY_E;
      }

      if (ts - pumpTs > PUMP_TIMEOUT * 60000) {
        saveLastVal ();
        G.state = G.OFF_E;
      }
      break;

    // CAL: Calibration
    case G.CAL_E:
      mosfetOn ();
      Led.blink (-1, 500, 500);
      calTs = ts;
      G.state = G.CAL_I_PUMP;

    // Calibrate pumping current
    case G.CAL_I_PUMP:
      if (Button.longPress (APPLY_PRESS_DURATION)) {
        Nvm.iPump = G.iAdcVal;
        Led.blink (-1, 250, 250);
        calTs = ts;
        nvmWrite ();
        G.state = G.CAL_I_DRY;
      }
      break;

    // Calibrate dry run current
    case G.CAL_I_DRY:
      if (Button.longPress (APPLY_PRESS_DURATION)) {
        Nvm.iDry = G.iAdcVal;
        G.state = G.OFF_E;
        nvmWrite ();
      }
      break;
  }
}


/*
 * Save last I ADC value
 */
void saveLastVal (void) {
#ifdef SAVE_LAST_VALUES
  Nvm.iLast     = G.iAdcVal;
  Nvm.levelLast = G.levelAdcVal;
  nvmWrite ();
#endif
}


/*
 * Read EEPROM data
 */
void nvmRead (void) {
  eepromRead (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
  nvmValidate ();
}


/*
 * Write EEPROM data
 */
void nvmWrite (void) {
  eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
  nvmValidate ();
}


/*
 * Validate EEPROM data
 */
void nvmValidate (void) {
  if (Nvm.iDry >= Nvm.iPump - 100) G.state = G.ERROR_E;
  G.iThrDry  = (Nvm.iDry + Nvm.iPump) / 2;
}


/*
 * Turn off pump MOSFET
 */
void mosfetOff (void) {
  G.mosfet = LOW;
  digitalWrite (MOSFET_PIN, G.mosfet);
}


/*
 * Turn on pump MOSFET
 */
void mosfetOn (void) {
  G.mosfet = HIGH;
  digitalWrite (MOSFET_PIN, G.mosfet);
}


/*
 * CLI command for displaying the ADC reading
 */
int cmdShow (int argc, char **argv) {
  Cli.xprintf ("Realtime data:\n");
  Cli.xprintf ("State  = %u\n", G.state);
  Cli.xprintf ("MOSFET = %u\n", G.mosfet);
  Cli.xprintf ("I_adc  = %u\n", G.iAdcVal);
  Cli.xprintf ("L_adc  = %u\n", G.levelAdcVal);
  Serial.println ("");
  return 0;
}


/*
 * CLI command for displaying the calibration data
 */
int cmdRom (int argc, char **argv) {
  Cli.xprintf ("Calibration data:\n");
  Cli.xprintf ("I_dry      = %u\n", Nvm.iDry);
  Cli.xprintf ("I_pump     = %u\n", Nvm.iPump);
  Cli.xprintf ("I_thr_dry  = %u\n", G.iThrDry);
#ifdef SAVE_LAST_VALUES
  Cli.xprintf ("I_last     = %u\n", Nvm.iLast);
  Cli.xprintf ("L_last     = %u\n", Nvm.levelLast);
#endif
  Serial.println ("");
  return 0;
}


/*
 * Set an NVM value
 */
int cmdSetNvm (int argc, char **argv) {
  uint16_t idx;
  uint16_t val;
  uint16_t *ptr = (uint16_t *)&Nvm;
  if (argc == 3) {
    idx = atoi(argv[1]);
    val = atoi(argv[2]);
  }
  else {
    return 1;
  }
  ptr[idx] = val;
  Cli.xprintf ("Nvm[%u] = %u\n", idx, ptr[idx]);
  Serial.println ("");
  nvmWrite ();
  return 0;
}
