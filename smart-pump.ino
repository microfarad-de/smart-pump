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
 * Version: 2.1.0
 * Date:    April 22, 2022
 */

#define VERSION_MAJOR 2  // Major version
#define VERSION_MINOR 1  // Minor version
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
#define V_APIN       ADC_PIN3  // Analog pin for sensing the power supply voltage
#define MOSFET_PIN          9  // Digital output pin controlling the gate of the power MOSFET
#define LED_PIN            10  // Digital output pin controlling the status LED
#define BUTTON_PIN         11  // Digital input pin reading the push button


/*
 * Configuration parameters
 */
#define SERIAL_BAUD         115200  // Serial communication baud rate
#define ADC_AVG_SAMPLES        128  // Number of ADC samples to be averaged
#define ADC_V_CORRECTION       100  // Voltage correction factor (256 -> 1, 128 -> 0.5, 0 -> 0)
#define ADC_I_FROST_THR       1000  // ADC current reading threshold for triggering frost protection
#define DEBOUNCE_SAMPLES        20  // Button debouncing level (ms until a button press is detected)
#define CAL_PRESS_DURATION    5000  // Long button press duration in ms to enter the calibration mode
#define APPLY_PRESS_DURATION  1023  // Long button press duration in ms to apply the calibration setting
#define MEAS_DURATION         1000  // Pump current measurement duration in ms
#define PUMP_TIMEOUT            10  // Time in minutes to exit the calibration mode if no button was pressed
#define TOP_UP_INTERVAL         30  // Time in minutes between consecutive tank top-up attempts


/*
 * Global variables
 */
struct {
  enum {
    ERROR_E, ERROR,
    OFF_E, OFF,
    STANDBY_E, STANDBY,
    PUMP_E, PUMP,
    CAL_E, CAL_I_PUMP, CAL_I_FULL, CAL_I_DRY} state = OFF_E;  // Main state
  uint16_t iAdcVal;       // ADC pump current reading value
  uint16_t vAdcVal;       // ADC power supply voltage value
  uint16_t mosfet;        // MOSFET switch state
  uint16_t thrDry;        // Current threshold for dry run detection
  uint16_t thrFull;       // Current threshold for full tank detection
  uint16_t vAvgDry;       // Average power supply voltage at calibration time of the dry run threshold
  uint16_t vAvgFull;      // Average power supply voltage at calibration time of the full tank threshold
  uint16_t iAdcDryVal;    // Normalized ADC value for detecting the dry run condition
  uint16_t iAdcFullVal;   // Normalized ADC value for detecting the Full tank condition
  int16_t  vAdcDryCorr;   // Dry ADC voltage correction value
  int16_t  vAdcFullCorr;  // Full ADC voltage correction value
  uint16_t iAdcTest = 0;  // ADC current dummy test value
  uint16_t vAdcTest = 0;  // ADC voltage dummy test value
} G;


/*
 * Parameters stored in EEPROM (non-volatile memory)
 */
struct {
  uint16_t iDry;   // ADC reading when the pump is running dry
  uint16_t iFull;  // ADC reading when the pump water output is blocked
  uint16_t iPump;  // ADC reading for regular pumping operation
                   // Note: iDry < iFull < iPump
  uint16_t vDry;   // Power supply voltage at dry run calibration time
  uint16_t vFull;  // Power supply voltage at full calibration time
  uint16_t vPump;  // Power supply voltage at regular pumping calibration time
  uint16_t iLast;  // Last registered current before switching to standy state
  uint16_t vLast;  // Last registered voltage before switching to standy state
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
int  cmdTestI    (int argc, char **argv);
int  cmdTestV    (int argc, char **argv);
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
  Cli.newCmd     ("i"   , "Set current test value (arg: <value>)"    , cmdTestI);
  Cli.newCmd     ("v"   , "Set voltage test value (arg: <value>)"    , cmdTestV);
  Cli.newCmd     ("n"   , "Set NVM value (arg: <idx> <value>)"       , cmdSetNvm);

  AdcPin_t adcPins[NUM_APINS] = {I_APIN, V_APIN};
  Adc.initialize (ADC_PRESCALER_128, ADC_INTERNAL, ADC_AVG_SAMPLES, NUM_APINS, adcPins);
  Button.initialize (BUTTON_PIN, LOW, DEBOUNCE_SAMPLES);
  Led.initialize (LED_PIN);

  nvmRead ();
}


/*
 * Main loop
 */
void loop () {
  static uint32_t measTs         = 0;
  static uint32_t topupTs        = 0;
  static uint32_t calTs          = 0;
  static uint32_t pumpTs         = 0;
  static uint32_t frostTs        = 0;
  static uint16_t iAdcDryValMin  = 0;
  static uint16_t iAdcFullValMin = 0;
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
    if (G.iAdcTest == 0) G.iAdcVal = Adc.result[I_APIN];
    else                 G.iAdcVal = G.iAdcTest;

    if (G.vAdcTest == 0) G.vAdcVal = Adc.result[V_APIN];
    else                 G.vAdcVal = G.vAdcTest;

    G.vAdcDryCorr  = ((int32_t)G.vAdcVal - (int32_t)G.vAvgDry) * (int32_t)ADC_V_CORRECTION / 256;
    G.vAdcFullCorr = ((int32_t)G.vAdcVal - (int32_t)G.vAvgFull) * (int32_t)ADC_V_CORRECTION / 256;
    G.iAdcDryVal   = ((uint32_t)G.iAdcVal * (uint32_t)G.vAvgDry) / (G.vAdcVal + G.vAdcDryCorr);
    G.iAdcFullVal  = ((uint32_t)G.iAdcVal * (uint32_t)G.vAvgFull) / (G.vAdcVal + G.vAdcFullCorr);
  }

  if (G.iAdcFullVal < ADC_I_FROST_THR) frostTs = ts;
  if (ts - frostTs > MEAS_DURATION) {
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
      topupTs = ts;
      G.state = G.STANDBY;
    case G.STANDBY:
      if (Button.falling()) {
        G.state = G.OFF_E;
      }
      if (ts - topupTs > TOP_UP_INTERVAL * 60000) {
        G.state = G.PUMP_E;
      }
      break;

    // PUMP: Pumping water
    case G.PUMP_E:
      mosfetOn ();
      Led.turnOn ();
      measTs = ts;
      pumpTs = ts;
      G.state = G.PUMP;
    case G.PUMP:
      if (Button.falling()) {
        saveLastVal();
        G.state = G.OFF_E;
      }
      if (G.iAdcFullVal > G.thrFull) {
        measTs = ts;
        iAdcDryValMin  = G.iAdcDryVal;
        iAdcFullValMin = G.iAdcFullVal;
      }
      else if (ts - measTs > MEAS_DURATION) {
        if (iAdcDryValMin < G.thrDry) {
          saveLastVal();
          G.state = G.OFF_E;
        }
        else if (iAdcFullValMin < G.thrFull) {
          saveLastVal();
          G.state = G.STANDBY_E;
        }
      }
      else {
        if (G.iAdcDryVal < iAdcDryValMin)   iAdcDryValMin = G.iAdcDryVal;
        if (G.iAdcFullVal < iAdcFullValMin) iAdcFullValMin = G.iAdcFullVal;
      }
      if (ts - pumpTs > PUMP_TIMEOUT * 60000) {
        saveLastVal();
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
        Nvm.vPump = G.vAdcVal;
        Led.blink (-1, 250, 250);
        calTs = ts;
        nvmWrite ();
        G.state = G.CAL_I_FULL;
      }
      break;

    // Calibrate full current (water output is blocked)
    case G.CAL_I_FULL:
      if (Button.longPress (APPLY_PRESS_DURATION)) {
        Nvm.iFull = G.iAdcVal;
        Nvm.vFull = G.vAdcVal;
        Led.blink (-1, 125, 125);
        calTs = ts;
        nvmWrite ();
        G.state = G.CAL_I_DRY;
      }
      break;

    // Calibrate dry run current
    case G.CAL_I_DRY:
      if (Button.longPress (APPLY_PRESS_DURATION)) {
        Nvm.iDry = G.iAdcVal;
        Nvm.vDry = G.vAdcVal;
        G.state = G.OFF_E;
        nvmWrite ();
      }
      break;
  }
}


/*
 * Save last I and V ADC values
 */
void saveLastVal (void) {
  Nvm.iLast = G.iAdcVal;
  Nvm.vLast = G.vAdcVal;
  nvmWrite ();
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
  if (Nvm.iDry >= Nvm.iFull - 10)  G.state = G.ERROR_E;
  if (Nvm.iFull >= Nvm.iPump - 10) G.state = G.ERROR_E;
  G.thrDry   = (Nvm.iDry + Nvm.iFull) / 2;
  G.thrFull  = (Nvm.iFull + Nvm.iPump) / 2;
  G.vAvgDry  = (Nvm.vDry + Nvm.vFull) / 2;
  G.vAvgFull = (Nvm.vFull + Nvm.vPump) / 2;
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
  Cli.xprintf ("State      = %u\n", G.state);
  Cli.xprintf ("MOSFET     = %u\n", G.mosfet);
  Cli.xprintf ("I_adc      = %u\n", G.iAdcVal);
  Cli.xprintf ("I_adc_dry  = %u\n", G.iAdcDryVal);
  Cli.xprintf ("I_adc_full = %u\n", G.iAdcFullVal);
  Cli.xprintf ("V_adc      = %u\n", G.vAdcVal);
  Cli.xprintf ("V_cor_dry  = %d\n", G.vAdcDryCorr);
  Cli.xprintf ("V_cor_full = %d\n", G.vAdcFullCorr);
  Cli.xprintf ("I_test     = %u\n", G.iAdcTest);
  Cli.xprintf ("V_test     = %u\n", G.vAdcTest);
  Serial.println ("");
  return 0;
}


/*
 * CLI command for displaying the calibration data
 */
int cmdRom (int argc, char **argv) {
  Cli.xprintf ("Calibration data:\n");
  Cli.xprintf ("I_dry      = %u\n", Nvm.iDry);
  Cli.xprintf ("I_full     = %u\n", Nvm.iFull);
  Cli.xprintf ("I_pump     = %u\n", Nvm.iPump);
  Cli.xprintf ("I_thr_dry  = %u\n", G.thrDry);
  Cli.xprintf ("I_thr_full = %u\n", G.thrFull);
  Cli.xprintf ("V_dry      = %u\n", Nvm.vDry);
  Cli.xprintf ("V_full     = %u\n", Nvm.vFull);
  Cli.xprintf ("V_pump     = %u\n", Nvm.vPump);
  Cli.xprintf ("V_avg_dry  = %u\n", G.vAvgDry);
  Cli.xprintf ("V_avg_full = %u\n", G.vAvgFull);
  Cli.xprintf ("I_last     = %u\n", Nvm.iLast);
  Cli.xprintf ("V_last     = %u\n", Nvm.vLast);
  Serial.println ("");
  return 0;
}


/*
 * Set the ADC test current value
 */
int cmdTestI (int argc, char **argv) {
  uint16_t val;
  if (argc == 2) val = atoi(argv[1]);
  else           return 1;
  G.iAdcTest = val;
  Cli.xprintf ("I_test = %u\n", G.iAdcTest);
  Serial.println ("");
  return 0;
}


/*
 * Set the ADC test voltage value
 */
int cmdTestV (int argc, char **argv) {
  uint16_t val;
  if (argc == 2) val = atoi(argv[1]);
  else           return 1;
  G.vAdcTest = val;
  Cli.xprintf ("V_test = %u\n", G.vAdcTest);
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
