/*
 *-----------------------------------------------------------------------------
 *
 * Test_ADS1115_Pyranometer.ino: Test for modified version of the InstESRE
 *                               pyranometer
 *
 * Copyright (C) 2019  Chris Satterlee
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
 *-----------------------------------------------------------------------------
 *
 * This is a standalone test of a modified InstESRE pyranometer.  The
 * hardware modifications are:
 *
 *    - PDB-C139 photodiode leads connected to pins A0 and A1 of an
 *      ADS1115 analog-to-digital converter (ADC)
 *
 *    - ADS1115 connected to Arduino:
 *        - ADS1115 VDD pin connected to Arduino +5V
 *        - ADS1115 GND pin connected to Arduino GND
 *        - ADS1115 SCL pin connected to Arduino SCL
 *        - ADS1115 SDA pin connected to Arduino SDA
 *
 *    - (optional) TMP36 temperature sensor connected to ADS1115
 *        - TMP36 pin 1 connected to ADS1115 VDD pin
 *        - TMP36 pin 2 connected to ADS1115 pin A2
 *        - TMP36 pin 3 connected to ADS1115 GND pin
 *
 * This sketch should be run with the Arduino IDE's Serial Monitor open,
 * and with its baud rate set to 57600.
 *
 *
 */
#include <stdio.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

// Test behavior constants (may be modified)
#define SERIAL_BAUD 57600               // Serial port baud rate
#define CSV_OUTPUT false                // Set to true for CSV
#define CSV_MV_ONLY false               // Set to true for mV only in CSV
#define IRRADIANCE_POLLING_LOOPS 10
#define TEMP_POLLING_LOOPS 5
#define REQUIRE_STABLE_TEMP true
#define MAX_STABLE_TEMP_ERR_PPM 5000    // 5000 = 0.5%
#define REQUIRE_STABLE_IRRAD true
#define MAX_STABLE_IRRAD_ERR_PPM 10000  // 10000 = 1%
#define MS_DELAY_BETWEEN_LOOPS 250
#define INVALID_TEMP -9999.0

// ADS1115 and TMP36 constants (do not modify)
#define ADS1115_UNITY_GAIN_MAX_MILLIVOLTS 4096
#define ADS1115_NON_SIGN_BITS 15
#define ADS1115_MIN_VALUE ((long)1 << ADS1115_NON_SIGN_BITS)
#define ADS1115_MAX_VALUE (((long)1 << ADS1115_NON_SIGN_BITS) - 1)
#define ADS1115_PGA_GAIN_TMP36 2
#define ADS1115_PGA_GAIN_PDB_C139 8
#define TMP36_OFFSET_MILLIVOLTS 500     // from datasheet
#define TMP36_MV_PER_DEG_C 10           // from datasheet

// Calibration constants (may be modified)
#define PYRANO_CAL 4.3                // X coefficient (slope if A=0): W/m^2/mV
#define PYRANO_CAL_A 0.0              // X^2 coefficient: W/m^2/mV^2
#define PHOTODIODE_NOMINAL_DEG_C 25.0
#define PHOTODIODE_PCT_PER_DEG_C 0.16 // determined empirically, YMMV

Adafruit_ADS1115 ads1115;

float photodiode_temp_scaling_factor = 1.0;

void setup()
{
  Serial.begin(SERIAL_BAUD);
  ads1115.begin();
  if (!CSV_OUTPUT) {
    Serial.println(
      F("------------------------------------------------------------------"));
    Serial.println(
      F("Format of results:"));
    Serial.println(
      F(""));
    Serial.println(
      F("   1) If TMP36 is found:"));
    Serial.println(
      F(""));
    Serial.println(
      F("      Irradiance: <1> W/m^2 @ <2> deg C   (<3>)"));
    Serial.println(
      F(""));
    Serial.println(
      F("          <1>:  irradiance WITH temperature adjustment"));
    Serial.println(
      F("          <2>:  TMP36 temperature"));
    Serial.println(
      F("          <3>:  irradiance WITHOUT temperature adjustment"));
    Serial.println(
      F(""));
    Serial.println(
      F("   2) If TMP36 is not found:"));
    Serial.println(
      F(""));
    Serial.println(
      F("      Irradiance: <1> W/m^2"));
    Serial.println(
      F(""));
    Serial.println(
      F("          <1>:  irradiance WITHOUT temperature adjustment"));
    Serial.println(
      F("------------------------------------------------------------------"));
    Serial.println(F(""));
  }
}

void loop()
{
  int16_t ads1115_val;
  long ads1115_val_sum, ads1115_val_avg, ppm_error_from_avg;
  bool ads1115_present, tmp36_present, found_stable_value;
  float photodiode_temp;

  // Pyranometer temperature (TMP36)
  ads1115.setGain(GAIN_TWO);  // -2 V to 2 V
  ads1115_val_sum = 0;
  ads1115_val_avg = 0;
  ads1115_present = true;
  tmp36_present = true;
  found_stable_value = false;
  while (!found_stable_value) {
    for (int ii = 0; ii < TEMP_POLLING_LOOPS; ii++) {
      ads1115_val = ads1115.readADC_SingleEnded(2);
      if (ads1115_val == -1) {
        // Value of -1 indicates no ADS1115
        ads1115_present = false;
        found_stable_value = true;
        break;
      }
      if (ads1115_val < 4000) {
        // Values less than 250mV (-25 deg C) are assumed to be noise,
        // meaning there is no TMP36 connected to A2
        tmp36_present = false;
        found_stable_value = true;
        break;
      }
      ads1115_val_sum += ads1115_val;
    }
    if (ads1115_present && tmp36_present) {
      ads1115_val_avg = ads1115_val_sum / TEMP_POLLING_LOOPS;
      found_stable_value = true;
      ads1115_val_sum = 0;
      if (REQUIRE_STABLE_TEMP) {
        // If REQUIRE_STABLE_TEMP is true, loop again checking each
        // value for its deviation from the average. If any exceeds
        // MAX_STABLE_TEMP_ERR_PPM, the value is not stable, so retry
        // the whole thing.
        for (int ii = 0; ii < TEMP_POLLING_LOOPS; ii++) {
          ads1115_val = ads1115.readADC_SingleEnded(2);
          ppm_error_from_avg =
            (1000000 * abs(ads1115_val - ads1115_val_avg)) /
            abs(ads1115_val_avg);
          if (ppm_error_from_avg > MAX_STABLE_TEMP_ERR_PPM) {
            // If any value is more than MAX_STABLE_TEMP_ERR_PPM from
            // the average, we don't have a stable value
            if (!CSV_OUTPUT) {
              photodiode_temp = convert_ads1115_val_to_deg_c(ads1115_val_avg);
              Serial.print(F("TMP36 value unstable: "));
              Serial.println(photodiode_temp);
              //Serial.print(F("Average A2 value: "));
              //Serial.print(ads1115_val_avg);
              //Serial.print(F("   Bad A2 value: "));
              //Serial.println(ads1115_val);
            }
            found_stable_value = false;
            break;
          }
        }
      }
    }
  }
  photodiode_temp = convert_ads1115_val_to_deg_c(ads1115_val_avg);

  // Irradiance (PDB-C139)
  if (ads1115_present) {
    ads1115.setGain(GAIN_EIGHT); // -512 mV to 512 mV
    ads1115_val_sum = 0;
    ads1115_val_avg = 0;
    found_stable_value = false;
    while (!found_stable_value) {
      for (int ii = 0; ii < IRRADIANCE_POLLING_LOOPS; ii++) {
        ads1115_val = ads1115.readADC_Differential_0_1();
        ads1115_val_sum += ads1115_val;
      }
      ads1115_val_avg = ads1115_val_sum / IRRADIANCE_POLLING_LOOPS;
      found_stable_value = true;
      ads1115_val_sum = 0;
      // If REQUIRE_STABLE_IRRAD is true, loop again checking each
      // value for its deviation from the average. If any exceeds
      // MAX_STABLE_IRRAD_ERR_PPM, the value is not stable, so retry
      // the whole thing.
      if (REQUIRE_STABLE_IRRAD) {
        for (int ii = 0; ii < IRRADIANCE_POLLING_LOOPS; ii++) {
          ads1115_val = ads1115.readADC_Differential_0_1();
          ppm_error_from_avg =
            (1000000 * abs(ads1115_val - ads1115_val_avg)) /
            abs(ads1115_val_avg);
          if (ppm_error_from_avg > MAX_STABLE_IRRAD_ERR_PPM) {
            // If any value is more than MAX_STABLE_IRRAD_ERR_PPM from
            // the average, we don't have a stable value
            if (!CSV_OUTPUT) {
              Serial.println(F("Irradiance value unstable"));
              //Serial.print(F("Average A0/A1 value: "));
              //Serial.print(ads1115_val_avg);
              //Serial.print(F("   ppm_error_from_avg: : "));
              //Serial.print(ppm_error_from_avg);
              //Serial.print(F("   Bad A0/A1 value: "));
              //Serial.println(ads1115_val);
            }
            found_stable_value = false;
            break;
          }
        }
      }
    }
    // Print results
    if (CSV_OUTPUT) {
      print_results_csv(photodiode_temp, ads1115_val_avg);
    } else {
      print_results(photodiode_temp, ads1115_val_avg);
    }
  } else {
    Serial.println(F("ADS1115 not found"));
  }

  // Delay before next iteration
  delay(MS_DELAY_BETWEEN_LOOPS);
}

void print_results(float photodiode_temp, long ads1115_val) {
  Serial.print(F("Irradiance: "));
  if (photodiode_temp != INVALID_TEMP) {
    Serial.print(convert_ads1115_val_to_irradiance(ads1115_val, true));
    Serial.print(F(" W/m^2 @ "));
    Serial.print(photodiode_temp);
    Serial.print(F(" deg C   ("));
    Serial.print(convert_ads1115_val_to_irradiance(ads1115_val, false));
    Serial.println(F(")"));
  } else {
    Serial.print(convert_ads1115_val_to_irradiance(ads1115_val, false));
    Serial.println(F(" W/m^2"));
  }
}

void print_results_csv(float photodiode_temp, long ads1115_val) {
  if (photodiode_temp != INVALID_TEMP) {
    Serial.print(photodiode_temp);
    Serial.print(F(","));
    if (CSV_MV_ONLY) {
      Serial.println(convert_ads1115_val_to_millivolts(ads1115_val));
    } else {
      Serial.print(convert_ads1115_val_to_irradiance(ads1115_val, true));
      Serial.print(F(","));
      Serial.print(convert_ads1115_val_to_irradiance(ads1115_val, false));
    }
  } else {
    if (CSV_MV_ONLY) {
      Serial.println(convert_ads1115_val_to_millivolts(ads1115_val));
    } else {
      Serial.print(convert_ads1115_val_to_irradiance(ads1115_val, false));
    }
  }
}

float convert_ads1115_val_to_deg_c(int16_t ads1115_val) {
  int max_millivolts;
  float tmp36_millivolts;
  float deg_c, temp_diff, multiplier;

  if (ads1115_val == 0) {
    return INVALID_TEMP; 
  }

  // First convert reading value to millivolts
  max_millivolts = (ADS1115_UNITY_GAIN_MAX_MILLIVOLTS /
                    ADS1115_PGA_GAIN_TMP36);
  tmp36_millivolts =
    (max_millivolts * (float)ads1115_val) / ADS1115_MIN_VALUE;

  // Then convert millivolts to degrees C
  deg_c = ((tmp36_millivolts - TMP36_OFFSET_MILLIVOLTS) /
           TMP36_MV_PER_DEG_C);

  // Calculate photodiode scaling factor
  temp_diff = deg_c - PHOTODIODE_NOMINAL_DEG_C;
  multiplier = PHOTODIODE_PCT_PER_DEG_C / 100;
  photodiode_temp_scaling_factor = temp_diff * multiplier + 1.0;

  // Return value
  return deg_c;
}

float convert_ads1115_val_to_irradiance(int16_t ads1115_val, bool temp_comp) {
  int max_millivolts;
  float photodiode_millivolts;
  float temp_scaling;
  float scaled_photodiode_millivolts;
  float adjusted_millivolts;
  float w_per_m_squared;

  // First convert reading value to millivolts
  photodiode_millivolts = convert_ads1115_val_to_millivolts(ads1115_val);

  // Conditionally apply temperature scaling
  if (temp_comp) {
    temp_scaling = photodiode_temp_scaling_factor;
  } else {
    temp_scaling = 1.0;
  }
  scaled_photodiode_millivolts =
    photodiode_millivolts * temp_scaling;

  // Then convert millivolts to irradiance
  //
  // Polynomial curve:
  //
  //   y = Ax^2 + Bx
  //
  //   x: scaled_photodiode_millivolts
  //   A: PYRANO_CAL_A
  //   B: PYRANO_CAL
  //   y: w_per_m_squared (irradiance)
  //
  // If A is 0, scaling is linear.  Intercept is always zero.
  //
  w_per_m_squared = ((PYRANO_CAL_A *
                      scaled_photodiode_millivolts *
                      scaled_photodiode_millivolts) +
                     (PYRANO_CAL *
                      scaled_photodiode_millivolts));

  return w_per_m_squared;
}

float convert_ads1115_val_to_millivolts(int16_t ads1115_val) {
  int max_millivolts;
  float photodiode_millivolts;

  max_millivolts = (ADS1115_UNITY_GAIN_MAX_MILLIVOLTS /
                    ADS1115_PGA_GAIN_PDB_C139);
  photodiode_millivolts =
    max_millivolts * (float(abs(ads1115_val)) / ADS1115_MIN_VALUE);

  return photodiode_millivolts;
}
