/*
    Box Mod Vape v3.1
      - improved calculations
    Author: Ihor Chaban
    Feb 2020
*/

#include <ArduinoSTL.h>
#include <EEPROMex.h>
#include <LowPower.h>
#include <map>
#include <OneButton.h>
#include <TimerOne.h>
#include <TM74HC595Display.h>

// Software settings
#define INITIAL_CALIBRATION 0
#define BATTERY_PERCENTAGE  1
#define STANDBY_TIME        60000
#define LOCK_TIME           3000
#define FIRE_LIMIT_TIME     5000
#define FIRE_FREQUENCY      20000
#define DISPLAY_FREQUENCY   30
#define VOLTS_STEP          0.05
#define WATTS_STEP          1
#define AMPS_STEP           1
#define OHMS_STEP           0.005
#define F_B_DEBOUNCE_TIME   50

// Battery settings
#define BATTERY_MIN         2800
#define BATTERY_MAX         4200
#define BATTERY_RESISTANCE  0.015

// Hardware settings
#define FIRE_BUTTON_PIN     2
#define MODE_BUTTON_PIN     3
#define DOWN_BUTTON_PIN     4
#define UP_BUTTON_PIN       5
#define SCLK_PIN            6
#define RCLK_PIN            7
#define DIO_PIN             8
#define MOSFET_POWER_PIN    10
#define DISPLAY_POWER_PIN   13

// System settings
#define VALUES_UPD_INTERVAL 50
#define VOLTAGE_ARR_SIZE    10
#define PWM_ARR_SIZE        10
#define VCC_CONST_POSITION  (0)
#define MODE_POSITION       (VCC_CONST_POSITION + sizeof(vcc_const))
#define VOLT_POSITION       (MODE_POSITION + sizeof(byte))
#define WATT_POSITION       (VOLT_POSITION + sizeof(volt))
#define AMP_POSITION        (WATT_POSITION + sizeof(watt))
#define OHM_POSITION        (AMP_POSITION + sizeof(amp))

OneButton fire_button(FIRE_BUTTON_PIN, true);
OneButton mode_button(MODE_BUTTON_PIN, true);
OneButton down_button(DOWN_BUTTON_PIN, true);
OneButton up_button(UP_BUTTON_PIN, true);
TM74HC595Display disp(SCLK_PIN, RCLK_PIN, DIO_PIN);

int amp, voltage, watt;
float vcc_const, volt, ohm;
byte last_fire_mode, last_setting_mode;
unsigned long standby_time, fire_time, values_update_time, f_b_debounce_time;
bool settings_mode, f_b_state, f_b_last_state, f_b_reading, sleeping, allow_fire;
word voltage_array[VOLTAGE_ARR_SIZE], PWM_array[PWM_ARR_SIZE], PWM, voltage_drop;
std::map <char, byte> symbols;
std::map <byte, char*> display_shortcuts;

enum Modes {VARIVOLT, VARIWATT, HELL, AMP, OHM} mode;
enum DisplayShortcuts {LOWB = (OHM + 1), BYE, FIRE1, FIRE2, V___, VA__, VAP_, VAPE};
Modes operator++(Modes& i, int) {
  if (!settings_mode) {
    if (i >= HELL) {
      return i = VARIVOLT;
    }
    byte temp = i;
    return i = static_cast<Modes> (++temp);
  } else {
    if (i >= OHM) {
      return i = AMP;
    } else {
      return i = OHM;
    }
  }
}

void setup() {
  pinMode(FIRE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOSFET_POWER_PIN, OUTPUT);
  pinMode(DISPLAY_POWER_PIN, OUTPUT);
  DisableAllFire();
  digitalWrite(DISPLAY_POWER_PIN, HIGH);
  mode_button.attachClick(ChangeMode);
  mode_button.attachDuringLongPress(ShowVoltage);
  mode_button.attachDoubleClick(ChangeSettingsMode);
  down_button.attachClick(ReduceValue);
  down_button.attachDuringLongPress(ReduceValueL);
  up_button.attachClick(IncreaseValue);
  up_button.attachDuringLongPress(IncreaseValueL);
  fire_button.attachDoubleClick(SleepPuzzle);
  InitDisplaySymbols();
  InitDisplayShortcuts();
  InitTimer2();
  Timer1.initialize(round(1000000.0 / FIRE_FREQUENCY));
  DisplaySlide(display_shortcuts[VAPE], false);
  vcc_const = EEPROM.readFloat(VCC_CONST_POSITION);
  if (!vcc_const) {
    vcc_const = 1.1;
  }
  if (INITIAL_CALIBRATION) {
    Calibration();
  }
  mode = EEPROM.readByte(MODE_POSITION);
  volt = EEPROM.readFloat(VOLT_POSITION);
  watt = EEPROM.readByte(WATT_POSITION);
  amp = EEPROM.readByte(AMP_POSITION);
  ohm = EEPROM.readFloat(OHM_POSITION);
  voltage = GetVoltage();
  last_fire_mode = mode;
  last_setting_mode = AMP;
  if (voltage < BATTERY_MIN) {
    DisplaySlide(display_shortcuts[LOWB], false);
    GoodNight();
  }
  standby_time = millis();
}

void loop() {
  CheckButtons();
  if (!sleeping) {
    f_b_reading = !digitalRead(FIRE_BUTTON_PIN);
    if (millis() - values_update_time >= VALUES_UPD_INTERVAL) {
      values_update_time = millis();
      MeasureVoltage();
      voltage = GetVoltage();
      switch (mode) {
        case VARIVOLT: {
            volt = constrain(volt, 0, (voltage - voltage_drop) / 1000.0);
            voltage_drop = round((volt * BATTERY_RESISTANCE * 1000.0) / (ohm + BATTERY_RESISTANCE));
            AddPWM(map(volt * 1000, 0, voltage, 0, 1023));
            break;
          }
        case VARIWATT: {
            watt = constrain(watt, 0, round(pow(voltage - voltage_drop, 2) / ohm / 1000000.0));
            voltage_drop = round(sqrt(ohm * watt) * BATTERY_RESISTANCE * 1000.0 / ohm);
            AddPWM(map(sqrt(ohm * watt) * 1000, 0, voltage, 0, 1023));
            break;
          }
        case HELL: {
            voltage_drop = round((voltage * BATTERY_RESISTANCE) / (ohm + BATTERY_RESISTANCE));
            break;
          }
        default: {
            voltage_drop = 0;
            break;
          }
      }
      PWM = GetPWM();
      if (!mode_button.isLongPressed() && !allow_fire) {
        ShowMainScreen();
      }
    }
    if (f_b_reading != f_b_last_state) {
      f_b_debounce_time = millis();
    }
    if (millis() - f_b_debounce_time >= F_B_DEBOUNCE_TIME) {
      if (f_b_reading != f_b_state) {
        f_b_state = f_b_reading;
        if (f_b_state == HIGH) {
          allow_fire = true;
          fire_time = millis();
        }
      }
    }
    f_b_last_state = f_b_reading;
    if (f_b_state && !f_b_last_state) {
      f_b_state = false;
      allow_fire = false;
      DisableAllFire();
    }
    if (allow_fire) {
      if ((mode == VARIVOLT || mode == VARIWATT) && (ohm > 0) && (PWM > 0)) {
        Timer1.pwm(MOSFET_POWER_PIN, PWM);
        ShowFireAnimation();
      } else if (mode == HELL && ohm > 0) {
        digitalWrite(MOSFET_POWER_PIN, HIGH);
        ShowFireAnimation();
      }
      standby_time = millis();
    }
    if (allow_fire && millis() - fire_time >= FIRE_LIMIT_TIME) {
      allow_fire = false;
      DisableAllFire();
    }
    if (millis() - standby_time >= STANDBY_TIME) {
      DisplaySlide(display_shortcuts[BYE], false);
      GoodNight();
    }
    if (voltage < BATTERY_MIN) {
      DisableAllFire();
      DisplaySlide(display_shortcuts[LOWB], false);
      GoodNight();
    }
  } else {
    WakePuzzle();
  }
}

void ReduceValue() {
  standby_time = millis();
  switch (mode) {
    case VARIVOLT: {
        if (ohm > 0) {
          volt -= VOLTS_STEP;
          volt = round(volt / VOLTS_STEP) * VOLTS_STEP;
          volt = constrain(volt, 0, (voltage - voltage_drop) / 1000.0);
        } else {
          volt = 0;
        }
        break;
      }
    case VARIWATT: {
        if (ohm > 0) {
          watt -= WATTS_STEP;
          watt = round(watt / WATTS_STEP) * WATTS_STEP;
          watt = constrain(watt, 0, round(pow(voltage - voltage_drop, 2) / ohm / 1000000.0));
        } else {
          watt = 0;
        }
        break;
      }
    case AMP: {
        amp -= AMPS_STEP;
        amp = round(amp / AMPS_STEP) * AMPS_STEP;
        amp = constrain(amp, 0, 100);
        break;
      }
    case OHM: {
        if (amp > 0) {
          ohm -= OHMS_STEP;
          ohm = round(ohm / OHMS_STEP) * OHMS_STEP;
          ohm = constrain(ohm, BATTERY_MAX / (amp * 1000.0), 1);
        } else {
          ohm = 0;
        }
        break;
      }
  }
}

void ReduceValueL() {
  static unsigned long values_longpress_timer = millis();
  if (millis() - values_longpress_timer >= 100) {
    values_longpress_timer = millis();
    ReduceValue();
  }
}

void IncreaseValue() {
  standby_time = millis();
  switch (mode) {
    case VARIVOLT: {
        if (ohm > 0) {
          volt += VOLTS_STEP;
          volt = round(volt / VOLTS_STEP) * VOLTS_STEP;
          volt = constrain(volt, 0, (voltage - voltage_drop) / 1000.0);
        } else {
          volt = 0;
        }
        break;
      }
    case VARIWATT: {
        if (ohm > 0) {
          watt += WATTS_STEP;
          watt = round(watt / WATTS_STEP) * WATTS_STEP;
          watt = constrain(watt, 0, round(pow(voltage - voltage_drop, 2) / ohm / 1000000.0));
        } else {
          watt = 0;
        }
        break;
      }
    case AMP: {
        amp += AMPS_STEP;
        amp = round(amp / AMPS_STEP) * AMPS_STEP;
        amp = constrain(amp, 0, 100);
        break;
      }
    case OHM: {
        if (amp > 0) {
          ohm += OHMS_STEP;
          ohm = round(ohm / OHMS_STEP) * OHMS_STEP;
          ohm = constrain(ohm, BATTERY_MAX / (amp * 1000.0), 1);
        } else {
          ohm = 0;
        }
        break;
      }
  }
}

void IncreaseValueL() {
  static unsigned long values_longpress_timer = millis();
  if (millis() - values_longpress_timer >= 100) {
    values_longpress_timer = millis();
    IncreaseValue();
  }
}

void ChangeMode() {
  standby_time = millis();
  mode++;
  if (!settings_mode) {
    last_fire_mode = mode;
  } else {
    last_setting_mode = mode;
  }
  ShowModeTitle();
}

void ChangeSettingsMode() {
  standby_time = millis();
  if (!settings_mode) {
    last_fire_mode = mode;
  } else {
    last_setting_mode = mode;
  }
  settings_mode = !settings_mode;
  if (!settings_mode) {
    mode = last_fire_mode;
  } else {
    mode = last_setting_mode;
  }
  ShowModeTitle();
}

void DisableAllFire() {
  Timer1.disablePwm(MOSFET_POWER_PIN);
  digitalWrite(MOSFET_POWER_PIN, LOW);
}

void ResetVoltage() {
  for (byte i = 0; i < VOLTAGE_ARR_SIZE; i++) {
    voltage_array[i] = 0;
  }
}

void MeasureVoltage() {
  for (byte i = 0; i < VOLTAGE_ARR_SIZE - 1; i++) {
    voltage_array[i] = voltage_array[i + 1];
  }
  voltage_array[VOLTAGE_ARR_SIZE - 1] = ReadVCC();
}

int GetVoltage() {
  word sum = 0;
  while (voltage_array[0] == 0) {
    MeasureVoltage();
  }
  for (byte i = 0; i < VOLTAGE_ARR_SIZE; i++) {
    sum += voltage_array[i];
  }
  return round(sum / (float)VOLTAGE_ARR_SIZE);
}

void AddPWM(word value) {
  for (byte i = 0; i < PWM_ARR_SIZE - 1; i++) {
    PWM_array[i] = PWM_array[i + 1];
  }
  PWM_array[PWM_ARR_SIZE - 1] = value;
}

word GetPWM() {
  word sum = 0;
  for (byte i = 0; i < PWM_ARR_SIZE; i++) {
    sum += PWM_array[i];
  }
  return round(sum / (float)PWM_ARR_SIZE);
}

void GoodNight() {
  DisableAllFire();
  sleeping = true;
  disp.clear();
  delay(5);
  EEPROM.updateByte(MODE_POSITION, last_fire_mode);
  EEPROM.updateFloat(VOLT_POSITION, volt);
  EEPROM.updateByte(WATT_POSITION, watt);
  EEPROM.updateByte(AMP_POSITION, amp);
  EEPROM.updateFloat(OHM_POSITION, ohm);
  attachInterrupt(digitalPinToInterrupt(FIRE_BUTTON_PIN), WakeUp, FALLING);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void WakeUp() {
  detachInterrupt(FIRE_BUTTON_PIN);
}

void SleepPuzzle() {
  DisableAllFire();
  unsigned long wake_timer = millis();
  unsigned long display_upd_timer = millis();
  bool sleeping = 0;
  byte click_count = 2;
  while (millis() - wake_timer < LOCK_TIME && !sleeping) {
    f_b_reading = digitalRead(FIRE_BUTTON_PIN);
    if (f_b_reading != f_b_last_state) {
      f_b_debounce_time = millis();
    }
    if (millis() - f_b_debounce_time >= F_B_DEBOUNCE_TIME) {
      if (f_b_reading != f_b_state) {
        f_b_state = f_b_reading;
        if (f_b_state) {
          click_count++;
        }
      }
    }
    f_b_last_state = f_b_reading;
    if (millis() - display_upd_timer >= VALUES_UPD_INTERVAL) {
      display_upd_timer = millis();
      disp.clear();
      switch (click_count) {
        case 1: DisplayPrint(display_shortcuts[V___]);
          break;
        case 2: DisplayPrint(display_shortcuts[VA__]);
          break;
        case 3: DisplayPrint(display_shortcuts[VAP_]);
          break;
        case 4: DisplayPrint(display_shortcuts[VAPE]);
          break;
      }
    }
    if (click_count > 4) {
      sleeping = true;
    }
  }
  if (sleeping) {
    GoodNight();
  }
}

void WakePuzzle() {
  unsigned long wake_timer = millis();
  unsigned long display_upd_timer = millis();
  bool allow_wakeup = 0;
  byte click_count = 0;
  while ((millis() - wake_timer < LOCK_TIME) && !allow_wakeup) {
    f_b_reading = digitalRead(FIRE_BUTTON_PIN);
    if (f_b_reading != f_b_last_state) {
      f_b_debounce_time = millis();
    }
    if (millis() - f_b_debounce_time >= F_B_DEBOUNCE_TIME) {
      if (f_b_reading != f_b_state) {
        f_b_state = f_b_reading;
        if (f_b_state) {
          click_count++;
        }
      }
    }
    f_b_last_state = f_b_reading;
    if (millis() - display_upd_timer >= VALUES_UPD_INTERVAL) {
      display_upd_timer = millis();
      disp.clear();
      switch (click_count) {
        case 1: DisplayPrint(display_shortcuts[V___]);
          break;
        case 2: DisplayPrint(display_shortcuts[VA__]);
          break;
        case 3: DisplayPrint(display_shortcuts[VAP_]);
          break;
        case 4: DisplayPrint(display_shortcuts[VAPE]);
          break;
      }
    }
    if (click_count > 4) {
      allow_wakeup = true;
    }
  }
  if (allow_wakeup) {
    sleeping = false;
    mode = last_fire_mode;
    settings_mode = false;
    standby_time = millis();
    ResetVoltage();
    voltage = GetVoltage();
  } else {
    GoodNight();
  }
}

void ShowMainScreen() {
  disp.clear();
  switch (mode) {
    case VARIVOLT: {
        disp.set(symbols[display_shortcuts[VARIVOLT][0]], 3);
        disp.float_dot(volt, 2);
        break;
      }
    case VARIWATT: {
        disp.set(symbols[display_shortcuts[VARIWATT][0]], 3);
        disp.digit4(watt);
        break;
      }
    case HELL: {
        disp.set(symbols[display_shortcuts[HELL][0]], 3);
        disp.digit4(round((voltage - voltage_drop) / 1000.0 / ohm * (voltage - voltage_drop) / 1000.0));
        break;
      }
    case AMP: {
        disp.set(symbols[display_shortcuts[AMP][0]], 3);
        disp.digit4(amp);
        break;
      }
    case OHM: {
        disp.set((ohm < 1) ? (symbols[display_shortcuts[OHM][0]] & 0x7F) : (symbols[display_shortcuts[OHM][0]]), 3);
        if (ohm < 0.01) {
          disp.set(symbols['O'], 2);
          disp.set(symbols['O'], 1);
          disp.digit2(ohm * 1000, 0);
        } else if (ohm < 0.1) {
          disp.set(symbols['O'], 2);
          disp.digit2(ohm * 1000, 0);
        } else if (ohm < 1) {
          disp.digit4(ohm * 1000, 0);
        } else {
          disp.float_dot(ohm, 2);
        }
      }
      break;
  }
}

void ShowModeTitle() {
  disp.clear();
  switch (mode) {
    case VARIVOLT: {
        DisplaySlide(display_shortcuts[VARIVOLT], true);
        break;
      }
    case VARIWATT: {
        DisplaySlide(display_shortcuts[VARIWATT], true);
        break;
      }
    case HELL: {
        DisplaySlide(display_shortcuts[HELL], true);
        break;
      }
    case AMP: {
        DisplaySlide(display_shortcuts[AMP], true);
        break;
      }
    case OHM: {
        DisplaySlide(display_shortcuts[OHM], true);
        break;
      }
  }
}

void ShowVoltage() {
  static unsigned long voltage_upd_time;
  standby_time = millis();
  if (millis() - voltage_upd_time >= VALUES_UPD_INTERVAL) {
    voltage_upd_time = millis();
    disp.clear();
    disp.set(symbols['b'], 3);
    if (BATTERY_PERCENTAGE) {
      disp.digit4(map(constrain(voltage, BATTERY_MIN + voltage_drop, BATTERY_MAX), BATTERY_MIN + voltage_drop, BATTERY_MAX, 0, 100));
    } else {
      disp.float_dot(voltage / 1000.0, 2);
    }
  }
}

void ShowFireAnimation() {
  if (round(millis() / 100) % 2 == 0) {
    DisplayPrint(display_shortcuts[FIRE1]);
  } else {
    DisplayPrint(display_shortcuts[FIRE2]);
  }
}

void DisplayPrint(char text[]) {
  for (byte i = 0; i < 4; i++) {
    disp.set(symbols[text[i]], 3 - i);
  }
}

void DisplaySlide(char text[], bool keep_first) {
  disp.clear();
  for (byte i = 0; i < 4; i++) {
    disp.set(symbols[text[i]], 3 - i);
    delay(100);
  }
  for (byte i = 0; i < ((keep_first) ? 3 : 4); i++) {
    disp.set(symbols[' '], i);
    delay(100);
  }
}

void InitDisplaySymbols() {
  symbols['b'] = 0x83;
  symbols['t'] = 0x87;
  symbols['A'] = 0x88;
  symbols['C'] = 0xC6;
  symbols['E'] = 0x86;
  symbols['H'] = 0x89;
  symbols['I'] = 0xF9;
  symbols['L'] = 0xC3;
  symbols['M'] = 0x88;
  symbols['O'] = 0xC0;
  symbols['P'] = 0x8C;
  symbols['S'] = 0x92;
  symbols['Y'] = 0x91;
  symbols['V'] = 0xC1;
  symbols['W'] = 0x81;
  symbols['-'] = 0xBF;
  symbols['='] = 0xF6;
  symbols[' '] = 0xFF;
}

void InitDisplayShortcuts() {
  char c_bye_[] = {'b', 'Y', 'E', ' '};
  char c_volt[] = {'V', 'O', 'L', 't'};
  char c_watt[] = {'W', 'A', 't', 't'};
  char c_hell[] = {'H', 'E', 'L', 'L'};
  char c_amps[] = {'A', 'M', 'P', 'S'};
  char c_coil[] = {'C', 'O', 'I', 'L'};
  char c_lowb[] = {'L', 'O', 'W', 'b'};
  char c_fir1[] = {'-', '=', '-', '='};
  char c_fir2[] = {'=', '-', '=', '-'};
  char c_v___[] = {'V', ' ', ' ', ' '};
  char c_va__[] = {'V', 'A', ' ', ' '};
  char c_vap_[] = {'V', 'A', 'P', ' '};
  char c_vape[] = {'V', 'A', 'P', 'E'};
  display_shortcuts.insert(std::pair <byte, char*> (BYE, c_bye_));
  display_shortcuts.insert(std::pair <byte, char*> (VARIVOLT, c_volt));
  display_shortcuts.insert(std::pair <byte, char*> (VARIWATT, c_watt));
  display_shortcuts.insert(std::pair <byte, char*> (HELL, c_hell));
  display_shortcuts.insert(std::pair <byte, char*> (AMP, c_amps));
  display_shortcuts.insert(std::pair <byte, char*> (OHM, c_coil));
  display_shortcuts.insert(std::pair <byte, char*> (LOWB, c_lowb));
  display_shortcuts.insert(std::pair <byte, char*> (FIRE1, c_fir1));
  display_shortcuts.insert(std::pair <byte, char*> (FIRE2, c_fir2));
  display_shortcuts.insert(std::pair <byte, char*> (V___, c_v___));
  display_shortcuts.insert(std::pair <byte, char*> (VA__, c_va__));
  display_shortcuts.insert(std::pair <byte, char*> (VAP_, c_vap_));
  display_shortcuts.insert(std::pair <byte, char*> (VAPE, c_vape));
}

void CheckButtons() {
  fire_button.tick();
  mode_button.tick();
  up_button.tick();
  down_button.tick();
}

void Calibration() {
  Serial.begin(9600);
  Serial.println("Current VCC constant is " + String(vcc_const, 5));
  Serial.println("Current voltage is " + String(ReadVCC()) + String(" mV"));
  Serial.println();
  Serial.print("Real voltage is (in millivolts): ");
  while (Serial.available() == 0);
  word real_vcc = Serial.parseInt();
  Serial.println(real_vcc + String(" mV"));
  Serial.println();
  vcc_const = vcc_const * real_vcc / ReadVCC();
  Serial.println("New VCC constant is " + String(vcc_const, 5));
  Serial.println("Current voltage is " + String(ReadVCC()) + String(" mV"));
  EEPROM.updateFloat(VCC_CONST_POSITION, vcc_const);
  EEPROM.updateByte(MODE_POSITION, 0);
  EEPROM.updateFloat(VOLT_POSITION, 0);
  EEPROM.updateByte(WATT_POSITION, 0);
  EEPROM.updateByte(AMP_POSITION, 0);
  EEPROM.updateFloat(OHM_POSITION, 0);
  while (true);
}

long ReadVCC() {
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  uint8_t low  = ADCL;
  uint8_t high = ADCH;
  long result = (high << 8) | low;
  result = vcc_const * 1023 * 1000 / result;
  return result;
}

void InitTimer2() {
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = bit(WGM21);
  OCR2A = round(15625.0 / (DISPLAY_FREQUENCY * 8.0));
  TIMSK2 = bit(OCIE2A);
  TCNT2 = 0;
  GTCCR = bit(PSRASY);
  TCCR2B =  bit(CS21) | bit(CS22);
}

ISR (TIMER2_COMPA_vect) {
  disp.timerIsr();
}
