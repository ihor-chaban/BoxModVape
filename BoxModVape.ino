/*
    Box Mod Vape v3.3
      - improved voltage and PWM filtration
      - added possibility to adjust filtration using a coefficient
      - fixed calculation of power limitations during firing
      - added voltage display format toggle
      - replaced calibration mode with vcc_const set up menu
      - some code refactoring
    Author: Ihor Chaban
    Jun 2020
*/

#include <ArduinoSTL.h>
#include <EEPROMex.h>
#include <LowPower.h>
#include <map>
#include <OneButton.h>
#include <TimerOne.h>
#include <TM74HC595Display.h>

// Software settings
#define BATTERY_PERCENTAGE  1
#define PERCENTAGE_TOGGLING 1
#define STANDBY_TIME        300000
#define UNLOCK_TIME         3000
#define FIRE_LIMIT_TIME     5000
#define FIRE_FREQUENCY      20000
#define DISPLAY_FREQUENCY   30
#define VOLTS_STEP          0.05
#define WATTS_STEP          1
#define AMPS_STEP           1
#define OHMS_STEP           0.005
#define RESIST_STEP         0.001
#define VCC_STEP            0.001
#define F_B_DEBOUNCE_TIME   100

// Battery settings
#define BATTERY_MIN         2800
#define BATTERY_MAX         4200

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
#define VALUES_UPD_INTERVAL 10
#define VOLTAGE_FILTER_COEF 0.3
#define PWM_FILTER_COEF     0.1

// Do not change this if you are not 100% sure what you are doing
// Dummy changing of these values will cause system instability
#define VOLTAGE_BUFFER_SIZE 3
#define PWM_BUFFER_SIZE     3
#define VCC_CONST_POSITION  (0)
#define MODE_POSITION       (VCC_CONST_POSITION + sizeof(vcc_const))
#define VOLT_POSITION       (MODE_POSITION + sizeof(byte))
#define WATT_POSITION       (VOLT_POSITION + sizeof(volt))
#define AMP_POSITION        (WATT_POSITION + sizeof(watt))
#define OHM_POSITION        (AMP_POSITION + sizeof(amp))
#define RESIST_POSITION     (OHM_POSITION + sizeof(ohm))

OneButton fire_button(FIRE_BUTTON_PIN, true);
OneButton mode_button(MODE_BUTTON_PIN, true);
OneButton down_button(DOWN_BUTTON_PIN, true);
OneButton up_button(UP_BUTTON_PIN, true);
TM74HC595Display disp(SCLK_PIN, RCLK_PIN, DIO_PIN);

int amp, prev_voltage, voltage, watt;
float vcc_const, volt, ohm, battery_resistance;
byte last_fire_mode, last_setting_mode;
unsigned long standby_time, fire_time, values_update_time, f_b_debounce_time, percentage_toggle_time;
bool settings_mode, f_b_state, f_b_last_state, f_b_reading, sleeping, allow_fire, percentage;
word voltage_buffer[VOLTAGE_BUFFER_SIZE], PWM_buffer[PWM_BUFFER_SIZE], prev_PWM, PWM, voltage_drop;
std::map <char, byte> symbols;
std::map <byte, char*> display_shortcuts;

enum Modes {VARIVOLT, VARIWATT, HELL, AMP, OHM, RESIST, VCC} mode;
enum DisplayShortcuts {LOWB = (VCC + 1), BYE, FIRE1, FIRE2, V___, VA__, VAP_, VAPE};
Modes operator++(Modes& i, int) {
  if (!settings_mode) {
    if (i >= HELL) {
      return i = VARIVOLT;
    }
  } else {
    if (i >= VCC) {
      return i = AMP;
    }
  }
  byte temp = i;
  return i = static_cast<Modes> (++temp);
}

void setup() {
  pinMode(FIRE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOSFET_POWER_PIN, OUTPUT);
  pinMode(DISPLAY_POWER_PIN, OUTPUT);
  DisableAllFire();
  digitalWrite(DISPLAY_POWER_PIN, HIGH);
  mode_button.attachClick(ChangeMode);
  mode_button.attachLongPressStart(InitPercentageTogging);
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
  mode = EEPROM.readByte(MODE_POSITION);
  volt = EEPROM.readFloat(VOLT_POSITION);
  watt = EEPROM.readByte(WATT_POSITION);
  amp = EEPROM.readByte(AMP_POSITION);
  ohm = EEPROM.readFloat(OHM_POSITION);
  battery_resistance = EEPROM.readFloat(RESIST_POSITION);
  last_fire_mode = mode;
  last_setting_mode = AMP;
  InitVoltage();
  if (voltage < BATTERY_MIN) {
    DisableAllFire();
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
      voltage = GetVoltage();
      switch (mode) {
        case VARIVOLT: {
            volt = constrain(volt, 0, voltage / 1000.0);
            voltage_drop = round((volt * battery_resistance * 1000.0) / (ohm + battery_resistance));
            AddPWM(map(volt * 1000, 0, voltage, 0, 1023));
            break;
          }
        case VARIWATT: {
            watt = constrain(watt, 0, round(pow(voltage, 2) / ohm / 1000000.0));
            voltage_drop = round(sqrt(ohm * watt) * battery_resistance * 1000.0 / ohm);
            AddPWM(map(sqrt(ohm * watt) * 1000, 0, voltage, 0, 1023));
            break;
          }
        case HELL: {
            voltage_drop = round((voltage * battery_resistance) / (ohm + battery_resistance));
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
        if ((f_b_state == HIGH) && (ohm > 0)) {
          allow_fire = true;
          fire_time = millis();
        }
      }
    }
    f_b_last_state = f_b_reading;
    if (f_b_state && !f_b_last_state) {
      f_b_state = false;
      DisableAllFire();
    }
    if (allow_fire) {
      if ((mode == VARIVOLT || mode == VARIWATT) && (PWM > 0)) {
        Timer1.pwm(MOSFET_POWER_PIN, PWM);
        ShowFireAnimation();
      } else if (mode == HELL) {
        digitalWrite(MOSFET_POWER_PIN, HIGH);
        ShowFireAnimation();
      }
      standby_time = millis();
    }
    if (allow_fire && millis() - fire_time >= FIRE_LIMIT_TIME) {
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

void InitVoltage() {
  memset(voltage_buffer, 0, sizeof(voltage_buffer));
  memset(PWM_buffer, 0, sizeof(PWM_buffer));
  voltage = 0;
  prev_voltage = 0;
  voltage_drop = 0;
  PWM = 0;
  prev_PWM = 0;
  do {
    for (byte i = 0; i < VOLTAGE_BUFFER_SIZE - 1; i++) {
      voltage_buffer[i] = voltage_buffer[i + 1];
    }
    voltage_buffer[VOLTAGE_BUFFER_SIZE - 1] = ReadVCC();
    voltage += voltage_buffer[VOLTAGE_BUFFER_SIZE - 1];
  } while (voltage_buffer[0] == 0);
  voltage = round(voltage / (float)VOLTAGE_BUFFER_SIZE);
  prev_voltage = voltage;
}

int GetVoltage() {
  for (byte i = 0; i < VOLTAGE_BUFFER_SIZE - 1; i++) {
    voltage_buffer[i] = voltage_buffer[i + 1];
  }
  voltage_buffer[VOLTAGE_BUFFER_SIZE - 1] = ReadVCC();
  prev_voltage = RunningAverage(prev_voltage, Median(voltage_buffer[0], voltage_buffer[1], voltage_buffer[2]), VOLTAGE_FILTER_COEF);
  return prev_voltage;
}

void AddPWM(word value) {
  for (byte i = 0; i < PWM_BUFFER_SIZE - 1; i++) {
    PWM_buffer[i] = PWM_buffer[i + 1];
  }
  PWM_buffer[PWM_BUFFER_SIZE - 1] = value;
}

word GetPWM() {
  prev_PWM = RunningAverage(prev_PWM, Median(PWM_buffer[0], PWM_buffer[1], PWM_buffer[2]), PWM_FILTER_COEF);
  return prev_PWM;
}

int RunningAverage(int old_value, int new_value, float coef) {
  old_value += round((new_value - old_value) * coef);
  return old_value;
}

word Median(word a, word b, word c) {
  word middle;
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  } else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    }
    else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}

void DisableAllFire() {
  allow_fire = false;
  Timer1.disablePwm(MOSFET_POWER_PIN);
  digitalWrite(MOSFET_POWER_PIN, LOW);
}

void GoodNight() {
  DisableAllFire();
  sleeping = true;
  disp.clear();
  delay(5);
  EEPROM.updateFloat(VCC_CONST_POSITION, vcc_const);
  EEPROM.updateByte(MODE_POSITION, last_fire_mode);
  EEPROM.updateFloat(VOLT_POSITION, volt);
  EEPROM.updateByte(WATT_POSITION, watt);
  EEPROM.updateByte(AMP_POSITION, amp);
  EEPROM.updateFloat(OHM_POSITION, ohm);
  EEPROM.updateFloat(RESIST_POSITION, battery_resistance);
  attachInterrupt(digitalPinToInterrupt(FIRE_BUTTON_PIN), WakeUp, FALLING);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void WakeUp() {
  detachInterrupt(FIRE_BUTTON_PIN);
}

void SleepPuzzle() {
  DisableAllFire();
  unsigned long wake_time = millis();
  unsigned long display_upd_time = millis();
  bool sleeping = 0;
  byte click_count = 2;
  while (millis() - wake_time < UNLOCK_TIME && !sleeping) {
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
    if (millis() - display_upd_time >= VALUES_UPD_INTERVAL) {
      display_upd_time = millis();
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
  unsigned long wake_time = millis();
  unsigned long display_upd_time = millis();
  bool allow_wakeup = 0;
  byte click_count = 0;
  while (millis() - wake_time < UNLOCK_TIME && !allow_wakeup) {
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
    if (millis() - display_upd_time >= VALUES_UPD_INTERVAL) {
      display_upd_time = millis();
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
    InitVoltage();
    standby_time = millis();
  } else {
    GoodNight();
  }
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

void InitPercentageTogging() {
  percentage = BATTERY_PERCENTAGE;
  percentage_toggle_time = millis();
}

void ReduceValue() {
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
    case RESIST: {
        if (amp > 0) {
          battery_resistance -= RESIST_STEP;
          battery_resistance = round(battery_resistance / RESIST_STEP) * RESIST_STEP;
          battery_resistance = constrain(battery_resistance, 0, 0.1);
        } else {
          battery_resistance = 0;
        }
        break;
      }
    case VCC: {
        vcc_const -= VCC_STEP;
        vcc_const = round(vcc_const / VCC_STEP) * VCC_STEP;
        vcc_const = constrain(vcc_const, 1, 1.2);
        break;
      }
  }
  standby_time = millis();
}

void ReduceValueL() {
  static unsigned long reduce_longpress_time;
  if (millis() - reduce_longpress_time >= 100) {
    reduce_longpress_time = millis();
    ReduceValue();
  }
}

void IncreaseValue() {
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
    case RESIST: {
        if (amp > 0) {
          battery_resistance += RESIST_STEP;
          battery_resistance = round(battery_resistance / RESIST_STEP) * RESIST_STEP;
          battery_resistance = constrain(battery_resistance, 0, 0.1);
        } else {
          battery_resistance = 0;
        }
        break;
      }
    case VCC: {
        vcc_const += VCC_STEP;
        vcc_const = round(vcc_const / VCC_STEP) * VCC_STEP;
        vcc_const = constrain(vcc_const, 1, 1.2);
        break;
      }
  }
  standby_time = millis();
}

void IncreaseValueL() {
  static unsigned long increase_longpress_time;
  if (millis() - increase_longpress_time >= 100) {
    increase_longpress_time = millis();
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

void CheckButtons() {
  fire_button.tick();
  mode_button.tick();
  up_button.tick();
  down_button.tick();
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
        break;
      }
    case RESIST: {
        disp.set(symbols[display_shortcuts[RESIST][0]] & 0x7F, 3);
        if (battery_resistance < 0.01) {
          disp.set(symbols['O'], 2);
          disp.set(symbols['O'], 1);
          disp.digit2(battery_resistance * 1000, 0);
        } else if (battery_resistance < 0.1) {
          disp.set(symbols['O'], 2);
          disp.digit2(battery_resistance * 1000, 0);
        } else {
          disp.digit4(battery_resistance * 1000, 0);
        }
        break;
      }
    case VCC: {
        disp.set(symbols[display_shortcuts[VCC][0]] & 0x7F, 3);
        if ((vcc_const - 1) < 0.01) {
          disp.set(symbols['O'], 2);
          disp.set(symbols['O'], 1);
          disp.digit2((vcc_const - 1) * 1000, 0);
        } else if ((vcc_const - 1) < 0.1) {
          disp.set(symbols['O'], 2);
          disp.digit2((vcc_const - 1) * 1000, 0);
        } else {
          disp.digit4((vcc_const - 1) * 1000, 0);
        }
        break;
      }
  }
}

void ShowVoltage() {
  static unsigned long voltage_upd_time;
  if (PERCENTAGE_TOGGLING && millis() - percentage_toggle_time >= 1000) {
    percentage_toggle_time = millis();
    percentage = !percentage;
  }
  if (millis() - voltage_upd_time >= VALUES_UPD_INTERVAL) {
    voltage_upd_time = millis();
    disp.clear();
    disp.set(symbols['b'], 3);
    if (percentage) {
      disp.digit4(map(constrain(voltage, BATTERY_MIN + voltage_drop, BATTERY_MAX), BATTERY_MIN + voltage_drop, BATTERY_MAX, 0, 100));
    } else {
      disp.float_dot(voltage / 1000.0, 2);
    }
  }
  standby_time = millis();
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
    case RESIST: {
        DisplaySlide(display_shortcuts[RESIST], true);
        break;
      }
    case VCC: {
        DisplaySlide(display_shortcuts[VCC], true);
        break;
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
  symbols['r'] = 0xAF;
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
  char c_resi[] = {'r', 'E', 'S', 'I'};
  char c_vcc_[] = {'V', 'C', 'C', ' '};
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
  display_shortcuts.insert(std::pair <byte, char*> (RESIST, c_resi));
  display_shortcuts.insert(std::pair <byte, char*> (VCC, c_vcc_));
  display_shortcuts.insert(std::pair <byte, char*> (LOWB, c_lowb));
  display_shortcuts.insert(std::pair <byte, char*> (FIRE1, c_fir1));
  display_shortcuts.insert(std::pair <byte, char*> (FIRE2, c_fir2));
  display_shortcuts.insert(std::pair <byte, char*> (V___, c_v___));
  display_shortcuts.insert(std::pair <byte, char*> (VA__, c_va__));
  display_shortcuts.insert(std::pair <byte, char*> (VAP_, c_vap_));
  display_shortcuts.insert(std::pair <byte, char*> (VAPE, c_vape));
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

ISR (TIMER2_COMPA_vect) {
  disp.timerIsr();
}
