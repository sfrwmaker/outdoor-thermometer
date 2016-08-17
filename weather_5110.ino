/* External weather sensor. It uses two sensors: SI7021 ans BMP280.
 * AVR - atmeta328p-pu@1MHz + nokia 5110 display
 * The voltage regulator ams1117-adj, supplies constant 1.24 volts to BATT_PIN pin when is powered up through POWER_PIN
 * The higher the value read on BATT_PIN the lower the battery.
 * arduine reads 380 at 3.3 volts on battery, 569 at 2.22 volts
*/
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <SI7021.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "U8glib.h"

// Nokia 5110 display connection
const byte CE       = 10;
const byte RST      = 8;
const byte DC       = 9;
const byte DIN_MOSI = 11;
const byte CLK      = 13;

const byte POWER_PIN = 7;                       // Through this pin the voltage regulator is supplied
const byte BATT_PIN = 15;                       // A1, Batery voltage sensor

#define DIGIT_FONT    u8g_font_helvR14n
#define TEMP_FONT     u8g_font_gdb20n

const uint8_t bmCentigrade[] PROGMEM = {
  0b00111000,
  0b01000100,
  0b01000100,
  0b01000100,
  0b00111000,
  0b00000000,
  0b00000000,
  0b00000000
};

const uint8_t bmPercent[] PROGMEM = {
  0b01100001,
  0b10010010,
  0b10010100,
  0b01101000,
  0b00010110,
  0b00101001,
  0b01001001,
  0b10000110
};

const uint8_t battery_bitmap[] PROGMEM = {
  0b01111111, 0b11111000,
  0b10000000, 0b00000100,
  0b10001000, 0b01000111,
  0b10001000, 0b01000101,
  0b10001000, 0b01000111,
  0b10000000, 0b00000100,
  0b01111111, 0b11111000,
};

const uint8_t bmInc[] PROGMEM = {
  0b00000001,
  0b00000011,
  0b00000111,
  0b00001111,
  0b00011111,
  0b00111111,
  0b01111111,
  0b11111111
};

const uint8_t bmDec[] PROGMEM = {
  0b10000000,
  0b11000000,
  0b11100000,
  0b11110000,
  0b11111000,
  0b11111100,
  0b11111110,
  0b11111111
};

const uint8_t bmConst[] PROGMEM = {
  0b00000000,
  0b00110011,
  0b11111111,
  0b11001100,
  0b00110011,
  0b11111111,
  0b11001100,
  0b00000000
};

//------------------------------------------ class HISTORY ----------------------------------------------------
#define H_LENGTH 32
class HISTORY {
  public:
    HISTORY(void) { len = 0; }
    void init(void) { len = 0; }
    void put(int item) {
      if (len < H_LENGTH) {
        queue[len++] = item;
      } else {
        for (byte i = 0; i < len-1; ++i) queue[i] = queue[i+1];
        queue[H_LENGTH-1] = item;
      }
    }
    bool isFull(void) { return len == H_LENGTH; }
    int top(void) { return queue[0]; }
    int average(void);
    float dispersion(void);
    float gradient(void);
  private:
    int queue[H_LENGTH];
    byte len;
};

int HISTORY::average(void) {
  long sum = 0;
  if (len == 0) return 0;
  if (len == 1) return queue[0];
  for (byte i = 0; i < len; ++i) sum += queue[i];
  sum += len >> 1;                      // round the average
  sum /= len;
  return (int)sum;
}

float HISTORY::dispersion(void) {
  if (len < 3) return 1000;
  long sum = 0;
  long avg = average();
  for (byte i = 0; i < len; ++i) {
    long q = queue[i];
    q -= avg;
    q *= q;
    sum += q;
  }
  sum += len << 1;
  float d = (float)sum / (float)len;
  return d;
}

// approfimating the history with the line (y = ax+b) using method of minimum square. Gradient is parameter a
float HISTORY::gradient(void) {
  if (len < 2) return 0;
  long sx, sx_sq, sxy, sy;
  sx = sx_sq = sxy = sy = 0;
  for (byte i = 1; i <= len; ++i) {
    sx    += i;
  sx_sq += i*i;
  sxy   += i*queue[i-1];
  sy    += queue[i-1];
  }
  long numerator   = len * sxy - sx * sy;
  long denominator = len * sx_sq - sx * sx;
  float a = (float)numerator / (float)denominator;
  return a;
}

//------------------------------------------ class battery ---------------------------------------------------
class BATTERY {
  public:
    BATTERY(byte powerPIN, byte batteryPIN) {
      pwrPIN = powerPIN;
      batPIN = batteryPIN;
      nxt_check = 0;
      isOK = true;
      level = high_battery;
    }
  void init(void);
  byte percent(void);                         // Return the battery level in the percent
  private:
    byte pwrPIN;                              // The pin to power voltage regulator
    byte batPIN;                              // Analog pin to read the voltage
    uint32_t nxt_check;                       // The time in ms to check the batery level
    bool isOK;                                // Weither the battery is OK
    uint16_t level;                           // The battery level
    const uint32_t period = 3600;             // Check the battery every hour
    const uint16_t low_battery  = 600;        // The limit for low battery
    const uint16_t high_battery = 380;        // The limit for high battery
};

void BATTERY::init(void) {
  pinMode(pwrPIN, OUTPUT);
  pinMode(batPIN, INPUT);
  digitalWrite(pwrPIN, LOW);
}

byte BATTERY::percent(void) {
  if (isOK && (millis() >= nxt_check)) {      // Do not check BAD battery once more; It is time to check the battery level
    digitalWrite(pwrPIN, HIGH);               // power up the voltage regulator that supply 1.24 volts to batPIN
    delay(10);
    level = analogRead(batPIN);               // 0 <= level <= 1023
    digitalWrite(pwrPIN, LOW);                // power down the voltage regulator, save the power
    isOK = (level < low_battery);
    if (level > low_battery) {
      level = low_battery;
    } else {
      if (level < high_battery)
        level = high_battery;
    }
    nxt_check = millis() + period;
  }
  int p = low_battery - level;
  byte percent = map(p, 0, (low_battery - high_battery), 0, 100);
  return percent;
}

//------------------------------------------ class MODE ------------------------------------------------------
class MODE {
  public:
    MODE() {
      next = 0;
    }
    virtual void setNext(MODE* Next) { next = Next; }
    virtual void init(void) { }
    virtual void loop(void) { }
    virtual MODE* menu(void) {if (this->next != 0) return this->next; else return this; }
    protected:
      MODE* next;
};


class MAIN_MODE : public MODE {
  public:
    MAIN_MODE(U8GLIB_PCD8544* U8g, SI7021* Sensor, Adafruit_BMP280* Bme, BATTERY* Batt) : MODE() {
      pS    = Sensor;
      pBme  = Bme;
      pU8g  = U8g;
      pBatt = Batt;
      old_temp = -1000;
      old_hum = 0;
      old_press = 0;
    }
    virtual void loop(void);
  private:
    SI7021* pS;                                 // pointer to the SI7021 temperature & humidity sensor
    Adafruit_BMP280* pBme;                      // pointer ot the BMP280 sensor
    U8GLIB_PCD8544* pU8g;                       // pointer to the OLED screen instance
    BATTERY* pBatt;                             // pointer to the battery instance
    HISTORY hPress;                             // pressure history
    int      old_temp;                          // the temperature, measured last time
    byte     old_hum;                           // the humidity, measured last time
    uint16_t old_press;                         // the pressure, measured last time
    const byte s_width  = 84;                   // The Screen Width (there is some screen defect)
    const byte s_height = 48;                   // The Screen Height
};

void MAIN_MODE::loop(void) {
  char out_temp[6], out_humm[3], in_press[5];

  bool refresh = false;                         // by default, do not refresh the screen
  int temperature = pS->getCelsiusHundredths();
  if (temperature > 0)
    temperature += 5;
  else
    temperature -= 5;
  temperature /=10;

  if (temperature != old_temp) {
    old_temp = temperature;
    refresh = true;
  }

  byte humidity   = pS->getHumidityPercent();
  float p = pBme->readTemperature();
  p = pBme->readPressure() * 0.00750064;
  uint16_t pressure = (uint16_t)p;
  hPress.put(pressure);
  int p_diff = pressure - hPress.average();

  if (humidity != old_hum) {
    old_hum = humidity;
    refresh = true;
  }

  if (pressure != old_press) {
    old_press = pressure;
    refresh = true;
  }

  if (!refresh) return;                         // Do not refresh the screen

  if (temperature > 0) {
    sprintf(out_temp, "%2d.%1d", temperature / 10, temperature % 10);
  } else {
    temperature *= -1;
    out_temp[0] = '-';
    sprintf(&out_temp[1], "%2d.%1d", temperature / 10, temperature % 10);
  }
  sprintf(out_humm, "%2d", humidity);
  sprintf(in_press, "%3d", pressure);

  byte batt = map(pBatt->percent(), 0, 100, 1, 12);

  pU8g->firstPage();
  do {
    pU8g->drawBitmapP(0, 0, 2, 7, battery_bitmap);
    pU8g->drawBox(1, 1, batt, 5);

    pU8g->setFont(TEMP_FONT);
    byte width = pU8g->getStrWidth(out_temp);
    pU8g->drawStr(s_width-8-width, 24, out_temp);
    pU8g->drawBitmapP(s_width-8, 4, 1,  8, bmCentigrade);

    pU8g->drawHLine(5, s_height-18, s_width-10);
    pU8g->setFont(DIGIT_FONT);
    pU8g->drawStr(0, s_height, in_press);
    width = pU8g->getStrWidth(out_humm);
    pU8g->drawStr(s_width-10-width, s_height, out_humm);
    pU8g->drawBitmapP(s_width-8, s_height-8, 1,  8, bmPercent);
    if (p_diff > 0) {
        pU8g->drawBitmapP(34, s_height-8, 1,  8, bmInc);
    } else {
      if (p_diff < 0)
        pU8g->drawBitmapP(34, s_height-8, 1,  8, bmDec);
      else
        pU8g->drawBitmapP(34, s_height-8, 1,  8, bmConst);
    }

  } while(pU8g->nextPage());
}
// ===================================== End of the classes definition ==============================================

U8GLIB_PCD8544 u8g(CLK, DIN_MOSI, CE, DC, RST);
SI7021 sensor;
Adafruit_BMP280 bme;
BATTERY batt(POWER_PIN, BATT_PIN);

MAIN_MODE MainMode(&u8g, &sensor, &bme, &batt);

MODE* pCurrentMode = &MainMode;

volatile int f_wdt=1;

void setup() {
  sensor.begin();
  bme.begin(0x76);
  batt.init();
  u8g.setColorIndex(1);                         // pixel on
  pCurrentMode->loop();

  delay(15000);                                 // Sleep to flash new programm

  // Setup the WDT
  MCUSR &= ~(1<<WDRF);                          // Clear the reset flag
  
  // In order to change WDE or the prescaler, we need to
  // set WDCE (This will allow updates for 4 clock cycles).
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 1<<WDP0 | 1<<WDP3;                   // set new watchdog timeout prescaler value 8.0 seconds
  WDTCSR |= _BV(WDIE);                          // Enable the WD interrupt (note no reset).
}

void enterSleep(void) {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);          // EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption.
  sleep_enable();
  
  sleep_mode();                                 // Now enter sleep mode.
  
  // The program will continue from here after the WDT timeout. 
  sleep_disable();                              // First thing to do is disable sleep.
  power_all_enable();                           // Re-enable the peripherals.
}

// ======================== The Main Loop ====================================
void loop() {
  static int awake_counter = 1;

  if(f_wdt == 1) {
    f_wdt = 0;
    --awake_counter;
    if (awake_counter <= 0) {                   // Show Weather data
      awake_counter = 10;
      pCurrentMode->loop();           
    }
    enterSleep();
  }
}

ISR(WDT_vect) {
  if(f_wdt == 0) f_wdt=1;
}

