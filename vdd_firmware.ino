#include <HeliOS_Arduino.h>
#include <avr/pgmspace.h>
#include <SPI.h>

/* PIN DEFINITIONS */
const uint8_t pin_footswitch  = 2;
const uint8_t pin_rgb_b       = 3;
const uint8_t pin_rgb_r       = 5;
const uint8_t pin_rgb_g       = 6;
const uint8_t pin_delay_mod   = 9;
const uint8_t pin_delay_pwm   = 10;
//const uint8_t pin_bypass      = ;
const uint8_t pin_tails       = 8;
const uint8_t pin_delay_time  = A0;
const uint8_t pin_mod_speed   = A1;

/* MODE PARAM */
bool bypass = true;
bool tempo_blink = false;

const uint8_t SMODE_BYPASS  = 0;
const bool BYPASS_NORM    = 0;
const bool BYPASS_TAIL    = 1;

// LFO Freq = 1 / (WAVETABLE_LENGTH / (MOD_TASK_FREQ / MOD_SPEED))
const uint8_t SMODE_MOD     = 1;
volatile double mod_speed     = 0.5;

const uint8_t MODE_CONFIG = 0;
const uint8_t MODE_PLAY   = 1;
volatile uint8_t mode = MODE_PLAY;
volatile uint8_t smode = SMODE_MOD;

/* DELAY PARAMETERS */
volatile int delay_time = 250;
volatile int delay_time_prev = 0;
unsigned long t_ms = 0;
unsigned long prev_t = 0;

unsigned long first_tap_time;
unsigned long first_release_time;
bool wait_for_release = false;

/* DEBOUNCE PARAMETERS */
const uint8_t debounce_delay = 20;
unsigned long last_debounce_time = 0;
bool fsstate;
bool last_fsstate = HIGH;
bool last_button = HIGH;

/* TASKS */
void taskButton(xTaskId id_)
{
  int button = digitalRead(pin_footswitch);
  // reset debouncing timer if the switch changed, due to noise or pressing:
  if (button != last_button)
    last_debounce_time = millis();

  // If the button state is stable for at least [debounce_delay], fire body of the statement
  if ((millis() - last_debounce_time) > debounce_delay) {
    
    // Button and last_button represent the 'unstable' input that gets updated continuously.
    // These are used for debouncing.
    // fsstate is the stable input that can be used for reading button presses.
    if (button != fsstate) 
      fsstate = button;

    if (fsstate == LOW && last_fsstate == HIGH) // FALLING EDGE
    {
      first_tap_time = millis();
      wait_for_release = true;
      last_fsstate = fsstate;
    }
    else if (wait_for_release && ((millis() - first_tap_time) >= 1000))
    {
      wait_for_release = false;
      xTaskNotify(xTaskGetId("TASKMODEMAN"), 4, (char *)"SMOD" );
    }
    else if (fsstate == HIGH && last_fsstate == LOW) // RISING EDGE
    {
      if (wait_for_release)
      {
        first_release_time = millis();
        xTaskNotify(xTaskGetId("TASKMODEMAN"), 4, (char *)"STEP" );
        wait_for_release = false;
      }
      last_fsstate = fsstate;
    }
  }
  last_button = button;
}

void update_led_status() {
  analogWrite(pin_rgb_r, 255 * (smode == SMODE_BYPASS));
  analogWrite(pin_rgb_b, 255 * (smode == SMODE_MOD));

  if (smode == SMODE_BYPASS)
    analogWrite(pin_rgb_g, 127 * bypass);
}

void taskModeMan(xTaskId id_) {
  xTaskGetNotifResult res = xTaskGetNotif(id_);
  if (res) {

    if (strcmp(res->notifyValue, "SMOD") == 0)
    {
      smode++;
      if (smode > SMODE_MOD)
        smode = SMODE_BYPASS;
    }
    else if (strcmp(res->notifyValue, "STEP") == 0)
    {
      if (smode == SMODE_BYPASS)
      {
        bypass = !bypass;
        if (BYPASS_TAIL)
          digitalWrite(pin_tails, bypass);
        else
          digitalWrite(pin_tails, !bypass);
      }
      else // smode == SMODE_MOD
      {
        /*
          waveform++;
          if (waveform > SQUARE)
            waveform = TRIANGLE;
        */
        // TODO: INTRODUCE WAVEFORM COLORING?
        // TRIANGLE = BLUE
        // SINE = CYAN
        // SQUARE = PURPLE
      }
    }
    update_led_status();
  }
  xMemFree(res);
  xTaskNotifyClear(id_);
}

void taskDelay(xTaskId id_) {
  delay_time = analogRead(pin_delay_time);

  if (abs(delay_time - delay_time_prev) < 3)
    return;

  uint8_t res = map(delay_time, 0, 1023, 0, 255);
  long delay_time_us = long(map(delay_time, 0, 1023, 53, 626)) * 1000;
  xTaskSetTimer(xTaskGetId("TASKTEMPO"), delay_time_us);

  //SPIWrite(MCP_WRITE, res, ssMCPin);
  analogWrite(pin_delay_pwm, res);

  prev_t = t_ms;
  delay_time_prev = delay_time;
}

void taskSerial(xTaskId id_) {
  xTaskGetNotifResult res = xTaskGetNotif(id_);
  if (res)
    Serial.println(res->notifyValue);
  xMemFree(res);
  xTaskNotifyClear(id_);
}

/* MAIN FUNCTIONS */
void setup() {
  Serial.begin(115200);
  Serial.println(F("[VDD Debug Stream]"));
  TCCR1B = (TCCR1B & 0b11111000) | 0x01; // Set PWM Frequency to 31kHz

  pinMode(pin_footswitch, INPUT_PULLUP);
  pinMode(pin_delay_time, INPUT);
  pinMode(pin_mod_speed, INPUT);
  pinMode(pin_delay_pwm, OUTPUT);

  // Initialize RGB LED
  pinMode(pin_rgb_r, OUTPUT);
  pinMode(pin_rgb_g, OUTPUT);
  pinMode(pin_rgb_b, OUTPUT);
  pinMode(pin_tails, OUTPUT);

  update_led_status();

  //pinMode (ssMCPin, OUTPUT);
  //digitalWrite(ssMCPin, HIGH);
  digitalWrite(pin_tails, HIGH);

  xTaskId id = 0;
  xHeliOSSetup();

  id = xTaskAdd("TASKBUTTON", &taskButton);
  xTaskStart(id);

  id = xTaskAdd("TASKMODEMAN", &taskModeMan);
  xTaskWait(id);

  id = xTaskAdd("TASKMOD", &taskMod);
  xTaskWait(id);
  xTaskSetTimer(id, 4000); // 4ms

  id = xTaskAdd("TASKSERIAL", &taskSerial);
  xTaskWait(id);
}

void loop() {
  xHeliOSLoop();
}
