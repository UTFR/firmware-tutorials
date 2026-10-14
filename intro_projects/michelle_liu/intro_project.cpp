#include <arduino_freertos.h>
#include <stdint.h>
#include <semphr.h>
#include <string.h>
using namespace arduino;

/*
The car has three possible functional states:
1. Low Voltage (LV)
2. Tractive System (TS)
3. Ready to Drive (RTD)

When in LV, the car cannot drive. When in TS, the car is energized, but still
cannot drive. When in RTD, the car is energized and able to drive.

There are two Accumulator Isolation Relays (AIRs) that connect the high voltage
(HV) battery to the rest of the car. One is on the positive terminal, and the
other on the negative terminal (AIR+ and AIR-). That is to say, the battery is
disconnected unless we energize both relays, which is important for safety.
Whenever we have a critical error, we can open these relays / de-energize them
to make sure there is no HV outside the accumulator.

When we go from LV -> TS, we cannot simply energize the relays (ask an
electrical lead why!). Instead we first close AIR-, then another relay called
the "Precharge Relay". After ~5 seconds we close AIR+ and open the precharge
relay.

There is a TS On button and an RTD button on the dash that move the cars into
those states. ie. When you press TS On the car goes TS, and when you press RTD,
it goes to RTD. They are both active low, meaning when the button's are pressed,
the voltage goes low.

We also have a few sensors that we need to be able to calculate how much torque
to command:

1. Current sensor (How much current there is in the HV path)
2. Wheelspeeds
3. Steering Angle Sensor

The current sensor is a Hall Effect sensor read by an ADC through GPIO pins. The
conversion is: 1 amp per 10mV

Wheelspeeds are a bit tricky. The sensor is routed to a GPIO pin, and is
normally high. However, the wheels have a gear looking thing with 17 teeth --
everytime one of the teeth passes the sensor, the voltage at the GPIO pin goes
low. You can derive the RPM of the wheel from how often the voltage goes low.

The steering angle sensor comes to us via CAN.

We also have to monitor our battery to make sure nothing explodes or catches on
fire... Thankfully we have a Battery Management System (BMS) that monitors cell
voltages and temperatures and can send them to us. It sends this info via SPI.
If any cell voltage goes below 2.8V or above 4.3V, de-energize the car. Also, if
any temperature exceeds 60 degrees, de-energize.

Finally, we have an LCD on the dash which is useful to display information. You
should print all relevant info to the screen so we know what's going on with the
car. This also operates via SPI.

Mock library functions have been provided where necessary (marked with extern).

NOTE - EXTREMELY IMPORTANT: the CAN methods `can_send` and `can_receive` are NOT
thread safe.

Also think about the implications of having two peripherals on one SPI bus.

----------------------------

Requirements:

1. Command each motor with an appropriate torque every 1ms
2. Print sensor values and torque values to the LCD every 100ms
3. Monitor the highest/lowest voltage and highest temperature from the BMS and
shutdown the car if there is any unsafe condition

----------------------------

Pins:

TS On:          12
RTD:            13
Current Sensor: 19
CAN RX:         2
CAN TX:         3
MOSI:           5
MISO:           6
SCK:            7
LCD_CS:         8
BMS_CS:         9
AIR+:           22
Precharge:      23
AIR-:           10

----------------------------

"Submission" instructions. I want you all to get familiar with git, so we will
do this project with git. The repo is
https://github.com/UTFR/firmware-tutorials.

If you still haven't joined the github organization, let me know and i'll add
you. Clone the repository, and make a branch called `<your name>/intro_project`.
Copy this file into the directory `intro_projects/<your name>` and make all your
changes there.

Whenever you add a feature, `git add` the files, and `git commit -m "..."` with
a useful/descriptive message. Whenver you want your changes to be public, do
`git push origin <your name>/intro_project`.

Also, I won't enforce it for this project, but for the actual firmware repo we
have a code formatter (clang-format). It means that everyone's code will look
exactly the same, in terms of how much whitespace there is and other aesthetics
like that. It's not there for aesthetics, but moreso so that you when people
make changes you can see exactly what they changed, whereas without a formatter,
you end up with lots of useless formatting changes. You should install
clang-format as soon as possible and set it up :)
*/

/*
  Initialize the CAN peripheral with given RX and TX pins at a given baudrate.
*/
extern void can_init(uint8_t rx, uint8_t tx, uint32_t baudrate);
/*
  Send a CAN message with a given id.
  The 8 byte payload is encoded as a uint64_t
*/
extern void can_send(uint8_t id, uint64_t payload);
/*
  Receive a CAN message with a given id into a uint64_t
*/
extern void can_receive(uint64_t *payload, uint8_t id);

/*
  Calculates four torques, in order, for the Front Left, Front Right, Rear Left,
  and Rear Right motors given current, wheelspeeds (in the same order), and a
  steering angle.
*/
extern void calculate_torque_cmd(
  float *torques, float current, float *wheelspeeds, float steering_angle
);

/*
  Initialize the LCD peripheral
*/
extern void lcd_init(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t lcs_cs);

/*
  Print something to the LCD
*/
extern void lcd_printf(const char *fmt, ...);

/*
  Initialize the BMS
*/
extern void bms_init(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t lcs_cs);

/*
  Get voltage of the nth cell in the battery
*/
extern float bms_get_voltage(uint8_t n);

/*
  Get temperature of the nth cell in the battery
*/
extern float bms_get_temperature(uint8_t n);

/*
=======================
Project completed below
=======================
*/

// mutex
static SemaphoreHandle_t can_mutex;
static SemaphoreHandle_t spi_mutex;
static SemaphoreHandle_t val_mutex;

// pin assignments
const uint8_t TSON_PIN = 12;
const uint8_t RTD_PIN = 13;
const uint8_t CUR_PIN = 19;
const uint8_t CANRX_PIN = 2;
const uint8_t CANTX_PIN = 3;
const uint8_t MOSI_PIN = 5;
const uint8_t MISO_PIN = 6;
const uint8_t SCK_PIN = 7;
const uint8_t LCDCS_PIN = 8;
const uint8_t BMSCS_PIN = 9;
const uint8_t AIRP_PIN = 22;
const uint8_t PRE_PIN = 23;
const uint8_t AIRN_PIN = 10;

// wheelspeed pins (used any empty pins)
const uint8_t WSFL_PIN = 14;
const uint8_t WSFR_PIN = 15;
const uint8_t WSRL_PIN = 16;
const uint8_t WSRR_PIN = 17;

// car state
const uint8_t LV = 0;
const uint8_t TS = 1;
const uint8_t RTD = 2;
const uint8_t PRECHARGING = 3;
volatile uint8_t car_state = LV;

// set arbitrary vals
float wheelsrpm[4];
volatile uint32_t wsedges[4];

float current_amps;

float steerangle;
// assume steer ID
const uint8_t STEER_ID = 0x20;

float torques[4];
// assume motor IDs
const uint8_t MOTOR_IDS[4] = {0x10, 0x11, 0x12, 0x13}; // FL, FR, RL, RR

// assume number of battery cells
const uint8_t NUMCELLS = 100;
const float BVMIN = 2.8f;
const float BVMAX = 4.3f;
const float BTMAX = 60;
float bms_vmin, bms_vmax, bms_tmax;

// assuming HIGH = relay closed
void relay_close(uint8_t pin) { digitalWrite(pin, HIGH); }
void relay_open(uint8_t pin) { digitalWrite(pin, LOW); }

// init pins
void init_pins() {
  relay_open(AIRP_PIN);
  relay_open(PRE_PIN);
  relay_open(AIRN_PIN);
  pinMode(AIRP_PIN, OUTPUT);
  pinMode(PRE_PIN, OUTPUT);
  pinMode(AIRN_PIN, OUTPUT);
  // active LOW
  pinMode(TSON_PIN, INPUT_PULLUP);
  pinMode(RTD_PIN, INPUT_PULLUP);
  pinMode(WSFL_PIN, INPUT_PULLUP);
  pinMode(WSFR_PIN, INPUT_PULLUP);
  pinMode(WSRL_PIN, INPUT_PULLUP);
  pinMode(WSRR_PIN, INPUT_PULLUP);
}

// init isrs
void wsfl_isr(void) { wsedges[0]++; }
void wsfr_isr(void) { wsedges[1]++; }
void wsrl_isr(void) { wsedges[2]++; }
void wsrr_isr(void) { wsedges[3]++; }

void off() {
  relay_open(AIRP_PIN);
  relay_open(PRE_PIN);
  relay_open(AIRN_PIN);
  car_state = LV;
}

void precharge() {
  car_state = PRECHARGING;
  relay_close(AIRN_PIN);
  relay_close(PRE_PIN);
  vTaskDelay(pdMS_TO_TICKS(5000));
  if(car_state != PRECHARGING) {
    return;
  }
  relay_close(AIRP_PIN);
  relay_open(PRE_PIN);
  car_state = TS;
}

void state_task(void *) {
  TickType_t last = xTaskGetTickCount();
  for(;;) {
    bool ts_pressed = digitalRead(TSON_PIN) == LOW; // active low
    bool rtd_pressed = digitalRead(RTD_PIN) == LOW;

    if(car_state == LV && ts_pressed) {
      precharge();
    } else if(car_state == TS && rtd_pressed) {
      car_state = RTD;
    }

    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
  }
}

float get_current(void) {
  int rawcur = analogRead(CUR_PIN);
  return (rawcur * 3300.0f / (1024 * 10)); // mV to A
}

void get_wheelsrpm(float *wsarr) {
  uint32_t edges[4];

  static uint32_t lastmicros = 0;
  static float lastrpm[4];
  uint32_t now = micros();
  uint32_t elapsed = now - lastmicros;

  if(elapsed >= 20000) { // 20 ms window
    noInterrupts();      // pause to avoid isr changes

    for(int i = 0; i < 4; i++) {
      edges[i] = wsedges[i];
      wsedges[i] = 0;
    }
    interrupts();
    float seconds = elapsed / 1e6f;
    for(int i = 0; i < 4; i++) {
      lastrpm[i] = (edges[i] / 17.0f) / seconds * 60.0f;
    }
    lastmicros = now;
  }

  for(int i = 0; i < 4; ++i) {
    wsarr[i] = lastrpm[i];
  }
}

float get_steering(void) {
  float angle;
  uint64_t payload;
  xSemaphoreTake(can_mutex, portMAX_DELAY);
  can_receive(&payload, STEER_ID);
  xSemaphoreGive(can_mutex);
  uint32_t copypayload = uint32_t(payload);
  memcpy(&angle, &copypayload, 4);
  return angle;
}

void torque_task(void *) {
  TickType_t last = xTaskGetTickCount();
  for(;;) {
    float cur = get_current();
    float ws[4];
    get_wheelsrpm(ws);
    float steer = get_steering();

    float tq[4];
    calculate_torque_cmd(tq, cur, ws, steer);

    if(car_state != RTD) {
      for(int i = 0; i < 4; ++i) {
        tq[i] = 0;
      }
    }
    xSemaphoreTake(can_mutex, portMAX_DELAY);
    for(int i = 0; i < 4; ++i) {
      uint64_t payload = 0;
      memcpy(&payload, &tq[i], 4);
      can_send(MOTOR_IDS[i], payload);
    }
    xSemaphoreGive(can_mutex);

    xSemaphoreTake(val_mutex, portMAX_DELAY);
    current_amps = cur;
    memcpy(wheelsrpm, ws, sizeof(ws));
    steerangle = steer;
    memcpy(torques, tq, sizeof(tq));
    xSemaphoreGive(val_mutex);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
  }
}

void bms_task(void *) {
  TickType_t last = xTaskGetTickCount();
  for(;;) {
    float vmin = 100000.0f;
    float vmax = -100000.0f;
    float tmax = -100000.0f;
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    for(int i = 0; i < NUMCELLS; ++i) {
      float bv = bms_get_voltage(i);
      float bt = bms_get_temperature(i);

      if(bv < vmin) {
        vmin = bv;
      }
      if(bv > vmax) {
        vmax = bv;
      }
      if(bt > tmax) {
        tmax = bt;
      }
    }
    xSemaphoreGive(spi_mutex);
    if(vmin < BVMIN || vmax > BVMAX || tmax > BTMAX) {
      off();
    }

    xSemaphoreTake(val_mutex, portMAX_DELAY);
    bms_vmin = vmin;
    bms_vmax = vmax;
    bms_tmax = tmax;
    xSemaphoreGive(val_mutex);

    vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
  }
}

void lcd_task(void *) {
  const char *state_names[] = {"LV", "TS", "RTD", "PRECHARGING"};
  TickType_t last = xTaskGetTickCount();
  for(;;) {
    float cur, ws[4], steer, tq[4], vmin, vmax, tmax;
    uint8_t state = car_state;
    xSemaphoreTake(val_mutex, portMAX_DELAY);
    cur = current_amps;
    memcpy(ws, wheelsrpm, sizeof(ws));
    steer = steerangle;
    memcpy(tq, torques, sizeof(tq));
    vmin = bms_vmin;
    vmax = bms_vmax;
    tmax = bms_tmax;
    xSemaphoreGive(val_mutex);

    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    lcd_printf("State: %s\n", state_names[state]);
    lcd_printf("Current: %.1f A\n", cur);
    lcd_printf(
      "Wheel Speeds: %.0f %.0f %.0f %.0f\n", ws[0], ws[1], ws[2], ws[3]
    );
    lcd_printf("Steer Angle: %.1f\n", steer);
    lcd_printf("Torque: %.1f %.1f %.1f %.1f\n", tq[0], tq[1], tq[2], tq[3]);
    lcd_printf("Cell Voltage: %.2f - %.2f\n", vmin, vmax);
    lcd_printf("Max Temp: %.1f C\n", tmax);
    xSemaphoreGive(spi_mutex);

    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
  }
}
void setup(void) {
  init_pins();

  can_mutex = xSemaphoreCreateMutex();
  spi_mutex = xSemaphoreCreateMutex();
  val_mutex = xSemaphoreCreateMutex();

  // assumed baudrate = 500000
  can_init(CANRX_PIN, CANTX_PIN, 500000);
  lcd_init(MOSI_PIN, MISO_PIN, SCK_PIN, LCDCS_PIN);
  bms_init(MOSI_PIN, MISO_PIN, SCK_PIN, BMSCS_PIN);

  attachInterrupt(digitalPinToInterrupt(WSFL_PIN), wsfl_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(WSFR_PIN), wsfr_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(WSRL_PIN), wsrl_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(WSRR_PIN), wsrr_isr, FALLING);

  xTaskCreate(lcd_task, "lcd", 1024, NULL, 1, NULL);
  xTaskCreate(state_task, "state", 1024, NULL, 2, NULL);
  xTaskCreate(bms_task, "bms", 1024, NULL, 3, NULL);
  xTaskCreate(torque_task, "torque", 1024, NULL, 4, NULL);
  vTaskStartScheduler();
}

void loop(void) {}
