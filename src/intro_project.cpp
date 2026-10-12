#include <arduino_freertos.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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

TS On:          12 // active low
RTD:            13 //active low

Current Sensor: 19 //read voltage in mv and convert to amps
CAN RX:         2 //read
CAN TX:         3 //write
MOSI:           5 //spi
MISO:           6 //spi
SCK:            7
LCD_CS:         8 //LCD screen
BMS_CS:         9 //battery manager

AIR+:           22 //relay +
Precharge:      23 //
AIR-:           10 //relay -

//startup sequence LV -> TS:
* close AIR-
* close precharge
* 5 seconds
* close AIR +, open precharge

TS -> RTD
* TS not able to drive (but energized)
* RTD can drive

requirement 1: 1ms
give motors torque:
* get current from pin 19 (in mV) / 10 = Amps
* get wheelspeeds (??)
* calculate torques (*, current, )

requirement 2: 100 ms
* read data from bms

requirement 3:
* read data from BMS
* if exceeds any values, open all relays and de-energize the car




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

// MY CODE FROM HERE ON
// =================================================================================

// semaphore for spi communication
SemaphoreHandle_t m;

// wheel speeds etc. currently as of last update
float wheel_speeds[4] = {0, 0, 0, 0};
float torques[4] = {0, 0, 0, 0};
float steering_angle = 0, curr_current = 0;
uint8_t STEERING_ID = 0xFB; // id for steering angle can recieve //CAN ID
uint8_t torque_ids[4] = {
  0xFC, 0xFD, 0xFE, 0xFF
}; // placeholder set to something CAN IDS

// error happened somewhere
volatile bool faulted = false;

// cells and bms monitoring
constexpr uint8_t MAX_CELLS =
  8; // default number of cells on car i dont know...
float cell_temps[MAX_CELLS];
float cell_volts[MAX_CELLS];

// states
uint8_t STATE_LV = 0;
uint8_t STATE_TS = 1;
uint8_t STATE_RTD = 2;
uint8_t STATE_FAULT = 3;

volatile int mode = STATE_LV; // 0 = LV, 1 = TS, 2 = RTD

// pins
uint8_t TS = 12;
uint8_t RTD = 13;
uint8_t CURRENT = 19;
uint8_t CANRX = 2;
uint8_t CANTX = 3;
uint8_t PIN_MOSI = 5;
uint8_t PIN_MISO = 6;
uint8_t PIN_SCK = 7;
uint8_t LCD_CS = 8;
uint8_t BMS_CS = 9;
uint8_t AIRP = 22;
uint8_t PRECHARGE = 23;
uint8_t AIRN = 10;

// define wheel speed (CAN IDS FOR SENSORS)
uint8_t FL = 14; // front left ... etc
uint8_t FR = 15;
uint8_t RL = 16;
uint8_t RR = 17;

// moving LV -> TS -> RTD
void relay_close(uint8_t pin) { digitalWrite(pin, HIGH); }
void relay_open(uint8_t pin) { digitalWrite(pin, LOW); }

void open_relays() {
  relay_open(AIRP);
  relay_open(AIRN);
  relay_open(PRECHARGE);
}
void deenergise() {
  open_relays();
  faulted = true;
  mode = STATE_FAULT;
}
// used to check whether there was a fault while loading car into TS
bool wait(uint32_t total_ms) {
  const TickType_t start = xTaskGetTickCount();
  const TickType_t total = pdMS_TO_TICKS(total_ms);

  while((xTaskGetTickCount() - start) < total) {
    if(faulted)
      return false;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return !faulted;
}

void TS_ON() {
  if(faulted || mode != STATE_LV)
    return;
  // set pinmodes
  relay_close(AIRN);
  relay_close(PRECHARGE);

  // catch an error in startups
  if(!wait(5000)) {
    deenergise();
    return;
  }

  relay_close(AIRP);
  relay_open(PRECHARGE);

  mode = STATE_TS;
}
void RTD_ON() {
  if(mode == STATE_TS && !faulted)
    mode = STATE_RTD;
}

void init_pins() {
  pinMode(AIRN, OUTPUT);
  pinMode(AIRP, OUTPUT);
  pinMode(PRECHARGE, OUTPUT);

  pinMode(TS, INPUT_PULLUP);
  pinMode(RTD, INPUT_PULLUP);

  pinMode(FL, INPUT_PULLUP);
  pinMode(FR, INPUT_PULLUP);
  pinMode(RL, INPUT_PULLUP);
  pinMode(RR, INPUT_PULLUP);

  open_relays();
}

float read_current() {
  int raw = analogRead(CURRENT);
  float mv = raw * 3300.0f / 4095.0f;
  curr_current = mv / 10.0f;
  return curr_current;
}

volatile uint32_t last_edge_us[4];
volatile uint32_t period_us[4];

void get_period(uint8_t i) {
  uint32_t now = micros();
  period_us[i] = now - last_edge_us[i];
  last_edge_us[i] = now;
}

void get_per_FL() { get_period(0); }
void get_per_FR() { get_period(1); }
void get_per_RL() { get_period(2); }
void get_per_RR() { get_period(3); }

float read_wheelspeed_rpm(uint8_t i) {
  noInterrupts();
  uint32_t last = last_edge_us[i];
  uint32_t period = period_us[i];
  interrupts();

  if(period == 0 || (micros() - last) > 100000)
    return 0.0f; // no input for 100 ms is 0
  return 60000000.0f / (17 * (float)period);
}
float read_steering_angle() {
  uint64_t raw = 0;
  can_receive(&raw, STEERING_ID);

  uint32_t temp = (uint32_t)(raw); // convert to uint32
  memcpy(&steering_angle, &temp, sizeof(float));

  return steering_angle;
}
void send_torques(float *torques) {
  for(uint8_t x = 0; x < 4; x++) {
    uint64_t torque;
    memcpy(&torque, &torques[x], sizeof(float));
    can_send(torque_ids[x], torque);
  }
}
void do_torques() {
  float current = read_current();
  for(uint8_t x = 0; x < 4; x++)
    wheel_speeds[x] = read_wheelspeed_rpm(x);
  float angle = read_steering_angle();

  if(mode == STATE_RTD && !faulted) {
    calculate_torque_cmd(torques, current, wheel_speeds, angle);
  } else { // send 0 torque when not in RTD
    for(uint8_t x = 0; x < 4; x++)
      torques[x] = 0;
  }
  send_torques(torques);
}
bool read_cell(uint8_t i, float *v, float *t) {
  if(xSemaphoreTake(m, pdMS_TO_TICKS(5)) != pdTRUE)
    return false;
  *v = bms_get_voltage(i);
  *t = bms_get_temperature(i);

  xSemaphoreGive(m);
  return true;
}

void do_bms() {
  int count_failed = 0;
  for(uint8_t x = 0; x < MAX_CELLS; x++) {
    bool failed = !read_cell(x, &cell_volts[x], &cell_temps[x]);
    if(failed) {
      count_failed++;
      continue;
    }
    if(cell_volts[x] < 2.8f || cell_volts[x] > 4.3f || cell_temps[x] > 60.0f) {
      deenergise();
      return;
    }
  }
  if(count_failed > 2)
    deenergise(); // if too many battery measurements skipped then deenergise
                  // cuz smth happenign
}

void do_print() {
  // get lowest & highest battery voltages & highest temp
  float vl = 100, vh = 0,
        th = 0; // could be coded better but irrelevant i think

  for(uint8_t x = 0; x < MAX_CELLS; x++) {
    vl = cell_volts[x] < vl ? cell_volts[x] : vl;
    vh = cell_volts[x] > vh ? cell_volts[x] : vh;
    th = cell_temps[x] > th ? cell_temps[x] : th;
  }
  char batt[64];
  char tors[64];
  char wheel[64];

  snprintf(batt, sizeof(batt), "VL: %.1fV VH: %.1fV TH: %.1fC", vl, vh, th);
  snprintf(
    tors, sizeof(tors), "ang: %.1f FLT: %.1f FRT: %.1f RLT: %.1f RRT: %.1f",
    steering_angle, torques[0], torques[1], torques[2], torques[3]
  );
  snprintf(
    wheel, sizeof(wheel), "curr: %.1f FLS: %.1f FRS: %.1f RLS: %.1f RRS: %.1f",
    curr_current, wheel_speeds[0], wheel_speeds[1], wheel_speeds[2],
    wheel_speeds[3]
  );

  xSemaphoreTake(m, portMAX_DELAY);
  lcd_printf("%s\n%s\n%s\n", batt, tors, wheel);
  xSemaphoreGive(m);
  // steering angle, torques

  // do later TODO
}

void torque_task(void *) {
  TickType_t last = xTaskGetTickCount();
  for(;;) {
    do_torques();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
  }
}

void bms_task(void *) {
  TickType_t last = xTaskGetTickCount();
  for(;;) {
    do_bms();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
  }
}

void lcd_task(void *) {
  TickType_t last = xTaskGetTickCount();
  for(;;) {
    do_print();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
  }
}

void state_task(void *) {
  TickType_t last = xTaskGetTickCount();
  for(;;) {
    // check_pins
    bool currTS = digitalRead(TS) == LOW;
    bool currRTD = digitalRead(RTD) == LOW;

    if(currTS)
      TS_ON();
    else if(currRTD)
      RTD_ON();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(50)); // update every 50 ms
  }
}

void setup(void) {
  init_pins();
  m = xSemaphoreCreateMutex();

  // initialise all the initialisations
  can_init(CANRX, CANTX, 500000);
  lcd_init(PIN_MOSI, PIN_MISO, PIN_SCK, LCD_CS);
  bms_init(PIN_MOSI, PIN_MISO, PIN_SCK, BMS_CS);

  analogReadResolution(12);

  attachInterrupt(digitalPinToInterrupt(FL), get_per_FL, FALLING);
  attachInterrupt(digitalPinToInterrupt(FR), get_per_FR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RL), get_per_RL, FALLING);
  attachInterrupt(digitalPinToInterrupt(RR), get_per_RR, FALLING);

  xTaskCreate(torque_task, "torque", 2048, NULL, 4, NULL);
  xTaskCreate(bms_task, "bms", 2048, NULL, 3, NULL);
  xTaskCreate(lcd_task, "lcd", 2048, NULL, 1, NULL);
  xTaskCreate(state_task, "state", 2048, NULL, 2, NULL);

  vTaskStartScheduler();
}

void loop(void) {}
