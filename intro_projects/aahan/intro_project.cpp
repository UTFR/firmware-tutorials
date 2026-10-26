#include <arduino_freertos.h>
#include <stdint.h>
#include <semphr.h>

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

// Pins

#define TS_ON_PIN 12
#define RTD_PIN 13
#define CURRENT_PIN 19
#define CAN_RX_PIN 2
#define CAN_TX_PIN 3
#define MOSI_PIN 5
#define MISO_PIN 6
#define SCK_PIN 7
#define LCD_CS_PIN 8
#define BMS_CS_PIN 9
#define AIR_PLUS_PIN 22
#define PRECHARGE_PIN 23
#define AIR_MINUS_PIN 10

//TODO: confirm with lead - wheelspeed pins are not in the spec, so 14-17 were chosen arbitrarily
#define WHEEL_FL_PIN 14
#define WHEEL_FR_PIN 15
#define WHEEL_RL_PIN 16
#define WHEEL_RR_PIN 17

// Task periods (ms)
#define TORQUE_PERIOD_MS 1
#define BMS_PERIOD_MS 100
#define STATE_PERIOD_MS 50
#define LCD_PERIOD_MS 100

// Task priorities
#define TORQUE_PRIORITY (tskIDLE_PRIORITY + 4)
#define BMS_PRIORITY (tskIDLE_PRIORITY + 3)
#define STATE_PRIORITY (tskIDLE_PRIORITY + 2)
#define LCD_PRIORITY (tskIDLE_PRIORITY + 1)

// Tasks stack size (words)
#define TASK_STACK_DEPTH 1024

//TODO: confirm with lead - baudrate not in the spec
#define CAN_BAUDRATE 500000

// TODO: confirm with lead - onboarding doc says 6 segments x 23 cells
#define NUM_CELLS 138

// BMS safety limits (from the spec)
#define CELL_V_MIN 2.8f
#define CELL_V_MAX 4.3f
#define CELL_T_MAX 60.0f

// Current sensor: 10-bit ADC (0-1023 = 0-3.3 V), 1 A per 10 mV
// TODO: ask lead - this can't read negative (regen) current; real sensors use a mid-rail offset
#define ADC_MAX         1023.0f
#define ADC_VREF        3.3f
#define CURRENT_V_PER_A 0.010f

// TODO: confirm with lead - CAN IDs and steering encoding are not in the spec
#define STEERING_CAN_ID 0x10
#define TORQUE_CAN_ID   0x20

// TODO: confirm encoding with lead - assumed 4 x int16 torques, 0.1 Nm per bit, FL in bytes 0-1
#define TORQUE_SCALE 10.0f


//Shared resources
SemaphoreHandle_t spi_mutex; //protects the SPI bus (shared by LCD + BMS)
SemaphoreHandle_t data_mutex; //protects shared sensor / torque  / BMS values

// Precharge duration before closing AIR+
#define PRECHARGE_TIME_MS 5000

// Car state - written ONLY by the state machine task
enum class CarState { LV, PRECHARGING, TS, RTD, FAULT };
volatile CarState car_state = CarState::LV;

// BMS fault flag - set ONLY by the BMS task, never cleared (latching)
volatile bool bms_fault = false;

// Values shared between tasks - ONLY read or write while holding data_mutex
struct CarData {
  float current;          // HV current (A)
  float wheelspeeds[4];   // RPM, order FL, FR, RL, RR
  float steering_angle;   // degrees
  float torques[4];       // torque commands, order FL, FR, RL, RR
  float min_cell_v;       // lowest cell voltage (V)
  float max_cell_v;       // highest cell voltage (V)
  float max_cell_t;       // highest cell temperature (C)
};
CarData shared_data = {};

#define WHEEL_TEETH 17
#define WHEEL_STOP_TIMEOUT_US 300000  // no tooth for 300 ms -> treat wheel as stopped

// Written ONLY by the wheelspeed ISRs. Read by the torque task with interrupts off.
volatile uint32_t wheel_last_us[4] = {0, 0, 0, 0};    // when the last tooth passed
volatile uint32_t wheel_period_us[4] = {0, 0, 0, 0};  // time between the last two teeth

// Runs inside an interrupt: keep it tiny. No maths beyond a subtraction, no printing.
static void record_tooth(int wheel) {
  uint32_t now = micros();
  wheel_period_us[wheel] = now - wheel_last_us[wheel];
  wheel_last_us[wheel] = now;
}

static void wheel_fl_isr(void) { record_tooth(0); }
static void wheel_fr_isr(void) { record_tooth(1); }
static void wheel_rl_isr(void) { record_tooth(2); }
static void wheel_rr_isr(void) { record_tooth(3); }


// Opens every relay. Safe to call at any time, in any state.
static void open_all_relays(void) {
  digitalWrite(AIR_PLUS_PIN, arduino::LOW);
  digitalWrite(AIR_MINUS_PIN, arduino::LOW);
  digitalWrite(PRECHARGE_PIN, arduino::LOW);
}

// The ONLY task allowed to change car_state or touch the relays.
static void state_machine_task(void *p) {
  TickType_t last_wake = xTaskGetTickCount();
  TickType_t precharge_start = 0;
  // Start as "pressed" so a button held at power-on doesn't count as a press
  bool rtd_was_pressed = true;

  for (;;) {
    // 1. Fault check ALWAYS comes first
    if (bms_fault) {
      open_all_relays();
      car_state = CarState::FAULT;
    }

    // 2. State transitions (buttons are active low: pressed == LOW)
    bool ts_pressed = (digitalRead(TS_ON_PIN) == arduino::LOW);
    bool rtd_pressed = (digitalRead(RTD_PIN) == arduino::LOW);
    // Edge detection: only a fresh press (released -> pressed) counts
    bool rtd_new_press = rtd_pressed && !rtd_was_pressed;

    switch (car_state) {
      case CarState::LV:
        if (ts_pressed) {
          digitalWrite(AIR_MINUS_PIN, arduino::HIGH);
          digitalWrite(PRECHARGE_PIN, arduino::HIGH);
          precharge_start = xTaskGetTickCount();
          car_state = CarState::PRECHARGING;
        }
        break;

      case CarState::PRECHARGING:
        if (xTaskGetTickCount() - precharge_start >= pdMS_TO_TICKS(PRECHARGE_TIME_MS)) {
          digitalWrite(AIR_PLUS_PIN, arduino::HIGH);
          digitalWrite(PRECHARGE_PIN, arduino::LOW);
          car_state = CarState::TS;
        }
        break;

      case CarState::TS:
        if (rtd_new_press) {
          car_state = CarState::RTD;
        }
        break;

      case CarState::RTD:
      case CarState::FAULT:
        break;  // RTD only leaves on a fault; FAULT is latched forever
    }

    rtd_was_pressed = rtd_pressed;  // remember the button for next loop

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STATE_PERIOD_MS));
  }
}

// Scans every cell; on any unsafe reading, raises bms_fault.
// Never touches relays - only the state machine does that.
static void bms_task(void *p) {
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    float min_v = 0.0f, max_v = 0.0f, max_t = 0.0f;  // overwritten by cell 0
    bool unsafe = false;

    // Hold the SPI bus for the whole scan - only the LCD waits, and it can
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    for (uint8_t i = 0; i < NUM_CELLS; i++) {
      float v = bms_get_voltage(i);
      float t = bms_get_temperature(i);
      
      // Check that each value is INSIDE the safe range, so a NaN reading
      // (where every comparison is false) also counts as unsafe

      bool v_ok = (v >= CELL_V_MIN) && (v <= CELL_V_MAX);
      bool t_ok = (t <= CELL_T_MAX);
      if (!v_ok || !t_ok) {
        unsafe = true;
      }


      if (i == 0 || v < min_v) min_v = v;
      if (i == 0 || v > max_v) max_v = v;
      if (i == 0 || t > max_t) max_t = t;
    }
    xSemaphoreGive(spi_mutex);

    if (unsafe) {
      bms_fault = true;  // latched: never set back to false
    }

    // Publish for the LCD: lock, copy, unlock
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    shared_data.min_cell_v = min_v;
    shared_data.max_cell_v = max_v;
    shared_data.max_cell_t = max_t;
    xSemaphoreGive(data_mutex);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BMS_PERIOD_MS));
  }
}

// ---- Torque task helpers. CAN: ONLY the torque task may call can_send/can_receive. ----

static float read_current(void) {
  float adc_value = analogRead(CURRENT_PIN);
  float volts = (adc_value / ADC_MAX) * ADC_VREF;
  return volts / CURRENT_V_PER_A;
}

static void read_wheelspeeds(float *rpm_out) {
  uint32_t last[4], period[4];

  // Copy the ISR data with interrupts briefly off, so each pair matches
  noInterrupts();
  for (int w = 0; w < 4; w++) {
    last[w] = wheel_last_us[w];
    period[w] = wheel_period_us[w];
  }
  interrupts();

  uint32_t now = micros();
  for (int w = 0; w < 4; w++) {
    if (now - last[w] > WHEEL_STOP_TIMEOUT_US || period[w] == 0) {
      rpm_out[w] = 0.0f;  // no recent tooth: wheel is stopped
    } else {
      rpm_out[w] = 60.0f * 1000000.0f / ((float)period[w] * WHEEL_TEETH);
    }
  }
}

static float read_steering(void) {
  uint64_t payload = 0;
  can_receive(&payload, STEERING_CAN_ID);
  // TODO: confirm encoding - assumed bytes 0-1, signed, 0.1 degrees per bit
  int16_t raw = (int16_t)(payload & 0xFFFF);
  return raw * 0.1f;
}

// Packs 4 torques into one 8-byte CAN payload: 2 bytes each, FL lowest
static uint64_t pack_torques(const float *torques) {
  uint64_t payload = 0;
  for (int m = 0; m < 4; m++) {
    int16_t raw = (int16_t)(torques[m] * TORQUE_SCALE);    // e.g. 12.3 Nm -> 123
    payload |= (uint64_t)(uint16_t)raw << (16 * m);        // place in bytes 2m, 2m+1
  }
  return payload;
}

// Highest priority: runs every 1 ms. Keep it SHORT - it must never be late.
static void torque_task(void *p) {
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    float current = read_current();
    float wheelspeeds[4];
    read_wheelspeeds(wheelspeeds);
    float steering = read_steering();

    float torques[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // default: no torque
    if (car_state == CarState::RTD) {
      calculate_torque_cmd(torques, current, wheelspeeds, steering);
    }

    can_send(TORQUE_CAN_ID, pack_torques(torques));

    xSemaphoreTake(data_mutex, portMAX_DELAY);
    shared_data.current = current;
    for (int i = 0; i < 4; i++) {
      shared_data.wheelspeeds[i] = wheelspeeds[i];
    }
    shared_data.steering_angle = steering;
    for (int i = 0; i < 4; i++) {
      shared_data.torques[i] = torques[i];
    }
    xSemaphoreGive(data_mutex);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TORQUE_PERIOD_MS));
  }
}

// Turns a CarState into text for the dash
static const char *state_name(CarState s) {
  switch (s) {
    case CarState::LV:          return "LV";
    case CarState::PRECHARGING: return "PRECHARGING";
    case CarState::TS:          return "TS";
    case CarState::RTD:         return "RTD";
    case CarState::FAULT:       return "FAULT";
  }
  return "?";
}

// Lowest priority: display only. Hold each mutex as briefly as possible.
static void lcd_task(void *p) {
  TickType_t last_wake = xTaskGetTickCount();
  char text[200];

  for (;;) {
    // 1. Snapshot the shared data: lock, copy, unlock
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    CarData d = shared_data;
    xSemaphoreGive(data_mutex);
    CarState state = car_state;

    // 2. Format the text BEFORE touching the SPI bus
    snprintf(
      text, sizeof(text),
      "%s  %.1fA  Str %.1fdeg\n"
      "RPM %.0f %.0f %.0f %.0f\n"
      "Nm %.1f %.1f %.1f %.1f\n"
      "V %.2f-%.2f  T %.1fC",
      state_name(state), d.current, d.steering_angle,
      d.wheelspeeds[0], d.wheelspeeds[1], d.wheelspeeds[2], d.wheelspeeds[3],
      d.torques[0], d.torques[1], d.torques[2], d.torques[3],
      d.min_cell_v, d.max_cell_v, d.max_cell_t
    );
    // 3. Hold the SPI bus only for the actual print
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    lcd_printf("%s", text);
    xSemaphoreGive(spi_mutex);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LCD_PERIOD_MS));
  }
}

void setup(void) {
  //Relays: outputs, forced OPEN (LOW) first so HV stays inside the accumulator
  pinMode(AIR_PLUS_PIN, arduino::OUTPUT);
  digitalWrite(AIR_PLUS_PIN, arduino::LOW);
  pinMode(AIR_MINUS_PIN, arduino::OUTPUT);
  digitalWrite(AIR_MINUS_PIN, arduino::LOW);
  pinMode(PRECHARGE_PIN, arduino::OUTPUT);
  digitalWrite(PRECHARGE_PIN, arduino::LOW);

  //Dash buttons: active low, so use the internal pull-up resistor
  pinMode(TS_ON_PIN, arduino::INPUT_PULLUP);
  pinMode(RTD_PIN, arduino::INPUT_PULLUP);

  // Wheelspeed sensors: normally high, pulled low each time a tooth passes
  pinMode(WHEEL_FL_PIN, arduino::INPUT_PULLUP);
  pinMode(WHEEL_FR_PIN, arduino::INPUT_PULLUP);
  pinMode(WHEEL_RL_PIN, arduino::INPUT_PULLUP);
  pinMode(WHEEL_RR_PIN, arduino::INPUT_PULLUP);
 
  // Peripherals (CAN, plus LCD, and BMS sharing one SPI bus)
  can_init(CAN_RX_PIN, CAN_TX_PIN, CAN_BAUDRATE);
  lcd_init(MOSI_PIN, MISO_PIN, SCK_PIN, LCD_CS_PIN);
  bms_init(MOSI_PIN, MISO_PIN, SCK_PIN, BMS_CS_PIN);

  // Mutexes: must exist before any task that uses them is created
  spi_mutex = xSemaphoreCreateMutex();
  data_mutex = xSemaphoreCreateMutex();

  // If a mutex couldn't be created, stop here. The relays are already open,
  // so halting is the safe way to fail.
  if (spi_mutex == NULL || data_mutex == NULL) {
    for (;;) {}
  }

  attachInterrupt(digitalPinToInterrupt(WHEEL_FL_PIN), wheel_fl_isr, arduino::FALLING);
  attachInterrupt(digitalPinToInterrupt(WHEEL_FR_PIN), wheel_fr_isr, arduino::FALLING);
  attachInterrupt(digitalPinToInterrupt(WHEEL_RL_PIN), wheel_rl_isr, arduino::FALLING);
  attachInterrupt(digitalPinToInterrupt(WHEEL_RR_PIN), wheel_rr_isr, arduino::FALLING);

  xTaskCreate(state_machine_task, "state", TASK_STACK_DEPTH, NULL, STATE_PRIORITY, NULL);
  xTaskCreate(bms_task, "bms", TASK_STACK_DEPTH, NULL, BMS_PRIORITY, NULL);
  xTaskCreate(torque_task, "torque", TASK_STACK_DEPTH, NULL, TORQUE_PRIORITY, NULL);
  xTaskCreate(lcd_task, "lcd", TASK_STACK_DEPTH, NULL, LCD_PRIORITY, NULL);

  vTaskStartScheduler();   // hands control to FreeRTOS; never returns

  
}
void loop(void) {}
