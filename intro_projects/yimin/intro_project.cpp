#include <arduino_freertos.h>
#include <semphr.h>
#include <cmath>
#include <cstdio>
#include <stdint.h>

enum class VehicleState : uint8_t {
  LV,
  TS,
  RTD,
};

enum class PrechargeState : uint8_t {
  IDLE,
  IN_PROGRESS,
};

enum class FaultReason : uint8_t {
  NONE,
  CELL_UNDERVOLTAGE,
  CELL_OVERVOLTAGE,
  CELL_OVERTEMPERATURE,
  INVALID_BMS_READING,
};

struct BmsSummary {
  float lowest_voltage;
  float highest_voltage;
  float highest_temperature;
  FaultReason fault;
};

struct ButtonDebouncer {
  uint8_t pin;
  bool previous_raw_state;
  bool stable_state;
  TickType_t last_change_time;
};

constexpr uint8_t TS_ON_PIN = 12;
constexpr uint8_t RTD_PIN = 13;
constexpr uint8_t CURRENT_SENSOR_PIN = 19;
constexpr uint8_t AIR_POSITIVE_PIN = 22;
constexpr uint8_t PRECHARGE_PIN = 23;
constexpr uint8_t AIR_NEGATIVE_PIN = 10;
constexpr uint8_t BMS_CS_PIN = 9;
constexpr uint8_t LCD_CS_PIN = 8;
constexpr uint8_t SPI_MOSI_PIN = 5;
constexpr uint8_t SPI_MISO_PIN = 6;
constexpr uint8_t SPI_SCK_PIN = 7;
constexpr uint8_t CAN_RX_PIN =2;
constexpr uint8_t CAN_TX_PIN =3;
// TODO: Confirm baud rate, message ID, and angle encoding.
constexpr uint32_t CAN_BAUDRATE = 500000;
constexpr uint32_t STEERING_CAN_ID =0X10;
// TODO: Confirm torque CAN ID and payload encoding.
constexpr uint8_t TORQUE_CAN_ID = 0x20;
constexpr float TORQUE_COUNTS_PER_NM = 10.0F;

// TODO: Confirm the cell count with the actual accumulator configuration.
constexpr uint8_t BMS_CELL_COUNT = 138;

constexpr uint8_t WHEEL_COUNT = 4;
// TODO: Confirm these wheel-sensor pins; the tutorial does not specify them.
constexpr uint8_t WHEEL_PINS[WHEEL_COUNT] = {14, 15, 16, 17};
constexpr uint8_t WHEEL_TEETH = 17;
constexpr uint32_t WHEEL_STOP_TIMEOUT_US = 300000;

constexpr uint8_t RELAY_DEENERGIZED = arduino::LOW;
constexpr uint8_t RELAY_ENERGIZED = arduino::HIGH;

constexpr uint8_t ADC_RESOLUTION_BITS = 12;
constexpr uint16_t ADC_MAX_READING = (1U << ADC_RESOLUTION_BITS) - 1U;
constexpr float ADC_REFERENCE_VOLTAGE = 3.3F;
constexpr float CURRENT_SENSOR_VOLTS_PER_AMP = 0.010F;

constexpr uint16_t CONTROL_TASK_STACK_DEPTH = 512;
constexpr UBaseType_t CONTROL_TASK_PRIORITY = tskIDLE_PRIORITY + 2;
constexpr TickType_t CONTROL_TASK_PERIOD = pdMS_TO_TICKS(1);

constexpr uint16_t STATE_TASK_STACK_DEPTH = 512;
constexpr UBaseType_t STATE_TASK_PRIORITY = tskIDLE_PRIORITY + 1;
constexpr TickType_t STATE_TASK_PERIOD = pdMS_TO_TICKS(10);
constexpr TickType_t BUTTON_DEBOUNCE_TIME = pdMS_TO_TICKS(30);
constexpr TickType_t PRECHARGE_DURATION = pdMS_TO_TICKS(5000);

constexpr uint16_t BMS_TASK_STACK_DEPTH = 512;
constexpr UBaseType_t BMS_TASK_PRIORITY = tskIDLE_PRIORITY + 1;
constexpr TickType_t BMS_TASK_PERIOD = pdMS_TO_TICKS(100);

constexpr uint16_t LCD_TASK_STACK_DEPTH = 1024;
constexpr UBaseType_t LCD_TASK_PRIORITY = tskIDLE_PRIORITY + 1;
constexpr TickType_t LCD_TASK_PERIOD = pdMS_TO_TICKS(100);

constexpr float MIN_CELL_VOLTAGE = 2.8F;
constexpr float MAX_CELL_VOLTAGE = 4.3F;
constexpr float MAX_CELL_TEMPERATURE = 60.0F;

struct ControlSnapshot {
  float current_amps;
  float wheel_rpm[WHEEL_COUNT];
  float steering_angle;
  float torques[WHEEL_COUNT];
};

VehicleState vehicle_state = VehicleState::LV;
PrechargeState precharge_state = PrechargeState::IDLE;
FaultReason fault_reason = FaultReason::NONE;
// Allow precharge only after a complete, valid BMS scan.
bool bms_monitor_ready = false;
BmsSummary latest_bms_summary {0.0F, 0.0F, 0.0F,
                               FaultReason::INVALID_BMS_READING};
ControlSnapshot latest_control = {};
SemaphoreHandle_t spi_mutex = nullptr;

volatile uint32_t wheel_last_pulse_us[WHEEL_COUNT] = {};
volatile uint32_t wheel_period_us[WHEEL_COUNT] = {};

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

float read_current_amps(void) {
  const uint16_t raw_reading = analogRead(CURRENT_SENSOR_PIN);
  const float sensor_voltage =
    (static_cast<float>(raw_reading) / ADC_MAX_READING) * ADC_REFERENCE_VOLTAGE;

  return sensor_voltage / CURRENT_SENSOR_VOLTS_PER_AMP;
}

float rpm_from_period(uint32_t period_us) {
  if(period_us == 0) {
    return 0.0F;
  }
  return 60000000.0F /
         (WHEEL_TEETH * static_cast<float>(period_us));
}

void record_wheel_pulse(uint8_t wheel) {
  const uint32_t now = micros();
  const uint32_t previous = wheel_last_pulse_us[wheel];
  if(previous != 0) {
    wheel_period_us[wheel] = now - previous;
  }
  wheel_last_pulse_us[wheel] = now;
}

void wheel_fl_isr(void) { record_wheel_pulse(0); }
void wheel_fr_isr(void) { record_wheel_pulse(1); }
void wheel_rl_isr(void) { record_wheel_pulse(2); }
void wheel_rr_isr(void) { record_wheel_pulse(3); }

void read_wheel_rpm(float *rpm) {
  uint32_t last[WHEEL_COUNT];
  uint32_t period[WHEEL_COUNT];

  // Copy ISR data together so each wheel's time and period match.
  noInterrupts();
  for(uint8_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
    last[wheel] = wheel_last_pulse_us[wheel];
    period[wheel] = wheel_period_us[wheel];
  }
  interrupts();

  const uint32_t now = micros();
  for(uint8_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
    rpm[wheel] = (last[wheel] == 0 ||
                  now - last[wheel] > WHEEL_STOP_TIMEOUT_US)
                   ? 0.0F
                   : rpm_from_period(period[wheel]);
  }
}

float read_steering_angle(void){
  uint64_t payload =0;
  can_receive(&payload, STEERING_CAN_ID);

  // assume the lowest 16bits represent angle in units of 0.1 degree.
  const int16_t raw_angle =static_cast <int16_t>(payload & 0xffffu);
  return static_cast<float>(raw_angle) * 0.1f;
}

uint64_t pack_torques(const float *torques){
  uint64_t payload = 0;

  for(uint8_t motor =0; motor < WHEEL_COUNT; ++motor){
    const int16_t raw = static_cast<int16_t>(torques[motor]*TORQUE_COUNTS_PER_NM);
    payload |= static_cast<uint64_t>(static_cast<uint16_t>(raw)) << (16U * motor);
  }

  return payload;
}

bool read_button_pressed(uint8_t pin) {
  return digitalReadFast(pin) == arduino::LOW;
}

void initialize_button(ButtonDebouncer &button, TickType_t now) {
  const bool pressed = read_button_pressed(button.pin);
  button.previous_raw_state = pressed;
  button.stable_state = pressed;
  button.last_change_time = now;
}

bool update_button_pressed_edge(ButtonDebouncer &button, TickType_t now) {
  const bool raw_state = read_button_pressed(button.pin);

  if(raw_state != button.previous_raw_state) {
    button.previous_raw_state = raw_state;
    button.last_change_time = now;
  }

  if(raw_state != button.stable_state &&
     now - button.last_change_time >= BUTTON_DEBOUNCE_TIME) {
    button.stable_state = raw_state;
    return button.stable_state;
  }

  return false;
}

void open_all_relays(void) {
  digitalWrite(PRECHARGE_PIN, RELAY_DEENERGIZED);
  digitalWrite(AIR_POSITIVE_PIN, RELAY_DEENERGIZED);
  digitalWrite(AIR_NEGATIVE_PIN, RELAY_DEENERGIZED);
  precharge_state = PrechargeState::IDLE;
  vehicle_state = VehicleState::LV;
}

// Call from a FreeRTOS task, not from an interrupt. The first fault stays
// latched until the controller is reset.
void shutdown_for_fault(FaultReason reason) {
  if(reason == FaultReason::NONE) {
    return;
  }

  taskENTER_CRITICAL();
  if(fault_reason == FaultReason::NONE) {
    fault_reason = reason;
  }
  bms_monitor_ready = false;
  open_all_relays();
  taskEXIT_CRITICAL();
}

FaultReason classify_bms_reading(float cell_voltage,
                                 float cell_temperature) {
  // A failed sensor read must not be treated as a safe cell measurement.
  if(!std::isfinite(cell_voltage) || !std::isfinite(cell_temperature)) {
    return FaultReason::INVALID_BMS_READING;
  }
  if(cell_voltage < MIN_CELL_VOLTAGE) {
    return FaultReason::CELL_UNDERVOLTAGE;
  }
  if(cell_voltage > MAX_CELL_VOLTAGE) {
    return FaultReason::CELL_OVERVOLTAGE;
  }
  if(cell_temperature > MAX_CELL_TEMPERATURE) {
    return FaultReason::CELL_OVERTEMPERATURE;
  }
  return FaultReason::NONE;
}

void check_bms_reading(float cell_voltage, float cell_temperature) {
  const FaultReason reason =
    classify_bms_reading(cell_voltage, cell_temperature);
  if(reason != FaultReason::NONE) {
    shutdown_for_fault(reason);
  }
}

BmsSummary summarize_bms_readings(const float *voltages,
                                  const float *temperatures,
                                  uint8_t cell_count) {
  BmsSummary summary { 0.0F, 0.0F, 0.0F,
                       FaultReason::INVALID_BMS_READING };
  if(voltages == nullptr || temperatures == nullptr || cell_count == 0) {
    return summary;
  }

  for(uint8_t cell = 0; cell < cell_count; ++cell) {
    const float voltage = voltages[cell];
    const float temperature = temperatures[cell];
    const FaultReason reason = classify_bms_reading(voltage, temperature);
    if(reason != FaultReason::NONE) {
      summary.fault = reason;
      return summary;
    }

    if(cell == 0) {
      summary.lowest_voltage = voltage;
      summary.highest_voltage = voltage;
      summary.highest_temperature = temperature;
    } else {
      if(voltage < summary.lowest_voltage) {
        summary.lowest_voltage = voltage;
      }
      if(voltage > summary.highest_voltage) {
        summary.highest_voltage = voltage;
      }
      if(temperature > summary.highest_temperature) {
        summary.highest_temperature = temperature;
      }
    }
  }

  summary.fault = FaultReason::NONE;
  return summary;
}

void begin_precharge(void) {
  digitalWrite(AIR_NEGATIVE_PIN, RELAY_ENERGIZED);
  digitalWrite(PRECHARGE_PIN, RELAY_ENERGIZED);
  precharge_state = PrechargeState::IN_PROGRESS;
}

void complete_precharge(void) {
  digitalWrite(AIR_POSITIVE_PIN, RELAY_ENERGIZED);
  digitalWrite(PRECHARGE_PIN, RELAY_DEENERGIZED);
  precharge_state = PrechargeState::IDLE;
  vehicle_state = VehicleState::TS;
}

void control_task(void *) {
  TickType_t previous_wake_time = xTaskGetTickCount();

  for(;;) {
    const float current_amps = read_current_amps();
    float wheel_rpm[WHEEL_COUNT];
    read_wheel_rpm(wheel_rpm);
    const float steering_angle = read_steering_angle();

    float torques[WHEEL_COUNT] = {};
    taskENTER_CRITICAL();
    const bool drive_enabled = vehicle_state == VehicleState::RTD &&
                               fault_reason == FaultReason::NONE &&
                               bms_monitor_ready;
    taskEXIT_CRITICAL();

    if(drive_enabled) {
      calculate_torque_cmd(torques, current_amps, wheel_rpm, steering_angle);
    }

    can_send(TORQUE_CAN_ID,pack_torques(torques));

    taskENTER_CRITICAL();
    latest_control.current_amps = current_amps;
    latest_control.steering_angle = steering_angle;
    for(uint8_t motor = 0; motor < WHEEL_COUNT; ++motor) {
      latest_control.wheel_rpm[motor] = wheel_rpm[motor];
      latest_control.torques[motor] = torques[motor];
    }
    taskEXIT_CRITICAL();

    vTaskDelayUntil(&previous_wake_time, CONTROL_TASK_PERIOD);
  }
}

void state_task(void *) {
  TickType_t previous_wake_time = xTaskGetTickCount();
  TickType_t precharge_start_time = 0;
  ButtonDebouncer ts_on_button { TS_ON_PIN, false, false, 0 };
  ButtonDebouncer rtd_button { RTD_PIN, false, false, 0 };

  initialize_button(ts_on_button, previous_wake_time);
  initialize_button(rtd_button, previous_wake_time);

  //state fsm
  for(;;) {
    const TickType_t now = xTaskGetTickCount();
    const bool ts_on_pressed = update_button_pressed_edge(ts_on_button, now);
    const bool rtd_pressed = update_button_pressed_edge(rtd_button, now);

    // Keep every relay transition atomic with a fault shutdown from another
    // task, so a delayed precharge step cannot re-energize a faulted car.
    taskENTER_CRITICAL();

    if(fault_reason != FaultReason::NONE || !bms_monitor_ready) {
      open_all_relays();
      taskEXIT_CRITICAL();
      vTaskDelayUntil(&previous_wake_time, STATE_TASK_PERIOD);
      continue;
    }

    if(
      vehicle_state == VehicleState::LV &&
      precharge_state == PrechargeState::IDLE &&
      fault_reason == FaultReason::NONE && bms_monitor_ready && ts_on_pressed
    ) {
      precharge_start_time = now;
      begin_precharge();
    } // LV + IDLE + TS On pressed

    if(precharge_state == PrechargeState::IN_PROGRESS &&
       now - precharge_start_time >= PRECHARGE_DURATION) {
      complete_precharge();
    } // Precharge exceed 5 s

    if(vehicle_state == VehicleState::TS && rtd_pressed) {
      vehicle_state = VehicleState::RTD;
    } // TS + RTD pressed

    taskEXIT_CRITICAL();

    vTaskDelayUntil(&previous_wake_time, STATE_TASK_PERIOD);
  }
}

void bms_task(void *) {
  TickType_t previous_wake_time = xTaskGetTickCount();

  for(;;) {
    BmsSummary summary {0.0F, 0.0F, 0.0F, FaultReason::NONE};

    // The LCD and BMS share one SPI bus.
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    for(uint8_t cell = 0; cell < BMS_CELL_COUNT; ++cell) {
      const float voltage = bms_get_voltage(cell);
      const float temperature = bms_get_temperature(cell);
      summary.fault = classify_bms_reading(voltage, temperature);
      if(summary.fault != FaultReason::NONE) {
        break;
      }

      if(cell == 0) {
        summary.lowest_voltage = voltage;
        summary.highest_voltage = voltage;
        summary.highest_temperature = temperature;
      } else {
        if(voltage < summary.lowest_voltage) summary.lowest_voltage = voltage;
        if(voltage > summary.highest_voltage) summary.highest_voltage = voltage;
        if(temperature > summary.highest_temperature)
          summary.highest_temperature = temperature;
      }
    }
    xSemaphoreGive(spi_mutex);

    if(summary.fault != FaultReason::NONE) {
      shutdown_for_fault(summary.fault);
    } else {
      taskENTER_CRITICAL();
      if(fault_reason == FaultReason::NONE) {
        latest_bms_summary = summary;
        bms_monitor_ready = true;
      }
      taskEXIT_CRITICAL();
    }

    vTaskDelayUntil(&previous_wake_time, BMS_TASK_PERIOD);
  }
}

void lcd_task(void *) {
  TickType_t previous_wake_time = xTaskGetTickCount();
  char bms_line[64];
  char display[256];

  for(;;) {
    taskENTER_CRITICAL();
    const BmsSummary summary = latest_bms_summary;
    const ControlSnapshot control = latest_control;
    const VehicleState state = vehicle_state;
    const FaultReason fault = fault_reason;
    const bool bms_ready = bms_monitor_ready;
    taskEXIT_CRITICAL();

    if(bms_ready) {
      std::snprintf(bms_line, sizeof(bms_line), "BMS:%.2f-%.2fV %.1fC",
                    summary.lowest_voltage, summary.highest_voltage,
                    summary.highest_temperature);
    } else {
      std::snprintf(bms_line, sizeof(bms_line), "BMS unavailable");
    }

    std::snprintf(display, sizeof(display),
                  "S:%u F:%u I:%.1fA Str:%.1fdeg\n"
                  "RPM:%.0f %.0f %.0f %.0f\n"
                  "Nm:%.1f %.1f %.1f %.1f\n%s",
                  static_cast<unsigned>(state), static_cast<unsigned>(fault),
                  control.current_amps, control.steering_angle,
                  control.wheel_rpm[0], control.wheel_rpm[1],
                  control.wheel_rpm[2], control.wheel_rpm[3],
                  control.torques[0], control.torques[1],
                  control.torques[2], control.torques[3], bms_line);

    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    lcd_printf("%s", display);
    xSemaphoreGive(spi_mutex);

    vTaskDelayUntil(&previous_wake_time, LCD_TASK_PERIOD);
  }
}

void setup(void) {
  // Both dashboard buttons are active-low, so the internal pull-ups keep an
  // unpressed or disconnected input at a known HIGH level.
  pinMode(TS_ON_PIN, arduino::INPUT_PULLUP);
  pinMode(RTD_PIN, arduino::INPUT_PULLUP);

  analogReadResolution(ADC_RESOLUTION_BITS);
  pinMode(CURRENT_SENSOR_PIN, arduino::INPUT);

  // Load the safe output value before enabling the pins as outputs. This keeps
  // the high-voltage relays de-energized throughout startup.
  digitalWrite(AIR_POSITIVE_PIN, RELAY_DEENERGIZED);
  digitalWrite(PRECHARGE_PIN, RELAY_DEENERGIZED);
  digitalWrite(AIR_NEGATIVE_PIN, RELAY_DEENERGIZED);

  pinMode(AIR_POSITIVE_PIN, arduino::OUTPUT);
  pinMode(PRECHARGE_PIN, arduino::OUTPUT);
  pinMode(AIR_NEGATIVE_PIN, arduino::OUTPUT);

  for(uint8_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
    pinMode(WHEEL_PINS[wheel], arduino::INPUT_PULLUP);
  }
  attachInterrupt(WHEEL_PINS[0], wheel_fl_isr, arduino::FALLING);
  attachInterrupt(WHEEL_PINS[1], wheel_fr_isr, arduino::FALLING);
  attachInterrupt(WHEEL_PINS[2], wheel_rl_isr, arduino::FALLING);
  attachInterrupt(WHEEL_PINS[3], wheel_rr_isr, arduino::FALLING);

  spi_mutex = xSemaphoreCreateMutex();
  if(spi_mutex == nullptr) {
    open_all_relays();
    for(;;) {}
  }

  bms_init(SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN, BMS_CS_PIN);
  lcd_init(SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN, LCD_CS_PIN);
  can_init(CAN_RX_PIN,CAN_TX_PIN,CAN_BAUDRATE);

  const BaseType_t task_created =
    xTaskCreate(control_task, "control", CONTROL_TASK_STACK_DEPTH, nullptr,
                CONTROL_TASK_PRIORITY, nullptr);
  const BaseType_t state_task_created =
    xTaskCreate(state_task, "state", STATE_TASK_STACK_DEPTH, nullptr,
                STATE_TASK_PRIORITY, nullptr);
  const BaseType_t bms_task_created =
    xTaskCreate(bms_task, "bms", BMS_TASK_STACK_DEPTH, nullptr,
                BMS_TASK_PRIORITY, nullptr);
  const BaseType_t lcd_task_created =
    xTaskCreate(lcd_task, "lcd", LCD_TASK_STACK_DEPTH, nullptr,
                LCD_TASK_PRIORITY, nullptr);

  // If task creation fails, remain here with all high-voltage relays open.
  if(task_created != pdPASS || state_task_created != pdPASS ||
     bms_task_created != pdPASS || lcd_task_created != pdPASS) {
    open_all_relays();
    for(;;) {}
  }

  vTaskStartScheduler();

  // The scheduler should never return. Staying here is the safest fallback.
  for(;;) {}
}

void loop(void) {}
