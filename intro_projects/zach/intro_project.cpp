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
Planning:
States: LV -> PRECHARGE -> TS -> RTD.
TS pressed in PRECHARGE/TS/RTD -> LV (normal shutdown)
Any fault -> LV (latched)

Tasks (prioity high to low):
motor: 1ms sensors + latest steering -> torque (0 unless RTD) -> CAN
bms: 50ms check cells, trip_fault() if unsafe
steering: on message can_recieve (sleeps until message) -> save angle
state: 10ms buttons, state machine, relays
lcd: 100ms print everyting
ISRs: wheel teeth, timestamp only

Shared
CAN - no mutex, steering only caller of can_recieve, motor only calelr of
can_send SPI - bms and lcd (mutex) state - mutex data - mutex, copy then release

*/

#include <arduino_freertos.h>
#include <stdint.h>
#include <semphr.h>
#include <math.h>
#include <string.h>

// pins
#define TS_ON_PIN 12
#define RTD_PIN 13
#define CURRENT_SENSOR_PIN 19
#define CAN_RX_PIN 2
#define CAN_TX_PIN 3
#define MOSI_PIN 5
#define MISO_PIN 6
#define SCK_PIN 7
#define LCD_CS_PIN 8
#define BMS_CS_PIN 9
#define AIR_POS_PIN 22
#define PRECHARGE_PIN 23
#define AIR_NEG_PIN 10

#define WHEEL_FL_PIN 14
#define WHEEL_FR_PIN 15
#define WHEEL_RL_PIN 16
#define WHEEL_RR_PIN 17

// can
#define CAN_BAUDRATE 500000

#define CAN_ID_TORQUE_FL 0x10
#define CAN_ID_TORQUE_FR 0x11
#define CAN_ID_TORQUE_RL 0x12
#define CAN_ID_TORQUE_RR 0x13
#define CAN_ID_STEERING 0x20

const uint8_t TORQUE_IDS[4] = {
  CAN_ID_TORQUE_FL, CAN_ID_TORQUE_FR, CAN_ID_TORQUE_RL, CAN_ID_TORQUE_RR
};

// bms
#define CELL_COUNT 96
#define V_MIN 2.8f
#define V_MAX 4.3f
#define T_MAX 60.0f

#define PRECHARGE_TIME 5000
#define ADC_REF_V 3.3f
#define ADC_MAX 4095.0f
#define AMPS_PER_VOLT 100.0f
#define CURRENT_ZERO_V 0.0f

#define WHEEL_TEETH 17

/*
  Initialize the CAN peripheral with given RX and TX pins at a given
  baudrate.
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
  Calculates four torques, in order, for the Front Left, Front Right, Rear
  Left, and Rear Right motors given current, wheelspeeds (in the same
  order), and a steering angle.
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

enum State { LV, PRECHARGE, TS, RTD };
// everything LCD displays
struct CarData {
  float current;
  float steering;
  float wheelspeeds[4];
  float torques[4];
  float minV;
  float maxV;
  float maxT;
};
CarData carData = {}; // starts everything at 0

State state = LV;
bool fault = false;
bool bmsHealthy = false;

SemaphoreHandle_t dataMutex;
SemaphoreHandle_t stateMutex;
SemaphoreHandle_t spiMutex;

volatile uint32_t lastEdge[4] = {};   // when the last tooth passed
volatile uint32_t lastPeriod[4] = {}; // time between the last two teeth

void openRelays() {
  digitalWrite(AIR_POS_PIN, arduino::LOW);
  digitalWrite(PRECHARGE_PIN, arduino::LOW);
  digitalWrite(AIR_NEG_PIN, arduino::LOW);
}

void tripFault() {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  openRelays();
  fault = true;
  state = LV;
  xSemaphoreGive(stateMutex);
}

float readCurrent() {
  int raw = analogRead(CURRENT_SENSOR_PIN);
  float volts = raw * ADC_REF_V / ADC_MAX;
  return (volts - CURRENT_ZERO_V) * AMPS_PER_VOLT;
}

void wheelEdge(uint8_t i) {
  uint32_t now = micros();
  uint32_t elapsed = now - lastEdge[i];
  // after a long gap (wheel has stopped), this tooth starts fresh with period
  // unknown
  lastPeriod[i] = (elapsed > 500000) ? 0 : elapsed;
  lastEdge[i] = now;
}

void wheelFL() { wheelEdge(0); }
void wheelFR() { wheelEdge(1); }
void wheelRL() { wheelEdge(2); }
void wheelRR() { wheelEdge(3); }

void readRPM(float *rpm) {
  uint32_t period[4];
  uint32_t lastEdges[4];

  noInterrupts();
  for(uint8_t i = 0; i < 4; i++) {
    period[i] = lastPeriod[i];
    lastEdges[i] = lastEdge[i];
  }
  interrupts();

  uint32_t now = micros();
  for(uint8_t i = 0; i < 4; i++) {
    bool stopped = (now - lastEdges[i]) > 500000;
    if(stopped || period[i] == 0) {
      rpm[i] = 0.0f;
    } else {
      rpm[i] = 60000000.0f / (WHEEL_TEETH * period[i]);
    }
  }
}

// convert between torques/steering angles and CAN messages
uint64_t encodeFloat(float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return (uint64_t)bits;
}

float decodeFloat(uint64_t message) {
  uint32_t bits = (uint32_t)message;
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

void motorTask(void *) {
  TickType_t wake = xTaskGetTickCount();

  while(true) {
    // sensors
    float current = readCurrent();
    float rpm[4];
    readRPM(rpm);

    // latest steering
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    float steering = carData.steering;
    xSemaphoreGive(dataMutex);

    // decide + send
    float torques[4] = {0, 0, 0, 0};
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    if(state == RTD && !fault) {
      calculate_torque_cmd(torques, current, rpm, steering);
      for(uint8_t i = 0; i < 4; i++) {
        if(!isfinite(torques[i])) {
          torques[i] = 0; // never send garbage to a motor
        }
      }
    }
    for(uint8_t i = 0; i < 4; i++) {
      can_send(TORQUE_IDS[i], encodeFloat(torques[i]));
    }
    xSemaphoreGive(stateMutex);

    // share for lcd
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    carData.current = current;
    for(uint8_t i = 0; i < 4; i++) {
      carData.wheelspeeds[i] = rpm[i];
      carData.torques[i] = torques[i];
    }
    xSemaphoreGive(dataMutex);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}

void bmsTask(void *) {
  TickType_t wake = xTaskGetTickCount();
  while(true) {
    float minV = INFINITY;  // any real reading will be lower
    float maxV = -INFINITY; // any real reading will be higher
    float maxT = -INFINITY;
    bool unsafe = false;

    // reading, hold the SPI bus for the whole time
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    for(uint8_t i = 0; i < CELL_COUNT; i++) {
      float v = bms_get_voltage(i);
      float t = bms_get_temperature(i);

      // a broken reading will be treated as unsafe
      if(isnan(v) || isnan(t)) {
        unsafe = true;
      }
      if(v < minV) {
        minV = v;
      }
      if(v > maxV) {
        maxV = v;
      }
      if(t > maxT) {
        maxT = t;
      }
    }
    xSemaphoreGive(spiMutex);
    // check the extremes against the limits
    if(minV < V_MIN || maxV > V_MAX || maxT > T_MAX) {
      unsafe = true;
    }
    if(unsafe) {
      tripFault();
    } else {
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      bmsHealthy = true; // lets stateTask allow TS
      xSemaphoreGive(stateMutex);
    }
    // share to LCD
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    carData.minV = minV;
    carData.maxV = maxV;
    carData.maxT = maxT;
    xSemaphoreGive(dataMutex);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(50));
  }
}

void steeringTask(void *) {
  while(true) {
    uint64_t message = 0;
    can_receive(&message, CAN_ID_STEERING);
    float angle = decodeFloat(message);

    if(isfinite(angle)) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      carData.steering = angle;
      xSemaphoreGive(dataMutex);
    }
  }
}

// a button that reports each press exactly once, ignoring bounce and holding:
// holding: holding the button should only react once per press, not every 10ms.
// bounce: when a button is pressed, it may bounce between low and high before
// settling
struct Button {
  uint8_t pin;
  bool lastReading = false; // what we read last time (true = down)
  bool stableDown = false;  // the setteled state that we "trust"
};
// call every 10ms, returns true only on the cycle a new press settles
bool wasPressed(Button &button) {
  bool down = (digitalRead(button.pin) == arduino::LOW);
  bool pressed = false;
  // same reading twice in a row = bouncing has stopped and if settled value
  // differs from what we trusted, its a real change
  if(down == button.lastReading && down != button.stableDown) {
    button.stableDown = down;
    pressed = down;
  }
  button.lastReading = down;
  return pressed;
}

void stateTask(void *) {
  Button tsButton = {TS_ON_PIN};
  Button rtdButton = {RTD_PIN};
  TickType_t pcStart = 0;
  TickType_t wake = xTaskGetTickCount();

  while(true) {
    bool tsPressed = wasPressed(tsButton);
    bool rtdPressed = wasPressed(rtdButton);
    xSemaphoreTake(stateMutex, portMAX_DELAY);

    if(fault) {
    } // latched fault, ignore all buttons until reboot
    else if(state == LV) {
      // LV -> PRECHARGE
      if(tsPressed && bmsHealthy) {
        digitalWrite(AIR_NEG_PIN, arduino::HIGH);
        digitalWrite(PRECHARGE_PIN, arduino::HIGH);
        pcStart = xTaskGetTickCount();
        state = PRECHARGE;
      }
    } else if(tsPressed) {
      // PRECHARGE, TS, RTD + TS pressed -> normal shutdown (not a fault)
      openRelays();
      state = LV;
    } else if(state == PRECHARGE) {
      // PRECHARGE -> TS after 5s
      if(xTaskGetTickCount() - pcStart >= pdMS_TO_TICKS(PRECHARGE_TIME)) {
        digitalWrite(AIR_POS_PIN, arduino::HIGH);
        digitalWrite(PRECHARGE_PIN, arduino::LOW);
        state = TS;
      }
    } else if(state == TS) {
      // TS -> RTD
      if(rtdPressed) {
        state = RTD;
      }
    }
    xSemaphoreGive(stateMutex);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
  }
}

void lcdTask(void *) {
  TickType_t wake = xTaskGetTickCount();
  while(true) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(100));
  }
}

void setup(void) {
  digitalWrite(AIR_POS_PIN, arduino::LOW);
  digitalWrite(PRECHARGE_PIN, arduino::LOW);
  digitalWrite(AIR_NEG_PIN, arduino::LOW);
  pinMode(AIR_POS_PIN, arduino::OUTPUT);
  pinMode(PRECHARGE_PIN, arduino::OUTPUT);
  pinMode(AIR_NEG_PIN, arduino::OUTPUT);

  pinMode(TS_ON_PIN, arduino::INPUT_PULLUP);
  pinMode(RTD_PIN, arduino::INPUT_PULLUP);
  pinMode(CURRENT_SENSOR_PIN, arduino::INPUT);
  analogReadResolution(12);

  pinMode(WHEEL_FL_PIN, arduino::INPUT_PULLUP);
  pinMode(WHEEL_FR_PIN, arduino::INPUT_PULLUP);
  pinMode(WHEEL_RL_PIN, arduino::INPUT_PULLUP);
  pinMode(WHEEL_RR_PIN, arduino::INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(WHEEL_FL_PIN), wheelFL, arduino::FALLING
  );
  attachInterrupt(
    digitalPinToInterrupt(WHEEL_FR_PIN), wheelFR, arduino::FALLING
  );
  attachInterrupt(
    digitalPinToInterrupt(WHEEL_RL_PIN), wheelRL, arduino::FALLING
  );
  attachInterrupt(
    digitalPinToInterrupt(WHEEL_RR_PIN), wheelRR, arduino::FALLING
  );

  dataMutex = xSemaphoreCreateMutex();
  stateMutex = xSemaphoreCreateMutex();
  spiMutex = xSemaphoreCreateMutex();
  if(dataMutex == NULL || stateMutex == NULL || spiMutex == NULL) {
    openRelays();
    while(true) {}
  }

  can_init(CAN_RX_PIN, CAN_TX_PIN, CAN_BAUDRATE);
  lcd_init(MOSI_PIN, MISO_PIN, SCK_PIN, LCD_CS_PIN);
  bms_init(MOSI_PIN, MISO_PIN, SCK_PIN, BMS_CS_PIN);

  BaseType_t ok = pdPASS;
  ok &= xTaskCreate(motorTask, "motor", 512, NULL, 5, NULL);
  ok &= xTaskCreate(bmsTask, "bms", 512, NULL, 4, NULL);
  ok &= xTaskCreate(steeringTask, "steering", 512, NULL, 3, NULL);
  ok &= xTaskCreate(stateTask, "state", 512, NULL, 2, NULL);
  ok &= xTaskCreate(lcdTask, "lcd", 1024, NULL, 1, NULL);
  if(ok != pdPASS) {
    openRelays();
    while(true) {}
  }

  vTaskStartScheduler();
}

void loop(void) {}
