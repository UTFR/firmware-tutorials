#include <Arduino_FreeRTOS.h>
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

#define TS_ON 12
#define RTD 13
#define CURRENT 19
#define CAN_RX 2
#define CAN_TX 3
#define MOSI 5
#define MISO 6
#define SCK 7
#define LCD_CS 8
#define BMS_CS 9
#define AIRPLUS 22
#define PRECHARGE 23
#define AIRMIN 10

#define WHEELSPEED_FL 14
#define WHEELSPEED_FR 15
#define WHEELSPEED_RL 16
#define WHEELSPEED_RR 17

#define WHEELSPEED_WINDOW_MS 20

volatile unsigned long wheelspeedFLCount = 0, wheelspeedFRCount = 0, wheelspeedRLCount = 0, wheelspeedRRCount = 0;

#define BAUDRATE 50000 // idk what this is actually supposed to be um

#define STEERING_ANGLE_CAN_ID 1 // this is also definitely not right

SemaphoreHandle_t canMutex;
SemaphoreHandle_t spiMutex;

#define ADC_MAX_COUNTS 4095.0 // teensy 4.1 uses 12 bit resolution, 2^12 - 1
#define ADC_REF_VOLTAGE 3.3
#define TEETH_PER_REV 17

unsigned long prevFLCount = 0, prevFRCount = 0, prevRLCount = 0, prevRRCount = 0;
unsigned long prevWheelSpeedTime = 0;

SemaphoreHandle_t sensorDataMutex;
float sharedTorques[4];
float sharedWheelSpeeds[4];
float sharedCurrent;

// placeholder values idk what the real ones are :(
#define TORQUE_FL_CAN_ID 0x10
#define TORQUE_FR_CAN_ID 0x11
#define TORQUE_RL_CAN_ID 0x12
#define TORQUE_RR_CAN_ID 0x13

#define NUM_CELLS 5
#define V_MIN 2.8f
#define V_MAX 4.3f
#define TEMP_MAX 60.0f

volatile bool faultLatched = false;

float sharedMinV;
float sharedMaxV;
float sharedMaxTemp;

#define BUTTON_PERIOD_MS 10
#define PRECHARGE_TIME_MS 5000
#define PRECHARGE_CHECK_MS 50

typedef enum { STATE_LV, STATE_TS, STATE_RTD } car_state_t; // enum is like struct but the states are choices
volatile car_state_t carState = STATE_LV; // start at lv

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

static uint64_t packFloat(float f){
  uint64_t payload = 0;
  memcpy(&payload, &f, sizeof(float));
  return payload;
}

void torqueTask(void *pvParameters){
  static float wheelspeeds[4] = {0, 0, 0, 0};

  while (true) {
    int adcReading = analogRead(CURRENT);
    float voltage = (adcReading / ADC_MAX_COUNTS) * ADC_REF_VOLTAGE;
    float current = voltage * 100; // multiply by 1000 (mV), divide by 10 (mV per amp)

    unsigned long now = millis();
    if (now - prevWheelSpeedTime >= WHEELSPEED_WINDOW_MS){
      float dt_min = (now - prevWheelSpeedTime) / 60000.0; // time in minutes

      float wheelspeeds[4];

      wheelspeeds[0] = ((wheelspeedFLCount - prevFLCount) / (float)TEETH_PER_REV) / dt_min;
      wheelspeeds[1] = ((wheelspeedFRCount - prevFRCount) / (float)TEETH_PER_REV) / dt_min;
      wheelspeeds[2] = ((wheelspeedRLCount - prevRLCount) / (float)TEETH_PER_REV) / dt_min;
      wheelspeeds[3] = ((wheelspeedRRCount - prevRRCount) / (float)TEETH_PER_REV) / dt_min;

      prevFLCount = wheelspeedFLCount;
      prevFRCount = wheelspeedFRCount;
      prevRLCount = wheelspeedRLCount;
      prevRRCount = wheelspeedRRCount;
      prevWheelSpeedTime = now;

    }

    uint64_t steeringPayload;

    xSemaphoreTake(canMutex, portMAX_DELAY); // use mutex to avoid "not thread safe" issue
    can_receive(&steeringPayload, STEERING_ANGLE_CAN_ID);
    xSemaphoreGive(canMutex); // sandwich can_receive with same mutex, prevent crashes

    float steeringAngle = 0.0; // not right

    float torques[4];
    calculate_torque_cmd(torques, current, wheelspeeds, steeringAngle);

    if (carState != STATE_RTD){
      for (int i = 0; i < 4; i++){
        torques[i] = 0.0f;
      }
    }

    xSemaphoreTake(canMutex, portMAX_DELAY); // same thing
    can_send(TORQUE_FL_CAN_ID, 0);
    can_send(TORQUE_FR_CAN_ID, 0);
    can_send(TORQUE_RL_CAN_ID, 0);
    can_send(TORQUE_RR_CAN_ID, 0);
    xSemaphoreGive(canMutex);

    xSemaphoreTake(sensorDataMutex, portMAX_DELAY);
    for (int i = 0; i < 4; i++){
      sharedTorques[i] = torques[i];
      sharedWheelSpeeds[i] = wheelspeeds[i];
    }
    sharedCurrent = current;
    xSemaphoreGive(sensorDataMutex);

    vTaskDelay(1 / portTICK_PERIOD_MS); // convert time in ms to rtos ticks
  }
}

void lcdTask(void *pvParameters){
  while (true){
    float current, wheelspeeds[4], torques[4];
    float minV, maxV, maxTemp;

    xSemaphoreTake(sensorDataMutex, portMAX_DELAY);
    current = sharedCurrent;
    for (int i = 0; i < 4; i++){
      wheelspeeds[i] = sharedWheelSpeeds[i];
      torques[i] = sharedTorques[i];
    }
    minV = sharedMinV; // need to write these copies otherwise innaccessable outside of datamutex
    maxV = sharedMaxV;
    maxTemp = sharedMaxTemp;
    xSemaphoreGive(sensorDataMutex);

    car_state_t state = carState;

    xSemaphoreTake(spiMutex, portMAX_DELAY);
    lcd_printf("State: %d  Current: %.1f A\n", state, current);
    lcd_printf("RPM: %.0f %.0f %.0f %.0f\n",
                wheelspeeds[0], wheelspeeds[1], wheelspeeds[2], wheelspeeds[3]);
    lcd_printf("Torque: %.1f %.1f %.1f %.1f\n",
                torques[0], torques[1], torques[2], torques[3]);
    lcd_printf("V: %.2f-%.2f  T: %.1f\n", minV, maxV, maxTemp);
    xSemaphoreGive(spiMutex);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

static bool buttonPressed(uint8_t pin){
  return digitalRead(pin) == LOW;
}

static void openRelays(void){
  digitalWrite(AIRPLUS, LOW);
  digitalWrite(AIRMIN, LOW);
  digitalWrite(PRECHARGE, LOW);
}

void shutdown(void){
  openRelays();
  faultLatched = true;
}

static bool runPrecharge(void){
  digitalWrite(AIRMIN, HIGH);
  digitalWrite(PRECHARGE, HIGH);

  for (int elapsed = 0; elapsed < PRECHARGE_TIME_MS; elapsed += PRECHARGE_CHECK_MS){
    if (faultLatched){
      openRelays();
      return false;
    }

    vTaskDelay(PRECHARGE_CHECK_MS / portTICK_PERIOD_MS);
  }

  if (faultLatched){
    openRelays();
    return false;
  }

  digitalWrite(AIRPLUS, HIGH);
  digitalWrite(PRECHARGE, LOW);
  return true;

}

void stateMachineTask(void* pvParameters){
  while (true) {
    if (faultLatched){
      openRelays();
      carState = STATE_LV;
    } else {
      switch (carState) {
        case STATE_LV:
          if (buttonPressed(TS_ON)) {
            carState = runPrecharge() ? STATE_TS : STATE_LV;
          }
          break;
        
        case STATE_TS:
          if (buttonPressed(RTD)) { 
            carState = STATE_RTD;
          }
          break;
        
        case STATE_RTD:
          break;

      }
    }

    vTaskDelay(BUTTON_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

void bmsTask(void *pvParameters){
  while (true) {
    float minV = 1000.0f, maxV = -1000.0f, maxTemp = -1000.0f;

    for (uint8_t n = 0; n < NUM_CELLS; n++){
      xSemaphoreTake(spiMutex, portMAX_DELAY);
      float v = bms_get_voltage(n);
      float t = bms_get_temperature(n);
      xSemaphoreGive(spiMutex);
      
      if (v < minV) minV = v;
      if (v > maxV) maxV = v;
      if (t > maxTemp) maxTemp = t;

      if (minV < V_MIN || maxV > V_MAX || maxTemp > TEMP_MAX){
        shutdown();
        break;
      }

      sharedMinV = minV, sharedMaxV = maxV, sharedMaxTemp = maxTemp;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

}

void wheelspeedFL_ISR(void){
  wheelspeedFLCount++;
}

void wheelspeedFR_ISR(void){
  wheelspeedFRCount++;
}

void wheelspeedBL_ISR(void){
  wheelspeedRLCount++;
}

void wheelspeedBR_ISR(void){
  wheelspeedRRCount++;
}

void setup(void) {
  pinMode(TS_ON, INPUT_PULLUP);
  pinMode(RTD, INPUT_PULLUP);
  pinMode(CURRENT, INPUT);

  pinMode(AIRPLUS, OUTPUT);
  digitalWrite(AIRPLUS, LOW);
  pinMode(AIRMIN, OUTPUT);
  digitalWrite(AIRMIN, LOW);
  pinMode(PRECHARGE, OUTPUT);
  digitalWrite(PRECHARGE, LOW);

  can_init(CAN_RX, CAN_TX, BAUDRATE);
  lcd_init(MOSI, MISO, SCK, LCD_CS);
  bms_init(MOSI, MISO, SCK, BMS_CS);

  canMutex = xSemaphoreCreateMutex();
  spiMutex = xSemaphoreCreateMutex();
  sensorDataMutex = xSemaphoreCreateMutex();

  pinMode(WHEELSPEED_FL, INPUT_PULLUP);
  pinMode(WHEELSPEED_FR, INPUT_PULLUP);
  pinMode(WHEELSPEED_RL, INPUT_PULLUP);
  pinMode(WHEELSPEED_RR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(WHEELSPEED_FL), wheelspeedFL_ISR, FALLING); //pin from high to low
  attachInterrupt(digitalPinToInterrupt(WHEELSPEED_FR), wheelspeedFR_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(WHEELSPEED_RL), wheelspeedBL_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(WHEELSPEED_RR), wheelspeedBR_ISR, FALLING);

  xTaskCreate(torqueTask, "Torque", 256, NULL, 4, NULL);
  xTaskCreate(lcdTask, "LCD", 256, NULL, 1, NULL);
  xTaskCreate(bmsTask, "BMS", 256, NULL, 3, NULL);
  xTaskCreate(stateMachineTask, "State", 256, NULL, 2, NULL);

}

void loop(void) {}
