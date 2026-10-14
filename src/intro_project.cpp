#include <arduino_freertos.h>
#include <stdint.h>

#define WS_FL_PIN ???
#define WS_FR_PIN ???
#define WS_RL_PIN ???
#define WS_RR_PIN ???

#define STEERING_CAN_ID ???

#define MOTOR_FL_ID ???
#define MOTOR_FR_ID ???
#define MOTOR_RL_ID ???
#define MOTOR_RR_ID ???

#define NUM_CELLS ???

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

NOTE - EXTREMELY IMPORTANT: the CAN methods can_send and can_receive are NOT
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

volatile unsigned long timestamps[4];
volatile float rpm[4];

SemaphoreHandle_t spi_mutex;
SemaphoreHandle_t can_mutex;

enum CarState {LV, TS, RTD};
CarState car_state = LV;

float current = 0;
float steering_angle = 0;
float torques[4] = {0, 0, 0, 0};

void wheelspeed_fl_isr() {
  unsigned long now = micros();
  unsigned long gap = now - timestamps[0];
  rpm[0] = 60000000.0 / (gap * 17);
  timestamps[0] = now;
}

void wheelspeed_fr_isr() {
  unsigned long now = micros();
  unsigned long gap = now - timestamps[1];
  rpm[1] = 60000000.0 / (gap * 17);
  timestamps[1] = now;
}

void wheelspeed_rl_isr() {
  unsigned long now = micros();
  unsigned long gap = now - timestamps[2];
  rpm[2] = 60000000.0 / (gap * 17);
  timestamps[2] = now;
}

void wheelspeed_rr_isr() {
  unsigned long now = micros();
  unsigned long gap = now - timestamps[3];
  rpm[3] = 60000000.0 / (gap * 17);
  timestamps[3] = now;
}

void torque_task(void *parameters) {
  while (true) {
    current = analogRead(19) / 10.0; // convert from raw to voltage, then /10     raw / 495 * 3.3 / 10
    uint64_t steering_raw;
    xSemaphoreTake(can_mutex, portMAX_DELAY);
    can_receive(&steering_raw, STEERING_CAN_ID);
    xSemaphoreGive(can_mutex);
    steering_angle = (float)steering_raw;

    float wheelspeeds[4] = {rpm[0], rpm[1], rpm[2], rpm[3]};

    if (car_state == RTD) {
      calculate_torque_cmd(torques, current, wheelspeeds, steering_angle);
      xSemaphoreTake(can_mutex, portMAX_DELAY);
      can_send(MOTOR_FL_ID, (uint64_t)torques[0]);
      can_send(MOTOR_FR_ID, (uint64_t)torques[1]);
      can_send(MOTOR_RL_ID, (uint64_t)torques[2]);
      can_send(MOTOR_RR_ID, (uint64_t)torques[3]);
      xSemaphoreGive(can_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void lcd_task(void *parameters) {
  while (true) {
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    lcd_printf("Current:%f RPM:%f %f %f %f Steering:%f Torques:%f %f %f %f",
    current,
    rpm[0], rpm[1], rpm[2], rpm[3],
    steering_angle,
    torques[0], torques[1], torques[2], torques[3]);
    xSemaphoreGive(spi_mutex);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void bms_task(void *parameters) {
  while (true) {
    for (int i = 0; i < NUM_CELLS; i++) {
      xSemaphoreTake(spi_mutex, portMAX_DELAY);
      float voltage = bms_get_voltage(i);
      float temperature = bms_get_temperature(i);
      xSemaphoreGive(spi_mutex);
      if (voltage < 2.8 || voltage > 4.3 || temperature > 60) {
        digitalWrite(22, LOW);  // AIR+
        digitalWrite(10, LOW);  // AIR-
        digitalWrite(23, LOW);  // Precharge
        car_state = LV;
        break;
      }
    }
  }
}

void setup(void) {
  can_init(2, 3, ???);
  lcd_init(5, 6, 7, 8);
  bms_init(5, 6, 7, 9);

  pinMode(12, INPUT);
  pinMode(13, INPUT);
  pinMode(19, INPUT);
  pinMode(22, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(23, OUTPUT);

  digitalWrite(22, LOW);
  digitalWrite(10, LOW);
  digitalWrite(23, LOW);

  attachInterrupt(digitalPinToInterrupt(WS_FL_PIN), wheelspeed_fl_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(WS_FR_PIN), wheelspeed_fr_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(WS_RL_PIN), wheelspeed_rl_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(WS_RR_PIN), wheelspeed_rr_isr, FALLING);

  spi_mutex = xSemaphoreCreateMutex();
  can_mutex = xSemaphoreCreateMutex();

  xTaskCreate(torque_task, "torque", 1000, NULL, 1, NULL);
  xTaskCreate(lcd_task, "lcd", 1000, NULL, 1, NULL);
  xTaskCreate(bms_task, "bms", 1000, NULL, 1, NULL);
}

void loop(void) {
  if (car_state == LV && digitalRead(12) == LOW) {
    digitalWrite(10, HIGH);           // close AIR-
    digitalWrite(23, HIGH);           // close Precharge
    vTaskDelay(pdMS_TO_TICKS(5000)); // wait 5 seconds
    digitalWrite(22, HIGH);           // close AIR+
    digitalWrite(23, LOW);            // open Precharge
    car_state = TS;
  }
  if (car_state == TS && digitalRead(13) == LOW) {
    car_state = RTD;
  }
}
