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

#include <arduino_freertos.h>
#include <stdint.h>

// GPIO pins
constexpr uint8_t TS_ON_PIN = 12;
constexpr uint8_t RTD_PIN = 13;
constexpr uint8_t CURRENT_SENSOR_PIN = 19;

constexpr uint8_t CAN_RX_PIN = 2;
constexpr uint8_t CAN_TX_PIN = 3;

constexpr uint8_t MOSI_PIN = 5;
constexpr uint8_t MISO_PIN = 6;
constexpr uint8_t SCK_PIN = 7;

constexpr uint8_t LCD_CS_PIN = 8;
constexpr uint8_t BMS_CS_PIN = 9;

constexpr uint8_t AIR_PLUS_PIN = 22;
constexpr uint8_t PRECHARGE_PIN = 23;
constexpr uint8_t AIR_MINUS_PIN = 10;

//Assumed
constexpr uint8_t MAX_CELL_COUNT = 100;

constexpr float BMS_MAX_V = 4.3f;
constexpr float BMS_MIN_V = 2.8f;
constexpr float BMS_MAX_TEMP = 60.0f;

constexpr uint32_t CAN_BAUD_RATE = 1000000;

BmsTelementry struct 
{
  float voltage;
  float temperature;
  //Time stamp - not that essential.
};


//Note: In freeRTOS sempahores and mutexes are defined very similarly as they're similar. 
/*State Queue Handle*/
static QueueHandle_t bms_queue, torque_queue;
static const uint8_t bms_queue_len = 5;


/*States*/
#define LV = 0; //De-energized
#define TS = 1; //Energized
#define RTD = 2; //Ready to drive

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
  float *torques, float current, float wheelspeeds, float steering_angle
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

void Motor(void *pvParameters) {
  uint64_t current steering;
  uint64_t speed[4];
  float torque[4];
  TickType_t motor_beat = 0;

  for (;;) {

  }

  //Analyze current and motor speeds & steering
  current = can_receive(0x04) / 10;

  can_receive(&speed[1], 0x00); //& provides the adress of where I want the speed information to go.
  can_receive(&speed[2], 0x01);
  can_receive(&speed[3], 0x02);
  can_receive(&speed[4], 0x03);

  steering = can_receive(0x05);

  calculate_torque_cmd(&torque, speed, current, steering)

  //Send Motor Commands
  can_send(0x00)
  can_send(0x01)
  can_send(0x02)
  can_send(0x03)



  vTaskDelayUntil(motor_beat, pdMS_TO_TICKS(1)); //We tick relative to the last tick - this ensures that the motor runs in 1ms intervals. A standard vTaskDelay would be baesd on time slices.
}

//Should warn the manage state task.
void BMS(void *pvParameters) {
  //Iterate through the cells and get temperature and voltage
  float voltage, highest_voltage, temperature, highest_temperature;

  //Iterate through cells
  for (int c; c < MAX_CELL_COUNT - c; i++) {

    voltage = bms_get_voltage(c);
    temperature = bms_get_temperature(c);

    if (BMS_MIN_V < voltage < BMS_MAX_V) {
      //A FAULT IS DETECTED -> DE-ENERGIZE
      semaphoreGive(BMS_FAULT)
    };

    if (temperature > BMS_MAX_TEMP) {
      //A FAULT IS DETECTED -> DE-ENERGIZE
      semaphoreGive(BMS_FAULT)
    };
    
    //Identify highest and lowest temperature
    if (voltage > highest_voltage) highest_voltage = voltage;
    if (temperature > highest_temperature) highest_temperature = temperature;
    
    //Output information to a queue
    xQueueSend({&highest_voltage, &highest_temperature}, ); //In progress - we must share information with the display task. We probably don't want a long delay. 

  };
};

void Display(void *pvParameters) {
  xQueueReceive() 
}

//What this thing is gonna do:
//Have a semaphore and wake up whenever the BMS cries
//Read user inputs from the steering wheel - probably have this in a queue

void States(void *pvParameters) { //This should be a high priority task - but it doesn't need to run constantly
  //We need to check for inputs to determine what state the CAR is in.
  //We're going use some sort of CAN to communicate between the managerBMS and the manageState.


  for (;;) {

  }
}


/*This program runs once. */
void setup(void) {
  //Initializations
  bms_init(MOSI, MISO, SCK, BMS_CS);
  can_init(CAN_RX, CAN_TX, BD_RATE);
  lcd_init(MOSI, MISO, SCK, LCD_CS); 

  
}

void loop(void) {
  can_send(M, SPEED);

}
