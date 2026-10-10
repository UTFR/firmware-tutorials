#include <arduino_freertos.h>
#include <stdint.h>

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
Start with assigning pins. Remember constants are defined with #define.
They come with the limitation that need parentheses. 
For instance, given #define a = 5+4. 
#define b = a * 2 does not yield 18 as you might expect
but instead 5+4*2 or 13. 
*/

/*
  Initialize the CAN peripheral with given RX and TX pins at a given baudrate.
*/

#define TS_ON = 12;
#define RTD = 13;
#define CURRENT_SENSOR = 19;

#define CAN_RX = 2;
#define CAN_TX = 3;
#define MOSI = 5;

#define MISO = 6;
#define SCK: 7;
#define LCD_CS = 8;

#define BMS_CS: = 9;
#define AIR_P = 22;
#define Precharge = 23;

#define AIR_N = 10;


/*
  Important Terminology (I2C):
  MOSI - Master Out Slave In
  MISO - Master In Slave Out
  SCK - Serial Clock
  Ics_cs - Chip Select/Slave Slect
 */

/*Other important constants*/ 
#define BD_RATE = 1000000;
#define SPEED = 90;
#define MAX_CELL_COUNT = 1000;


/*Motor constant*/
#define M = 1;


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

/*Create a function for checking the batteries*/
boolean bms_check() {
/*For thread safety*/
  Semaphore

  #define BMS_MAX_V = 28;
  #define BMS_MIN_V = 10;
  #define BMS_MAX_TEMP = 41;
  #define BMS_MIN_TEMP = 11;

  float temp;
  float voltage;

  /*Basics of freeRTOS strucutre.*/
  /*Example of a task in RTOS: void toggleLED(void *parameter) {}*/
  /*Regarding timing: Remember, in RTOS, if you delay a task, RTOS will execute another task until the the delay time is over.*/
  /*Tick Timers: Almost all RTOS' are based off a tick timer. A tick timer is simply a hardware timer allocated to interupt a hardware process over an interval. By default, freeRTOS sets one tick timer to one ms, and portTICK_PERIOD_MS to one.*/
  /*Remember, the freeRTOS function vTaskDelay expects as input # of tick delay, not # of MS. This isn't too bad as by default a tick is simply 1ms.*/

  /*
  What do you need before running any task?
  Before running any task, you must call the vTaskStartSchduler() in main after setting up your tasks. Only then do tasks begin to excute.
  */

  /*How do I create a task in RTOS?*/
  /*You create a task in RTOS with:
  xTaskCreate(
      toggleLED, //Function to be called in a task
      "Toggle LED", //Name of the task'
      1024, //Stack size (# of words in FreeRTOS)
      Null, //This is the next perameter. Parameter to pass to function.
      1, //This is the priority of a task. By default, in FreeRTOS, you can set priority from 0 to 24. Therefore, you have 25 different priority levels (0 to configMAX_PRIORITIES - 1).
      Null, //This is the task handle, you can assign a pointer to watch over a task. Basically, the handle is primraily used by other tasks to affect the state of another task. 

  ).*/

  /*Note a word in a 32bit system is 4bytes, while a word in a 64 bit processor is 8 bytes. In essence, a word is the maximum amoutn of data a CPU can adress in one operation.*/
  /*Where are tasks made in freeRTOS? In freeRTOS, tasks are created in void setup. Ignore void loop, it has no purpose.*/

  /*Understanding the schduler*/


  /*BMS Safety Checks*/
  for (int i = 0; i < MAX_CELL_COUNT; < i++) {

    /*retreive data*/
    b
    temp = bms_get_temperature(i);
    voltage = bms_get_voltage(i);

    /*Case1: Car fails battery safety checks.*/
    if !((BMS_MIN_TEMP < temp < BMS_MAX_TEMP) && (BMS_MIN_V < voltage < BMS_MAX_TEMP)) {
      return false;
    };

    /*Case2: Car passed safety inspections*/
    return true;
  };
}

/*This program runs once. */
void setup(void) {
  /*Important initializations*/
  bms_init(MOSI, MISO, SCK, BMS_CS);
  can_init(CAN_RX, CAN_TX, BD_RATE);
  lcd_init(MOSI, MISO, SCK, LCD_CS); /*We must chip select the LCD?*/





}

void loop(void) {
  can_send(M, SPEED);

}
