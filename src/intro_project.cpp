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
  /*Describe the process by which a multithreaded program gets executed:
  1. Setup acts as a "main" function. In other words, void setup acts as the entry point into the program. Next, within the setup function, tasks can be setup. You can treat a task at a basic level as a forever loop.
  2. Tasks begin to get executed.   
  */

  /*
  What's an ISR?
  An ISR is an interruprt Service Routine. It's used to handle timer oerflows, pin changes and to send messsages. 
  */

  /*
  What's time slicing?
  Time slicing is where a processor interupts programs at regular intervals, switching between tasks to give the illusion that they're being executed.
  The "slices" of time the FreeRTOS creates are called time slices, and typically a time slice is 1ms, or one tick.
  
  Descirbe how an OS, Task A with priority 0 and task B with priority 1 work together:
  1. First the Schduler is ran by the OS.
  2. Next the schduler runs say Task A as there are no other tasks besides it. 
  3. Task A runs until its allocated time slice runs out.
  4. The Tick Timer interrupts Task A allowing the OS to run the schduler once more.
  5. The schduler descides what to run. If Task A is still on its own, Task A is ran.
  6. Task A runs and completes and calls the vTaskDelay function for two ticks and thus does not run for two ticks. After vTaskDelay is ran, TaskA is said to be in the blocked state.
  //Important: Even if your program finishes at say 1.8ms, at 2.0ms, vTaskDelay counts a tick even though only 0.2s elapsed. Therefore, instead of TaskA unblocking at 4.0s, it unblocks at 3.0s.
  To better unstand this, think about this in terms of the schduler. The scheduler counts ticks. Once a new tick tick begins, it first increments the tick, it sees that the number of ticks is 2, and so unblocks/frees taskA. What this means is that by 3.0s mark, task 3 can run, right after it updates the state of the tasks, it decides which tasks to run.  
  7. By the third time slice, the schduler once again runs, but as there are no tasks, the system idles.
  */

  /*
  What if two tasks are unblocked but have equal priority? What occurs?
  In a case where two tasks of equal priority are waiting to be executed, a round robin occurs. A round robbin is where the schduler alternates between each task between interuprts/time slices.
  As an example, if Task A with priority 1 runs during time slice 1, by the next interuprt, beginning time slice 2, the schduler will excute Task B with priority B, even if A is not finished executing.

  What's preemtive schduling?
  Pre-emptive schduling is where CPU time is taken away from lower priority tasks to run higher priority tasks.
  */

  /*
  What tasks have the highest priority?
  Hardware interupts always take priority over running software unless hardware interupts are disabled.
  The only case where a hardware interupt is pre-empted is if another hardware interupt pre-empts it. That case is called a nested interupt.
  */

  /*
  What's an example of a hardware interupt?
  The tick timer is an example of a hardware interupt.
  */

  /*
  What happens if a hardware interupt occurs while a task is running?
  If a hardware interupt occurs while a task is running, as soon as the interupt resolves, the task returns to a state of output. 
  */

  /*
  What are the states available to a task?
    -Ready State (In this state, a task is waiting to get executed)
    -Run State (If a task is selected to be ran by the schduler, it enters the run state)
    -Blocked State (When running, a task can call an API function like vTaskDelay to enter the blocked state) (Tasks in this state cannot be ran until a condition is met that unblocks them)
    -Suspended (FreeRTOS has another api called vTaskSuspend. This function forces the task into the suspended state. When in this state, vTaskREsume must be called to return the task back to the ready state) (This is a good way to put a task to sleep if you don't want to work with a timer)
  */

  /*
  What's context switching?
  To preface, when switching between tasks, the schduler has the responsibility of remembering task related data and retreiing it. All this information is stored as something called a context. Saving and restoring context is the essence of context switching. 
  */

  /*
  Describe how context switching works in freeRTOS:
  1. ISR (Hardware Interuprt) Takes the CPU out of executing taskA. CPU begins to execute the code defined by the hardware interupt.
  2. Within the hardware interupt, the hardware interupt is programmed to tell the computer to save all data on the registers to the stack of taskA (Remember, Tasks have their own unique allocations within RAM).
  3. TaskA's pointer now points to the top of its stack.
  4. vTaskSwitch runs - it looks for what tasks are available to run. 
  5. portRestoreContext runs - it loads the pointer of stackB. It pops all of TaskB's saved registers into the CPU
  6. A "return" instruction is ran and TaskB begins to run once more. 
  */

  /*
  During context switching, explain what information is stored in the Stack and how it's stored:
  1. First the PC(A) - the program counter - is added to the stack. The Program counter keeps track of what line of code the CPU was last executing. This is pushed on the stack by a hardwar interupt.
  2. Next, all the data of TaskA stored on registers are added to the stack by portSaveContext() (Note: Information that could be in the CPU registeries include local variables - mind you inactive local variables just sit on the stack. )
  3. Kernel makes a copy of the Task pointer - the one that points to the top of the stack of say TaskA.

  */

  /*
  Where is the pointer for a task stored - the "copy" created by the kernel?
  All information associated with a task is stored in a TCB (Task Control Block). A TCB is created whenever a taks is created.
  The first member of a task TCB is the pointer. 
  */


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
