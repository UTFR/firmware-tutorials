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

A small board-support layer (bsp.h / bsp.c) making real STM32G4 HAL calls has
been provided -- the peripherals are real hardware calls now (real GPIO, ADC,
SPI, and CAN/FDCAN transactions), but the concurrency architecture is still
entirely your job: which tasks you create, which mutexes/queues you use, and
how you protect the shared SPI1 bus and the CAN-received steering angle.

NOTE - EXTREMELY IMPORTANT: bsp_lcd_printf(), bsp_bms_get_voltage(), and
bsp_bms_get_temperature() all drive the same physical SPI1 bus and are NOT
synchronized against each other or against anything else touching that bus.

Also, g_last_steering_angle_deg (declared in bsp.h) is written from CAN
receive-interrupt context and is NOT synchronized with whoever reads it.

Also think about the implications of having two peripherals on one SPI bus.

----------------------------

Requirements:

1. Command each motor with an appropriate torque every 1ms
2. Print sensor values and torque values to the LCD every 100ms
3. Monitor the highest/lowest voltage and highest temperature from the BMS and
shutdown the car if there is any unsafe condition

----------------------------

Pins (see controllers/intro_project/src/main.c's pins[] table and bsp.c for the
peripheral pins -- GPIO ports/pins below, STM32G474RET6):

TS On:              PB4
RTD:                PB5
Current Sensor:     PA0  (ADC1_IN1)
CAN RX / TX:        PA11 / PA12 (FDCAN1)
Debug UART RX / TX: PA3  / PA2  (USART2)
SPI1 SCK/MISO/MOSI: PA5  / PA6  / PA7  (shared LCD + BMS bus)
LCD_CS:             PB0
BMS_CS:             PB1
AIR+:               PB6
AIR-:               PB7
Precharge:          PB8
Wheelspeed FL/FR/RL/RR: PC6 / PC7 / PC8 / PC9

----------------------------

"Submission" instructions. I want you all to get familiar with git, so we will
do this project with git. The repo is
https://github.com/UTFR/firmware-tutorials.

If you still haven't joined the github organization, let me know and i'll add
you. Clone the repository, and make a branch called `<your name>/intro_project`.
Copy this file (and controllers/intro_project/include/bsp.h) into the directory
`intro_projects/<your name>` and make all your changes there.

Whenever you add a feature, `git add` the files, and `git commit -m "..."` with
a useful/descriptive message. Whenver you want your changes to be public, do
`git push origin <your name>/intro_project`.

Also, we have a code formatter (clang-format) set up in this repo, matching
what the real firmware repo uses. It means that everyone's code will look
exactly the same, in terms of how much whitespace there is and other aesthetics
like that. It's not there for aesthetics, but moreso so that you when people
make changes you can see exactly what they changed, whereas without a formatter,
you end up with lots of useless formatting changes. Run `clang-format -i` on
your files before committing (or let CI's `format` job tell you if you forgot).
*/

#include "bsp.h"

void app_main(void) {
  // TODO: your concurrency architecture goes here -- tasks, mutexes, queues,
  // and the LV -> TS -> RTD state machine described above.
}
