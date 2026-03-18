#include <arduino_freertos.h>
#include <queue.h>
#include <semphr.h>
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

constexpr int TS_ON_PIN = 12;
constexpr int RTD_PIN = 13;
constexpr int CURRENT_SENSOR_PIN = 19;
constexpr int CAN_RX_PIN = 2;
constexpr int CAN_TX_PIN = 3;
constexpr int MOSI_PIN = 5;
constexpr int MISO_PIN = 6;
constexpr int SCK_PIN = 7;
constexpr int LCD_CS_PIN = 8;
constexpr int BMS_CS_PIN = 9;
constexpr int AIR_PLUS_PIN = 22;
constexpr int PRECHARGE_PIN = 23;
constexpr int AIR_MINUS_PIN = 10;
constexpr int WHEELSPEED_PIN = 11;  // 11 ??

constexpr int BATTERIES_COUNT = 138;

QueueHandle_t current_queue;

volatile unsigned long wheel_speed_interval = 0;
volatile unsigned long wheel_speed_last_pulse = 0;

QueueHandle_t steering_angle_queue;

volatile float bms_voltage_min = 0.0f;
volatile float bms_voltage_max = 0.0f;
volatile float bms_temperature_max = 0.0f;
volatile bool bms_fault_active = false;
SemaphoreHandle_t bms_mutex;

QueueHandle_t torques_queue;

SemaphoreHandle_t can_mutex;
SemaphoreHandle_t spi_mutex;

constexpr int TaskPriority_LCD = 0;
constexpr int TaskPriority_SENSOR = 1;
constexpr int TaskPriority_BMS = 2;

// handling states for transitions (state machine)
enum class CarState { LV,
                      PRECHARGE,
                      TS,
                      RTD };
volatile CarState car_state = CarState::LV;

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
extern void can_receive(uint64_t* payload, uint8_t id);

/*
  Calculates four torques, in order, for the Front Left, Front Right, Rear Left,
  and Rear Right motors given current, wheelspeeds (in the same order), and a
  steering angle.
*/
extern void calculate_torque_cmd(float* torques, float current, float* wheelspeeds, float steering_angle);

/*
  Initialize the LCD peripheral
*/
extern void lcd_init(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t lcs_cs);

/*
  Print something to the LCD
*/
extern void lcd_printf(const char* fmt, ...);

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

float sensor_current_read() {
    static const float ADC_VREF_V = 3.3f;
    static const float ADC_RESOLUTION = 1024.0f;
    static const float ADC_CONVERSION = 0.01f;

    int raw = analogRead(CURRENT_SENSOR_PIN);

    float voltage_V = (raw * ADC_VREF_V) / ADC_RESOLUTION;
    float current_A = voltage_V / ADC_CONVERSION;

    return current_A;
}

void sensor_current_update(void* parameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1);

    while (1) {
        float new_current = sensor_current_read();
        xQueueOverwrite(current_queue, &new_current);

        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

float sensor_current_get(void) {
    float current = 0.0f;
    xQueueReceive(current_queue, &current, 0);

    return current;
}

void sensor_wheel_speed_update(void) {  // ISR called on each falling edge from the wheelspeed sensor
    unsigned long current = micros();
    wheel_speed_interval = current - wheel_speed_last_pulse;
    wheel_speed_last_pulse = current;
}

float sensor_wheel_speed_get(void) {
    // check for a timeout in case car stops moving. Use about 0.5s as the check
    unsigned long current_time = micros();

    if ((current_time - wheel_speed_last_pulse) > 500000)
        return 0.0f;

    // RPM is calculated as: (60 * wheel_pulse_freq) / (total pulses per revolution)
    if (wheel_speed_interval == 0) return 0.0f;                           // avoid divide by zero
    return (60.0f * 1000000.0f) / (17.0f * (float)wheel_speed_interval);  // mult to get right units
}

void sensor_steering_angle_update(void* parameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(10);

    while (1) {
        uint64_t steering_angle_raw;

        if (xSemaphoreTake(can_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            can_receive(&steering_angle_raw, 0x1);
            xSemaphoreGive(can_mutex);

            float steering_angle = (float)steering_angle_raw;
            xQueueOverwrite(steering_angle_queue, &steering_angle);
        }

        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

float sensor_steering_angle_get(void) {
    float steering_angle = 0.0f;
    xQueueReceive(steering_angle_queue, &steering_angle, 0);

    return steering_angle;
}

void torque_update(void* parameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1);

    while (1) {
        if (car_state != CarState::RTD) {  // don't drive unless right state. Just extra safe gaurding
            vTaskDelayUntil(&last_wake_time, frequency);
            continue;
        }
        float current = sensor_current_get();
        float wheel_speed = sensor_wheel_speed_get();
        float steering_angle = sensor_steering_angle_get();

        float torques[4];
        float wheelspeeds[4] = {wheel_speed, wheel_speed, wheel_speed, wheel_speed};  // adding cause calculate_torque_cmd expects an array of 4 items
        calculate_torque_cmd(torques, current, wheelspeeds, steering_angle);

        if (xSemaphoreTake(can_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            can_send(0x2, torques[0]);
            can_send(0x3, torques[1]);
            can_send(0x4, torques[2]);
            can_send(0x5, torques[3]);
            xSemaphoreGive(can_mutex);
        }

        xQueueOverwrite(torques_queue, torques);

        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

// helper to print car state on the lcd
const char* car_state_to_string(CarState state) {
    switch (state) {
        case CarState::LV:
            return "LV";
        case CarState::PRECHARGE:
            return "PRECHARGE";
        case CarState::TS:
            return "TS";
        case CarState::RTD:
            return "RTD";
        default:
            return "UNKNOWN";
    }
}

void lcd_update(void* parameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(100);

    while (1) {
        if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(100))) {
            float min_voltage = 0.0f;
            float max_voltage = 0.0f;
            float max_temperature = 0.0f;
            bool fault_active = false;

            // read bms values with mutex to prevent race conditions
            if (xSemaphoreTake(bms_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                min_voltage = bms_voltage_min;
                max_voltage = bms_voltage_max;
                max_temperature = bms_temperature_max;
                fault_active = bms_fault_active;
                xSemaphoreGive(bms_mutex);
            }

            float current = sensor_current_get();
            float wheel_speed = sensor_wheel_speed_get();
            float steering_angle = sensor_steering_angle_get();

            float torques[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            xQueueReceive(torques_queue, torques, 0);

            lcd_printf("state: %s\n", car_state_to_string(car_state));   // car state
            lcd_printf("fault: %s\n", fault_active ? "YES" : "NO");      // bms fault
            lcd_printf("current: %.3f A\n", current);                    // current
            lcd_printf("wheel speed: %.3f RPM\n", wheel_speed);          // rpm
            lcd_printf("steering angle: %.3f degrees", steering_angle);  // steering angle
            lcd_printf("bms minV: %.3f V\n", min_voltage);               // bms vals as computed in the bms task (min and max voltage and max temp)
            lcd_printf("bms maxV: %.3f V\n", max_voltage);
            lcd_printf("bms maxT: %.3f C\n", max_temperature);
            lcd_printf("torque\n\tFL: %.3f \tFR: %.3f\n\tRL: %.3f RR %.3f", torques[0], torques[1], torques[2], torques[3]);

            xSemaphoreGive(spi_mutex);
        }

        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

void deenergize(void) {
    digitalWrite(AIR_PLUS_PIN, arduino::LOW);
    digitalWrite(AIR_MINUS_PIN, arduino::LOW);
    digitalWrite(PRECHARGE_PIN, arduino::LOW);
    car_state = CarState::LV;  // go back to LV state
}

void bms_monitoring_task(void* parameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(50);

    while (1) {
        if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            bool fault_active = false;  // to prevent giving spi mutex twice
            float lowest_voltage = 10000.0f;
            float highest_voltage = 0.0f;
            float highest_temp = -10000.0f;

            for (int i = 0; i < BATTERIES_COUNT; i++) {
                float voltage = bms_get_voltage(i);
                float temperature = bms_get_temperature(i);

                // get the max/min values to print on lcd
                if (voltage < lowest_voltage)
                    lowest_voltage = voltage;
                if (voltage > highest_voltage)
                    highest_voltage = voltage;
                if (temperature > highest_temp)
                    highest_temp = temperature;

                if (voltage < 2.8f || voltage > 4.3f || temperature > 60.0f) {
                    fault_active = true;
                    break;
                }
            }
            xSemaphoreGive(spi_mutex);  // give the mutex first

            if (fault_active)
                deenergize();

            // modfiy the global variables using mutex cause LCD reads from them, this task writes to them, avoid race conditions
            if (xSemaphoreTake(bms_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                bms_voltage_min = lowest_voltage;
                bms_voltage_max = highest_voltage;
                bms_temperature_max = highest_temp;
                bms_fault_active = fault_active;
                xSemaphoreGive(bms_mutex);
            }
        }

        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

void state_machine_task(void* parameters) {
    // set all relay pins as outputs
    pinMode(AIR_MINUS_PIN, arduino::OUTPUT);
    pinMode(AIR_PLUS_PIN, arduino::OUTPUT);
    pinMode(PRECHARGE_PIN, arduino::OUTPUT);
    pinMode(TS_ON_PIN, arduino::INPUT_PULLUP);
    pinMode(RTD_PIN, arduino::INPUT_PULLUP);

    while (1) {
        if (car_state == CarState::LV) {
            if (digitalRead(TS_ON_PIN) == LOW) {  // active low = pressed
                car_state = CarState::PRECHARGE;  // PRECHARGE state just to track if smth inturrupts this sequence
                // precharge sequence
                digitalWrite(AIR_MINUS_PIN, arduino::HIGH);  // close AIR-
                digitalWrite(PRECHARGE_PIN, arduino::HIGH);  // close precharge relay
                vTaskDelay(pdMS_TO_TICKS(5000));             // wait 5 seconds
                if (car_state == CarState::PRECHARGE) {
                    digitalWrite(AIR_PLUS_PIN, arduino::HIGH);  // close AIR+
                    digitalWrite(PRECHARGE_PIN, arduino::LOW);  // open precharge relay
                    car_state = CarState::TS;                   // set state
                } else {
                    // sequence aborted due to deenergize(), undo changes preivously made
                    digitalWrite(AIR_MINUS_PIN, arduino::LOW);
                    digitalWrite(PRECHARGE_PIN, arduino::LOW);
                }
            }
        } else if (car_state == CarState::TS) {
            if (digitalRead(RTD_PIN) == LOW) {  // active low = pressed
                car_state = CarState::RTD;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // check buttons every 10ms
    }
}

void setup(void) {
    current_queue = xQueueCreate(1, sizeof(float));
    steering_angle_queue = xQueueCreate(1, sizeof(float));
    torques_queue = xQueueCreate(1, sizeof(float) * 4);

    can_mutex = xSemaphoreCreateMutex();
    spi_mutex = xSemaphoreCreateMutex();  // create mutex for spi
    bms_mutex = xSemaphoreCreateMutex();  // create mutex for bms

    can_init(CAN_RX_PIN, CAN_TX_PIN, 250000);  // 250000 ??
    lcd_init(MOSI_PIN, MISO_PIN, SCK_PIN, LCD_CS_PIN);
    bms_init(MOSI_PIN, MISO_PIN, SCK_PIN, BMS_CS_PIN);

    pinMode(WHEELSPEED_PIN, arduino::INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(WHEELSPEED_PIN), sensor_wheel_speed_update, arduino::FALLING);  // inturupt to check for if the wheel turned

    xTaskCreate(sensor_current_update, "sensor_current_update", 1024, NULL, TaskPriority_SENSOR, NULL);
    xTaskCreate(sensor_steering_angle_update, "sensor_steering_angle_update", 1024, NULL, TaskPriority_SENSOR, NULL);
    xTaskCreate(bms_monitoring_task, "bms_monitoring_task", 1024, NULL, TaskPriority_BMS, NULL);
    xTaskCreate(lcd_update, "lcd_update", 1024, NULL, TaskPriority_LCD, NULL);
    xTaskCreate(torque_update, "torque_update", 1024, NULL, TaskPriority_SENSOR, NULL);  // wasn't started before
    // state machine stuff
    xTaskCreate(state_machine_task, "state_machine_task", 1024, NULL, TaskPriority_SENSOR, NULL);

    vTaskStartScheduler();
}

void loop(void) {}