#include <arduino_freertos.h>
#include <stdint.h>
#include <semphr.h>

//Pins:
#define PIN_TS_ON       12
#define PIN_RTD         13
#define PIN_CURRENT     19
#define PIN_CAN_RX      2
#define PIN_CAN_TX      3
#define PIN_MOSI        5
#define PIN_MISO        6
#define PIN_SCK         7
#define PIN_LCD_CS      8
#define PIN_BMS_CS      9
#define PIN_AIR_POS     22
#define PIN_PRECHARGE   23
#define PIN_AIR_NEG     10

//Car States
typedef enum {
    STATE_LV,
    STATE_TS,
    STATE_RTD,
    STATE_FAULT
} CarState_t;

volatile CarState_t current_state = STATE_LV;

//FreeRTOS Synchronization Primitives
SemaphoreHandle_t spiMutex = NULL;
SemaphoreHandle_t canMutex = NULL;

//Declarations
extern void can_init(uint8_t rx, uint8_t tx, uint32_t baudrate);
extern void can_send(uint8_t id, uint64_t payload);
extern void can_receive(uint64_t *payload, uint8_t id);
extern void calculate_torque_cmd(float *torques, float current, float *wheelspeeds, float steering_angle);
extern void lcd_init(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t lcs_cs);
extern void lcd_printf(const char *fmt, ...);
extern void bms_init(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t lcs_cs);
extern float bms_get_voltage(uint8_t n);
extern float bms_get_temperature(uint8_t n);

//Global Sensor Data / Shared Variables
float g_current = 0.0f;
float g_wheelspeeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float g_steering_angle = 0.0f;
float g_max_voltage = 0.0f;
float g_min_voltage = 5.0f;
float g_max_temp = 0.0f;


// Thread-Safe Wrapper for CAN Send and Receiver:
//portMAX_DELAY means wait forever until it becomes available. (you could pass a timeout in milliseconds)
void safe_can_send(uint8_t id, uint64_t payload) {
    if (xSemaphoreTake(canMutex, portMAX_DELAY) == pdTRUE) {
        can_send(id, payload);
        xSemaphoreGive(canMutex);
    }
}

void safe_can_receive(uint64_t *payload, uint8_t id) {
    if (xSemaphoreTake(canMutex, portMAX_DELAY) == pdTRUE) {
        can_receive(payload, id);
        xSemaphoreGive(canMutex);
    }
}

//De-energize or Shutdown  
void trip_shutdown(void) {
    current_state = STATE_FAULT;
    digitalWrite(PIN_AIR_POS, LOW);
    digitalWrite(PIN_PRECHARGE, LOW);
    digitalWrite(PIN_AIR_NEG, LOW);
}

//Task 1: Motor Command & Control Loop (Every 1ms)
void TaskMotorControl(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms
    xLastWakeTime = xTaskGetTickCount();

    for (;;) { //keeps running indefinietly and vTaskDelayUntil bascially puts it to sleep and hands over execution to other tasks - scheduler ...
        //Read Current Sensor (conversion is 1 amp per 10mV )
        int adc_val = analogRead(PIN_CURRENT);
        // Assuming standard 3.3V / 5V ADC mapping, generalised here:
        g_current = (adc_val * (3.3f / 1023.0f)) / 0.010f; 

        // Read Steering Angle via CAN (ID 0x10 as example)
        uint64_t can_payload = 0;
        safe_can_receive(&can_payload, 0x10);
        g_steering_angle = *(float*)&can_payload; // unpacking mock float from payload

        //Calculate torques if in RTD state, else zero torques
        float torques[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        if (current_state == STATE_RTD) {
            calculate_torque_cmd(torques, g_current, g_wheelspeeds, g_steering_angle);
        }

        //Send torque commands or motor control signals via CAN if needed
        //safe_can_send(0x20, *(uint64_t*)&torques[0]);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);//!!!
    }
}

//Task 2: Safety & State Machine Monitor (Every 10ms - 50ms)
void TaskSafetyMonitor(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    
    for (;;) {
        // 1. Check BMS Health across cells (Assume e.g., 10 cells for demonstration)
        float max_v = 0.0f;
        float min_v = 5.0f;
        float max_t = 0.0f;

        if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
            for (uint8_t i = 0; i < 10; i++) {
                float v = bms_get_voltage(i);
                float t = bms_get_temperature(i);
                if (v > max_v) max_v = v;
                if (v < min_v) min_v = v;
                if (t > max_t) max_t = t;
            }
            xSemaphoreGive(spiMutex);
        }

        g_max_voltage = max_v;
        g_min_voltage = min_v;
        g_max_temp = max_t;

        // Safety trip conditions
        if (min_v < 2.8f || max_v > 4.3f || max_t > 60.0f) {
            trip_shutdown();
        }

        // 2. Handle State Transitions (Buttons are active low)
        bool ts_pressed = (digitalRead(PIN_TS_ON) == LOW);
        bool rtd_pressed = (digitalRead(PIN_RTD) == LOW);

        if (current_state == STATE_LV && ts_pressed) {
            // LV -> TS Sequence
            digitalWrite(PIN_AIR_NEG, HIGH);  // Close AIR-
            digitalWrite(PIN_PRECHARGE, HIGH);// Close Precharge Relay
            
            //5 seconds wait for precharge (using FreeRTOS delay)
            vTaskDelay(pdMS_TO_TICKS(5000));

            digitalWrite(PIN_AIR_POS, HIGH);//  Close AIR+
            digitalWrite(PIN_PRECHARGE, LOW);//Open Precharge Relay
            
            current_state = STATE_TS;
        } 
        else if (current_state == STATE_TS && rtd_pressed) {
            current_state = STATE_RTD;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

//Task 3: LCD Display Update (Every 100ms)
void TaskDisplay(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
            lcd_printf("State: %d\n", current_state);
            lcd_printf("I: %.1fA T: %.1fC\n", g_current, g_max_temp);
            lcd_printf("Vmax: %.2fV Vmin: %.2fV\n", g_max_voltage, g_min_voltage);
            xSemaphoreGive(spiMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


void setup(void) {
    // Pins
    pinMode(PIN_TS_ON, INPUT_PULLUP);
    pinMode(PIN_RTD, INPUT_PULLUP);
    pinMode(PIN_AIR_POS, OUTPUT);
    pinMode(PIN_PRECHARGE, OUTPUT);
    pinMode(PIN_AIR_NEG, OUTPUT);

    //Peripherals
    can_init(PIN_CAN_RX, PIN_CAN_TX, 500000);
    lcd_init(PIN_MOSI, PIN_MISO, PIN_SCK, PIN_LCD_CS);
    bms_init(PIN_MOSI, PIN_MISO, PIN_SCK, PIN_BMS_CS);

    //Mutexes for Shared Peripherals!!!
    spiMutex = xSemaphoreCreateMutex();
    canMutex = xSemaphoreCreateMutex();

    //Create FreeRTOS Tasks
    xTaskCreate(TaskMotorControl, "MotorCtrl", 128, NULL, 3, NULL);
    xTaskCreate(TaskSafetyMonitor, "Safety", 128, NULL, 2, NULL);
    xTaskCreate(TaskDisplay, "Display", 128, NULL, 1, NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();
}

void loop(void) {
    //empty when using FreeRTOS scheduler
}