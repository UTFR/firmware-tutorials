#include <Arduino.h>
#include <arduino_freertos.h>
#include <semphr.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

// Functions supplied by the tutorial mock library. Do not reimplement these.
extern void can_init(uint8_t rx, uint8_t tx, uint32_t baudrate);
extern void can_send(uint8_t id, uint64_t payload);
extern void can_receive(uint64_t *payload, uint8_t id);
extern void calculate_torque_cmd(float *torques, float current,
                                 float *wheelspeeds, float steering_angle);
extern void lcd_init(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t lcd_cs);
extern void lcd_printf(const char *fmt, ...);
extern void bms_init(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t bms_cs);
extern float bms_get_voltage(uint8_t n);
extern float bms_get_temperature(uint8_t n);

namespace cfg {
constexpr uint8_t TS_BUTTON = 12, RTD_BUTTON = 13, CURRENT_ADC = 19;
constexpr uint8_t CAN_RX = 2, CAN_TX = 3;
constexpr uint8_t MOSI = 5, MISO = 6, SCK = 7, LCD_CS = 8, BMS_CS = 9;
constexpr uint8_t AIR_PLUS = 22, PRECHARGE = 23, AIR_MINUS = 10;

// REQUIRED CONFIGURATION: the assignment does not supply these values.
// Replace them with validated values from the actual tutorial mock/protocol.
// Until then, CONFIGURED=false prevents energizing AIRs and all tutorial-library
// calls, allowing a disabled build without the external tutorial library.
constexpr bool CONFIGURED = false;
constexpr uint8_t WHEEL_PINS[4] = {255, 255, 255, 255};
constexpr uint8_t CELL_COUNT = 0;
constexpr uint32_t CAN_BAUD = 0;
constexpr uint8_t STEERING_ID = 0, MOTOR_IDS[4] = {0, 0, 0, 0};

constexpr float ADC_REFERENCE_V = 3.3f;
constexpr float CURRENT_ZERO_V = 0.0f; // Must be calibrated for actual Hall sensor.
constexpr uint32_t WHEEL_TIMEOUT_US = 500000;
constexpr uint32_t PRECHARGE_MS = 5000;
}

static_assert(configTICK_RATE_HZ >= 1000,
              "A 1 ms FreeRTOS task requires a tick rate of at least 1000 Hz");

static_assert(sizeof(float) == 4, "CAN packing assumes 32-bit float");

enum class State : uint8_t { LV, PRECHARGE, TS, RTD };
struct Measurements {
    float current = 0, steering = 0;
    float rpm[4] = {}, torque[4] = {};
    float vmin = NAN, vmax = NAN, tmax = NAN;
};

static Measurements readings;
static State state = State::LV;
static bool bms_healthy = false, fault = false;
static SemaphoreHandle_t data_mutex = nullptr;
static SemaphoreHandle_t state_mutex = nullptr;
static SemaphoreHandle_t can_mutex = nullptr;
static SemaphoreHandle_t spi_mutex = nullptr;

static volatile uint32_t last_edge_us[4] = {};
static volatile uint32_t edge_period_us[4] = {};
static volatile bool edge_seen[4] = {};

static void wheelEdge(uint8_t i) {
    const uint32_t now = micros();
    const uint32_t previous = last_edge_us[i];
    const uint32_t elapsed = now - previous;
    // Zero is a valid timestamp, including at micros() rollover. After a gap
    // beyond the timeout, this edge starts a new pair rather than measuring
    // a period that includes the stopped interval.
    edge_period_us[i] = (edge_seen[i] && elapsed <= cfg::WHEEL_TIMEOUT_US)
                            ? elapsed : 0;
    edge_seen[i] = true;
    last_edge_us[i] = now;
}
static void wheelFL() { wheelEdge(0); }
static void wheelFR() { wheelEdge(1); }
static void wheelRL() { wheelEdge(2); }
static void wheelRR() { wheelEdge(3); }

static void openRelays() {
    digitalWrite(cfg::AIR_PLUS, arduino::LOW);
    digitalWrite(cfg::PRECHARGE, arduino::LOW);
    digitalWrite(cfg::AIR_MINUS, arduino::LOW);
}

// state_mutex is the serialization point for relay changes and CAN torque
// transmissions. Callers must hold it before modifying state or transmitting.
static void tripFault() {
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    fault = true; // Latched until reboot.
    state = State::LV;
    openRelays();
    xSemaphoreGive(state_mutex);
}

static float readCurrent() {
    const float volts = analogRead(cfg::CURRENT_ADC) *
                        (cfg::ADC_REFERENCE_V / 4095.0f);
    return (volts - cfg::CURRENT_ZERO_V) * 100.0f; // 1 A / 10 mV
}

static void readWheelRPM(float *rpm) {
    uint32_t periods[4], edges[4];
    noInterrupts();
    for (uint8_t i = 0; i < 4; ++i) {
        periods[i] = edge_period_us[i];
        edges[i] = last_edge_us[i];
    }
    interrupts();
    const uint32_t now = micros();
    for (uint8_t i = 0; i < 4; ++i) {
        rpm[i] = (!periods[i] ||
                  (uint32_t)(now - edges[i]) > cfg::WHEEL_TIMEOUT_US)
                     ? 0.0f : 60000000.0f / (17.0f * periods[i]);
    }
}

// Tutorial specifies uint64_t CAN payload but no wire encoding.
// This is a PLACEHOLDER protocol, not a verified UTFR CAN format.
static uint64_t encodeFloat(float number) {
    uint32_t bits;
    memcpy(&bits, &number, sizeof(bits));
    return static_cast<uint64_t>(bits);
}
static float decodeFloat(uint64_t payload) {
    uint32_t bits = static_cast<uint32_t>(payload);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void motorTask(void *) {
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1);
    for (;;) {
        float rpm[4], torques[4] = {};
        const float current = readCurrent();
        readWheelRPM(rpm);
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        const float steering = readings.steering;
        xSemaphoreGive(data_mutex);

        // Calculate only in RTD. Recheck the state immediately before sending:
        // a BMS trip cannot race with this guarded transmission section.
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        if (cfg::CONFIGURED && !fault && state == State::RTD) {
            calculate_torque_cmd(torques, current, rpm, steering);
            for (float &t : torques) if (!isfinite(t)) t = 0.0f;
        }
        if (cfg::CONFIGURED) {
            xSemaphoreTake(can_mutex, portMAX_DELAY);
            for (uint8_t i = 0; i < 4; ++i)
                can_send(cfg::MOTOR_IDS[i], encodeFloat(torques[i]));
            xSemaphoreGive(can_mutex);
        }
        xSemaphoreGive(state_mutex);

        xSemaphoreTake(data_mutex, portMAX_DELAY);
        readings.current = current;
        for (uint8_t i = 0; i < 4; ++i) {
            readings.rpm[i] = rpm[i];
            readings.torque[i] = torques[i];
        }
        xSemaphoreGive(data_mutex);
        vTaskDelayUntil(&wake, period);
    }
}

static void steeringTask(void *) {
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        if (cfg::CONFIGURED) {
            uint64_t payload = 0;
            // The mock API exposes no receive-success indication.
            // It must be confirmed to be nonblocking and to supply valid data.
            xSemaphoreTake(can_mutex, portMAX_DELAY);
            can_receive(&payload, cfg::STEERING_ID);
            xSemaphoreGive(can_mutex);
            float angle = decodeFloat(payload);
            if (isfinite(angle)) {
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                readings.steering = angle;
                xSemaphoreGive(data_mutex);
            }
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
    }
}

static void bmsTask(void *) {
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        bool bad = !cfg::CONFIGURED || cfg::CELL_COUNT == 0;
        float vmin = INFINITY, vmax = -INFINITY, tmax = -INFINITY;
        if (!bad) {
            xSemaphoreTake(spi_mutex, portMAX_DELAY);
            for (uint8_t i = 0; i < cfg::CELL_COUNT; ++i) {
                const float v = bms_get_voltage(i);
                const float t = bms_get_temperature(i);
                if (!isfinite(v) || !isfinite(t) ||
                    v < 2.8f || v > 4.3f || t > 60.0f) {
                    bad = true;
                    break; // Stop scanning promptly on an unsafe reading.
                }
                if (v < vmin) vmin = v;
                if (v > vmax) vmax = v;
                if (t > tmax) tmax = t;
            }
            xSemaphoreGive(spi_mutex);
        }
        if (bad) tripFault();
        else {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            if (!fault) bms_healthy = true;
            xSemaphoreGive(state_mutex);
        }
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        readings.vmin = bad ? NAN : vmin;
        readings.vmax = bad ? NAN : vmax;
        readings.tmax = bad ? NAN : tmax;
        xSemaphoreGive(data_mutex);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(50));
    }
}

static void stateTask(void *) {
    TickType_t wake = xTaskGetTickCount();
    bool previousTS = arduino::HIGH, previousRTD = arduino::HIGH;
    TickType_t prechargeStart = 0;
    for (;;) {
        const bool ts = digitalRead(cfg::TS_BUTTON);
        const bool rtd = digitalRead(cfg::RTD_BUTTON);
        const bool tsPressed = previousTS == arduino::HIGH && ts == arduino::LOW;
        const bool rtdPressed = previousRTD == arduino::HIGH && rtd == arduino::LOW;
        previousTS = ts;
        previousRTD = rtd;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        if (!fault && bms_healthy && cfg::CONFIGURED) {
            if (state == State::LV && tsPressed) {
                digitalWrite(cfg::AIR_MINUS, arduino::HIGH);
                digitalWrite(cfg::PRECHARGE, arduino::HIGH);
                prechargeStart = xTaskGetTickCount();
                state = State::PRECHARGE;
            } else if (state == State::PRECHARGE &&
                       (xTaskGetTickCount() - prechargeStart) >=
                           pdMS_TO_TICKS(cfg::PRECHARGE_MS)) {
                digitalWrite(cfg::AIR_PLUS, arduino::HIGH);
                digitalWrite(cfg::PRECHARGE, arduino::LOW);
                state = State::TS;
            } else if (state == State::TS && rtdPressed) {
                state = State::RTD;
            }
        }
        xSemaphoreGive(state_mutex);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    }
}

static void lcdTask(void *) {
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        // The LCD API belongs to the external tutorial library. In disabled
        // builds it must not be referenced, just like the CAN and BMS APIs.
        if (!cfg::CONFIGURED) {
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(100));
            continue;
        }
        Measurements snapshot;
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        snapshot = readings;
        xSemaphoreGive(data_mutex);
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        State s = state;
        bool failed = fault;
        xSemaphoreGive(state_mutex);
        const char *label = failed ? "FAULT" :
            s == State::RTD ? "RTD" : s == State::TS ? "TS" :
            s == State::PRECHARGE ? "PRECHARGE" : "LV";
        xSemaphoreTake(spi_mutex, portMAX_DELAY);
        lcd_printf("State: %s\n", label);
        lcd_printf("Current: %.2f A  Steering: %.2f\n",
                   snapshot.current, snapshot.steering);
        lcd_printf("RPM FL/FR/RL/RR: %.1f %.1f %.1f %.1f\n",
                   snapshot.rpm[0], snapshot.rpm[1],
                   snapshot.rpm[2], snapshot.rpm[3]);
        lcd_printf("TQ FL/FR/RL/RR: %.2f %.2f %.2f %.2f\n",
                   snapshot.torque[0], snapshot.torque[1],
                   snapshot.torque[2], snapshot.torque[3]);
        lcd_printf("BMS min/max V: %.2f %.2f  Tmax: %.1f C\n",
                   snapshot.vmin, snapshot.vmax, snapshot.tmax);
        xSemaphoreGive(spi_mutex);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(100));
    }
}

void setup() {
    // Set relay output latches low before enabling GPIO outputs.
    digitalWrite(cfg::AIR_PLUS, arduino::LOW);
    digitalWrite(cfg::AIR_MINUS, arduino::LOW);
    digitalWrite(cfg::PRECHARGE, arduino::LOW);
    pinMode(cfg::AIR_PLUS, arduino::OUTPUT);
    pinMode(cfg::AIR_MINUS, arduino::OUTPUT);
    pinMode(cfg::PRECHARGE, arduino::OUTPUT);
    openRelays();
    pinMode(cfg::TS_BUTTON, arduino::INPUT_PULLUP);
    pinMode(cfg::RTD_BUTTON, arduino::INPUT_PULLUP);
    pinMode(cfg::CURRENT_ADC, arduino::INPUT);
    analogReadResolution(12);

    can_mutex = xSemaphoreCreateMutex();
    spi_mutex = xSemaphoreCreateMutex();
    data_mutex = xSemaphoreCreateMutex();
    state_mutex = xSemaphoreCreateMutex();
    if (!can_mutex || !spi_mutex || !data_mutex || !state_mutex) {
        openRelays();
        while (true) {}
    }

    if (cfg::CONFIGURED) {
        can_init(cfg::CAN_RX, cfg::CAN_TX, cfg::CAN_BAUD);
        for (uint8_t i = 0; i < 4; ++i) pinMode(cfg::WHEEL_PINS[i], arduino::INPUT_PULLUP);
        void (*isrs[4])() = {wheelFL, wheelFR, wheelRL, wheelRR};
        for (uint8_t i = 0; i < 4; ++i)
            attachInterrupt(digitalPinToInterrupt(cfg::WHEEL_PINS[i]),
                            isrs[i], arduino::FALLING);
        lcd_init(cfg::MOSI, cfg::MISO, cfg::SCK, cfg::LCD_CS);
        bms_init(cfg::MOSI, cfg::MISO, cfg::SCK, cfg::BMS_CS);
    }

    if (xTaskCreate(motorTask, "Motor", 768, nullptr, 5, nullptr) != pdPASS ||
        xTaskCreate(bmsTask, "BMS", 640, nullptr, 4, nullptr) != pdPASS ||
        xTaskCreate(stateTask, "State", 384, nullptr, 3, nullptr) != pdPASS ||
        xTaskCreate(steeringTask, "CAN-RX", 384, nullptr, 2, nullptr) != pdPASS ||
        xTaskCreate(lcdTask, "LCD", 768, nullptr, 1, nullptr) != pdPASS) {
        openRelays();
        while (true) {}
    }
    vTaskStartScheduler();
    openRelays();
    while (true) {}
}

void loop() {}
