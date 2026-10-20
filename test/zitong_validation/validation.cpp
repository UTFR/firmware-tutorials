// Includes the actual implementation so its static functions can be exercised.
// VALIDATION_SOURCE is either the untouched source or an explicitly configured
// temporary copy made by run.py. No firmware is flashed.
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <functional>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include VALIDATION_SOURCE

struct StopTask {};
static uint32_t clock_us=0, ticks=0;
static int adc=0, ts_input=1, rtd_input=1, iterations=1, calculate_calls=0;
static int mutex_storage[4]={}, mutex_count=0;
static int can_init_calls=0, lcd_init_calls=0, bms_init_calls=0, task_count=0;
static std::map<int,int> pins;
static std::vector<std::pair<uint8_t,uint64_t>> sent;
static std::vector<uint32_t> delays;
static float cell_v[3]={3.5f,3.8f,4.0f}, cell_t[3]={20,30,40};
static float commanded[4]={1,2,3,4};
static uint64_t steering_payload=0;
static std::function<void()> on_delay;
static std::string display;

uint32_t micros() { return clock_us; }
int analogRead(uint8_t) { return adc; }
int digitalRead(uint8_t pin) { return pin==cfg::TS_BUTTON ? ts_input : rtd_input; }
void digitalWrite(uint8_t pin,int value) { pins[pin]=value; }
void pinMode(uint8_t,int) {}
void analogReadResolution(int) {}
void noInterrupts() {}
void interrupts() {}
int digitalPinToInterrupt(uint8_t p) { return p; }
void attachInterrupt(int,void (*)(),int) {}
TickType_t xTaskGetTickCount() { return ticks; }
void vTaskDelayUntil(TickType_t *wake,TickType_t period) {
    delays.push_back(period); *wake+=period; ticks=*wake;
    if (on_delay) on_delay();
    if (--iterations==0) throw StopTask{};
}
int xTaskCreate(void (*)(void *),const char *,unsigned,void *,unsigned,void *) { ++task_count; return pdPASS; }
void vTaskStartScheduler() { throw StopTask{}; }
SemaphoreHandle_t xSemaphoreCreateMutex() { return &mutex_storage[mutex_count++]; }
int xSemaphoreTake(SemaphoreHandle_t p,unsigned) {
    if (!p || *p) throw std::runtime_error("invalid or recursive mutex acquisition");
    *p=1; return pdPASS;
}
int xSemaphoreGive(SemaphoreHandle_t p) {
    if (!p || !*p) throw std::runtime_error("unbalanced mutex release");
    *p=0; return pdPASS;
}
void can_init(uint8_t,uint8_t,uint32_t) { ++can_init_calls; }
void can_send(uint8_t id,uint64_t payload) {
    if (!*state_mutex || !*can_mutex) throw std::runtime_error("unguarded CAN send");
    sent.emplace_back(id,payload);
}
void can_receive(uint64_t *payload,uint8_t) { *payload=steering_payload; }
void calculate_torque_cmd(float *torques,float,float *,float) {
    ++calculate_calls; std::copy(commanded,commanded+4,torques);
}
void lcd_init(uint8_t,uint8_t,uint8_t,uint8_t) { ++lcd_init_calls; }
void bms_init(uint8_t,uint8_t,uint8_t,uint8_t) { ++bms_init_calls; }
float bms_get_voltage(uint8_t i) { return cell_v[i]; }
float bms_get_temperature(uint8_t i) { return cell_t[i]; }
void lcd_printf(const char *fmt,...) {
    char buffer[512]; va_list args; va_start(args,fmt);
    vsnprintf(buffer,sizeof(buffer),fmt,args); va_end(args); display+=buffer;
}

static void require(bool ok,const char *message) { if (!ok) throw std::runtime_error(message); }
static void near(float actual,float expected) { require(std::fabs(actual-expected)<0.01f,"numeric result differs from expected"); }
static void runTask(void (*task)(void *),int count=1) {
    iterations=count;
    try { task(nullptr); } catch (const StopTask &) {}
    for (int lock:mutex_storage) require(lock==0,"task left a mutex locked");
}
static void relaysOpen() {
    require(pins[cfg::AIR_PLUS]==0 && pins[cfg::AIR_MINUS]==0 && pins[cfg::PRECHARGE]==0,"relays must be open");
}

int main(int argc,char **argv) {
    if (argc!=2) return 2;
    can_mutex=xSemaphoreCreateMutex(); spi_mutex=xSemaphoreCreateMutex();
    data_mutex=xSemaphoreCreateMutex(); state_mutex=xSemaphoreCreateMutex();
    const std::string name=argv[1];
    try {
        if (name=="float_roundtrip") {
            for (float n:{0.0f,-12.5f,123.25f}) near(decodeFloat(encodeFloat(n)),n);
        } else if (name=="adc_endpoints") {
            adc=0; near(readCurrent(),0); adc=4095; near(readCurrent(),330);
        } else if (name=="wheel_no_edges") {
            float rpm[4]; readWheelRPM(rpm); for (float v:rpm) near(v,0);
        } else if (name=="wheel_frequency") {
            clock_us=100; wheelFL(); clock_us=1100; wheelFL();
            float rpm[4]; readWheelRPM(rpm); near(rpm[0],60000000.0f/17000);
            for (int i=1;i<4;++i) near(rpm[i],0);
        } else if (name=="wheel_timeout") {
            clock_us=100; wheelFL(); clock_us=1100; wheelFL();
            clock_us+=cfg::WHEEL_TIMEOUT_US+1;
            float rpm[4]; readWheelRPM(rpm); near(rpm[0],0);
        } else if (name=="wheel_clock_wrap") {
            clock_us=UINT32_MAX-499; wheelFL(); clock_us=500; wheelFL();
            float rpm[4]; readWheelRPM(rpm); near(rpm[0],60000000.0f/17000);
        } else if (name=="wheel_edge_at_zero") {
            clock_us=0; wheelFL(); clock_us=1000; wheelFL();
            float rpm[4]; readWheelRPM(rpm); near(rpm[0],60000000.0f/17000);
        } else if (name=="wheel_wrap_to_zero") {
            clock_us=UINT32_MAX-999; wheelFL(); clock_us=0; wheelFL();
            float rpm[4]; readWheelRPM(rpm); near(rpm[0],60000000.0f/17000);
        } else if (name=="wheel_single_edge" || name=="wheel_duplicate_edge") {
            clock_us=100; wheelFL();
            if (name=="wheel_duplicate_edge") wheelFL();
            float rpm[4]; readWheelRPM(rpm); near(rpm[0],0);
        } else if (name=="wheel_timeout_boundary") {
            clock_us=100; wheelFL(); clock_us+=cfg::WHEEL_TIMEOUT_US; wheelFL();
            float rpm[4]; readWheelRPM(rpm);
            near(rpm[0],60000000.0f/(17.0f*cfg::WHEEL_TIMEOUT_US));
            clock_us+=cfg::WHEEL_TIMEOUT_US; readWheelRPM(rpm);
            require(rpm[0]>0,"measurement expired at inclusive timeout boundary");
            ++clock_us; readWheelRPM(rpm); near(rpm[0],0);
        } else if (name=="wheel_restart_after_timeout") {
            clock_us=100; wheelFL(); clock_us=1100; wheelFL();
            clock_us+=cfg::WHEEL_TIMEOUT_US+1; wheelFL();
            float rpm[4]; readWheelRPM(rpm);
            require(rpm[0]==0,"first edge after timeout uses stale pre-stop edge to report nonzero RPM");
            clock_us+=1000; wheelFL(); readWheelRPM(rpm);
            near(rpm[0],60000000.0f/17000);
        } else if (name=="wheel_restart_across_wrap") {
            clock_us=UINT32_MAX-2000; wheelFR(); clock_us+=1000; wheelFR();
            clock_us+=cfg::WHEEL_TIMEOUT_US+1; wheelFR();
            float rpm[4]; readWheelRPM(rpm); near(rpm[1],0);
            clock_us+=1000; wheelFR(); readWheelRPM(rpm); near(rpm[1],60000000.0f/17000);
        } else if (name=="fault_latch") {
            state=State::RTD; pins[cfg::AIR_PLUS]=pins[cfg::AIR_MINUS]=pins[cfg::PRECHARGE]=1;
            tripFault(); require(fault && state==State::LV,"fault must latch and enter LV"); relaysOpen();
            bms_healthy=true; ts_input=0; runTask(stateTask); require(state==State::LV,"fault allowed restart");
        } else if (name=="default_motor_inhibited") {
            require(!cfg::CONFIGURED,"test requires original defaults");
            state=State::RTD; // Disabled config must inhibit even an inconsistent RTD state.
            runTask(motorTask); require(sent.empty() && calculate_calls==0,"unconfigured motor activity");
        } else if (name=="default_lcd_inhibited") {
            fault=true; runTask(lcdTask);
            require(display.empty(),"unconfigured build called external LCD API");
            require(delays[0]==100,"disabled LCD task period differs from 100 ms");
        } else if (name=="setup_library_gating") {
            mutex_count=0;
            try { setup(); } catch (const StopTask &) {}
            const int expected=cfg::CONFIGURED ? 1 : 0;
            require(can_init_calls==expected && lcd_init_calls==expected && bms_init_calls==expected,
                    "setup did not gate external library initialization on configuration");
            require(task_count==5,"setup did not create all five tasks"); relaysOpen();
        } else if (name=="default_bms_fault") {
            runTask(bmsTask); require(fault && state==State::LV,"unconfigured BMS must fault"); relaysOpen();
            require(std::isnan(readings.vmin),"invalid BMS must display invalid voltage");
        } else if (name=="default_button_inhibited") {
            bms_healthy=true; ts_input=0; runTask(stateTask); require(state==State::LV,"unconfigured state advanced"); relaysOpen();
        } else if (name=="motor_rtd" || name=="motor_lv" || name=="motor_fault" || name=="motor_nonfinite") {
            state=name=="motor_lv" ? State::LV : State::RTD; fault=name=="motor_fault";
            if (name=="motor_nonfinite") { commanded[0]=NAN; commanded[1]=INFINITY; }
            runTask(motorTask); require(sent.size()==4,"expected four CAN commands");
            for (int i=0;i<4;++i) {
                require(sent[i].first==cfg::MOTOR_IDS[i],"wrong motor CAN ID");
                float expected=(state==State::RTD && !fault) ? commanded[i] : 0;
                if (!std::isfinite(expected)) expected=0;
                near(decodeFloat(sent[i].second),expected);
            }
            require(delays[0]==1,"motor task period differs from 1 ms");
        } else if (name=="bms_healthy" || name=="bms_boundaries") {
            if (name=="bms_boundaries") { cell_v[0]=2.8f; cell_v[2]=4.3f; cell_t[2]=60; }
            runTask(bmsTask); require(bms_healthy && !fault,"valid battery rejected");
            near(readings.vmin,cell_v[0]); near(readings.vmax,cell_v[2]); near(readings.tmax,cell_t[2]);
            require(delays[0]==50,"BMS task period differs from 50 ms");
        } else if (name.rfind("bms_bad_",0)==0) {
            if (name=="bms_bad_low") cell_v[1]=2.79f;
            else if (name=="bms_bad_high") cell_v[1]=4.31f;
            else if (name=="bms_bad_hot") cell_t[1]=60.01f;
            else if (name=="bms_bad_nan") cell_v[1]=NAN;
            else if (name=="bms_bad_inf") cell_t[1]=INFINITY;
            else throw std::runtime_error("unknown BMS test");
            state=State::RTD; runTask(bmsTask);
            require(fault && state==State::LV,"unsafe battery did not trip"); relaysOpen();
            cell_v[1]=3.8f; cell_t[1]=30; runTask(bmsTask); require(fault,"healthy scan cleared fault");
        } else if (name=="precharge_sequence") {
            bms_healthy=true; ts_input=0;
            on_delay=[] {
                if (delays.size()==1) { require(state==State::PRECHARGE,"TS did not start precharge");
                    require(pins[cfg::AIR_MINUS]==1 && pins[cfg::PRECHARGE]==1 && pins[cfg::AIR_PLUS]==0,"incorrect precharge relays"); }
                if (delays.size()==500) require(state==State::PRECHARGE,"precharge ended before 5 seconds");
                if (delays.size()==501) { require(state==State::TS,"precharge did not finish at 5 seconds");
                    require(pins[cfg::AIR_PLUS]==1 && pins[cfg::PRECHARGE]==0,"incorrect TS relays"); rtd_input=0; }
            };
            runTask(stateTask,502); require(state==State::RTD,"RTD button did not enable RTD");
        } else if (name=="bms_gate") {
            ts_input=0; runTask(stateTask); require(state==State::LV,"started before healthy BMS"); relaysOpen();
        } else if (name=="steering_valid" || name=="steering_nan") {
            readings.steering=12; steering_payload=encodeFloat(name=="steering_valid" ? 25.0f : NAN);
            runTask(steeringTask); near(readings.steering,name=="steering_valid" ? 25 : 12);
            require(delays[0]==5,"steering task period differs from 5 ms");
        } else if (name=="lcd_fault") {
            fault=true; runTask(lcdTask); require(display.find("State: FAULT")!=std::string::npos,"LCD omitted FAULT");
            require(delays[0]==100,"LCD task period differs from 100 ms");
        } else throw std::runtime_error("unknown test");
        std::cout<<"PASS "<<name<<'\n'; return 0;
    } catch (const std::exception &e) { std::cout<<"FAIL "<<name<<": "<<e.what()<<'\n'; return 1; }
}
