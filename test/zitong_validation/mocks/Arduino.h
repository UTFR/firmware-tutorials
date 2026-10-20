#pragma once
#include <cstdint>
namespace arduino {
constexpr int LOW=0, HIGH=1, INPUT=0, OUTPUT=1, INPUT_PULLUP=2, FALLING=3;
}
uint32_t micros();
int analogRead(uint8_t);
int digitalRead(uint8_t);
void digitalWrite(uint8_t, int);
void pinMode(uint8_t, int);
void analogReadResolution(int);
void noInterrupts();
void interrupts();
int digitalPinToInterrupt(uint8_t);
void attachInterrupt(int, void (*)(), int);
