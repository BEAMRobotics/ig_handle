#include "Trigger.h"

Trigger::Trigger(uint8_t pin_t, float duty_t, unsigned int period_t)
    : pin(pin_t), duty_cycle(duty_t), period(period_t)
{
  pulse_high = duty_cycle * period;
  pulse_low = period - pulse_high;
  pulse_time = 0;
  sequence = 0;
  prev_time = 0;
  state = LOW;
  pinMode(pin, OUTPUT);
}

bool Trigger::TriggerPin()
{
  cur_time = micros();
  if (cur_time - prev_time >= pulse_time)
  {
    prev_time = cur_time; //restart timing period
    if (state == LOW)
    {
      state = HIGH;
      pulse_time = pulse_high;
    }
    else
    {
      state = LOW;
      pulse_time = pulse_low;
    }
    digitalWriteFast(pin, state);
    if (state == HIGH)
    {
      return true;
    }
  }
  return false;
}

bool Trigger::TriggerPinImmediate()
{
  cur_time = micros();
  if (cur_time - prev_time >= period)
  {
    prev_time = micros();
    digitalWriteFast(pin, HIGH);
    delay(50);
    digitalWriteFast(pin, LOW);
    return true;
  }
  return false;
}