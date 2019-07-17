#include <ros.h>
class Trigger
{
public:
  Trigger(uint8_t pin_t, float duty_t, unsigned int period_t);
  virtual ~Trigger() = default;
  bool TriggerPin();
  bool TriggerPinImmediate();
  uint8_t state;
  float duty_cycle;
  unsigned int period;
  float pulse_high;
  float pulse_low;
  float pulse_time;
  unsigned int sequence;
  unsigned long cur_time;
  unsigned long prev_time;
  uint8_t pin;
};