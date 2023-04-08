#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>
#include <functional>

class Timer {
public:
  using TimerFunction = std::function<void(float)>;
  Timer(TimerFunction callback);
  void start(int maxTimeMicroseconds);
  bool tick(float usDelta);
  bool active();

private:
  TimerFunction _callback;
  bool _active = false;
  int _maxTime;
  int _currentTime;
};

#endif //TIMER_H
