#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>
#include <functional>

class Timer {
public:
  using TimerFunction = std::function<void(float)>;
  Timer(TimerFunction callback);
  void start(int maxTimeMicroseconds, int repeats = 0);
  bool tick(float usDelta);
  bool active();
  void clear();

private:
  TimerFunction _callback;
  bool _active = false;
  int _maxTime;
  int _currentTime;
  int _repeats;
};

#endif //TIMER_H
