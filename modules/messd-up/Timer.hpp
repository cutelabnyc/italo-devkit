#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>
#include <functional>

/**
 * Call a callback function repeatedly, up to a certain time.
 *
 * @remarks
 * The function will be called every time the `tick` function is called.
*/
class Timer {
public:
  using TimerFunction = std::function<void(float)>;
  Timer(TimerFunction callback);
  void start(int maxTimeMicroseconds, int repeats = 0);
  bool tick(float usDelta);
  bool active();
  void clear();

  /**
   * Restart the internal timer. Will not reset the repeat counter
  */
  void restart();

private:
  TimerFunction _callback;
  bool _active = false;
  int _maxTime;
  int _currentTime;
  int _repeats;
};

#endif //TIMER_H
