#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>
#include <functional>

template <typename DataType>
class Timer {
public:
  using TimerFunction = void (*)();
  Timer();
  bool start(int maxTime);
  bool tick(float usDelta);
  bool active();

private:
  void *_userData;
  TimerFunction _func;
  bool _active = false;
  int _maxTime;
  int _currentTime;
};

template <typename DataType>
Timer<DataType>::Timer()
{

}

template <typename DataType>
bool Timer<DataType>::start(int maxTime)
{

}

template <typename DataType>
bool Timer<DataType>::tick(float usDelta)
{

}

template <typename DataType>
bool Timer<DataType>::active()
{

}

#endif //TIMER_H
