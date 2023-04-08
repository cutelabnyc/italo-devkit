#include "Timer.hpp"

Timer::Timer(TimerFunction callback)
: _callback(callback) { }

void Timer::start(int maxTime)
{
  _active = true;
  _maxTime = maxTime;
  _currentTime = 0;
}

bool Timer::tick(float usDelta)
{
  _currentTime += usDelta;

  auto progress = ((float) _currentTime) / ((float) _maxTime);
  progress = fminf(1.0, fmaxf(0.0, progress));
  _callback(progress);

  bool done = progress >= 1.0f;
  _active = !done;
  return done;
}

bool Timer::active()
{
  return _active;
}
