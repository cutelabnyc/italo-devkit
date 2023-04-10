#include "Timer.hpp"

Timer::Timer(TimerFunction callback)
: _callback(callback) { }

void Timer::start(int maxTime, int repeats)
{
  _active = true;
  _maxTime = maxTime;
  _currentTime = 0;
  _repeats = repeats;
}

bool Timer::tick(float usDelta)
{
  _currentTime += usDelta;

  auto progress = ((float) _currentTime) / ((float) _maxTime);
  progress = fminf(1.0, fmaxf(0.0, progress));
  _callback(progress);

  bool done = progress >= 1.0f;

  if (done) {
    if (_repeats >= 0) {
      _repeats--;
      if (_repeats <= 0) {
        _repeats = 0;
        _active = false;
      }
    }

    _currentTime = (_currentTime % _maxTime);
  }

  return !_active;
}

bool Timer::active()
{
  return _active;
}

void Timer::clear()
{
  _active = false;
}

void Timer::restart()
{
  _currentTime = 0;
}
