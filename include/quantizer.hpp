#include <Arduino.h>

class Quantizer {
public:
/**
 * Create a Quantizer.
 * @remarks
 * Given an input range, quantizes those results to a smaller output range. Optionally, the midpoint
 * of the range may be skewed, so that the range need not be symmetrical. Hysteresis locks the value
 * to a particular output unless it deviates by more than a specified percentage.
 * @param in_low - Low input range
 * @param in_mid - Middle input range
 * @param in_high - High input range
 * @param out_low - Low output range
 * @param out_high - High output range
 * @param hysteresis - Percentage by which values must deviate to "break out" of their current output value
*/
  Quantizer(int32_t in_low, int32_t in_mid, int32_t in_high, int32_t out_low, int32_t out_high, float hysteresis)
  {
    reset(in_low, in_mid, in_high, out_low, out_high, hysteresis);
  }

/**
 * Re-initialize the Quantizer with new parameters
 * @param in_low - Low input range
 * @param in_mid - Middle input range
 * @param in_high - High input range
 * @param out_low - Low output range
 * @param out_high - High output range
 * @param hysteresis - Percentage by which values must deviate to "break out" of their current output value
*/
  void reset(int32_t in_low, int32_t in_mid, int32_t in_high, int32_t out_low, int32_t out_high, float hysteresis)
  {
    _in_low = in_low;
    _in_mid = in_mid;
    _in_high = in_high;
    _out_low = out_low;
    _out_high = out_high;
    _hysteresis = hysteresis;

    _recalculate();
  }

  /**
   * Special reset function that only sets the input midpoint
   * @param in_mid - Input Midpoint
  */
  void setInMid(int32_t in_mid)
  {
    _in_mid = in_mid;
    _recalculate();
  }

/**
 * Process input, returning quantized value
 * @param input - input value
*/
  int32_t process(int32_t input)
  {
    // If you're between your hysteresis points, then nothing needs to change
    if (input >= _break_low && input <= _break_high) {
      return _last_output;
    }

    // If you're here, then you know that something needs to change
    // First, figure out which bin you're in
    int32_t biased_input = input - _in_mid;
    bool is_upper = biased_input > 0;
    int32_t filled_half_bins = biased_input / ((is_upper ? _grain_high : -_grain_low));

    // Weird special case--you're in the center, which has an asymmetric bin
    if (filled_half_bins == 0) {
      _last_output = 0;
      _break_high = _in_mid + (_grain_high) * (1.0f + _hysteresis);
      _break_low = _in_mid - (_grain_low) * (1.0f + _hysteresis);
    } else if (is_upper) {
      int32_t bin_mid;
      _last_output = (filled_half_bins + 1) / 2;
      bin_mid = _in_mid + ((filled_half_bins + 1) / 2) * (_grain_high * 2);
      _break_high = bin_mid + (_grain_high) * (1.0f + _hysteresis);
      _break_low = bin_mid - (_grain_high) * (1.0f + _hysteresis);
    } else {
      int32_t bin_mid;
      _last_output = -(filled_half_bins + 1) / 2;
      bin_mid = _in_mid - ((filled_half_bins + 1) / 2) * (_grain_low * 2);
      _break_high = bin_mid + (_grain_low) * (1.0f + _hysteresis);
      _break_low = bin_mid - (_grain_low) * (1.0f + _hysteresis);
    }

    if (_inverting) _last_output = -_last_output;
    _last_output += (_out_high + _out_low) / 2;
    return _last_output;
  }

private:
  void _recalculate()
  {
    _inverting = _out_low > _out_high;
    _grain_high = 2 * (_in_high - _in_mid) / (_out_high - _out_low + 1);
    _grain_low = 2 * (_in_mid - _in_low) / (_out_high - _out_low + 1);
    if (_inverting){
      _grain_high = -_grain_high;
      _grain_low = -_grain_low;
    }

    // Guarantee that you'll recalculate on next process
    _break_high = _in_low;
    _break_low = _in_high;
  }

  int32_t _in_low, _in_mid, _in_high, _out_low, _out_high;
  float _hysteresis;
  bool _inverting;
  int32_t _break_high, _break_low;
  int32_t _last_output;
  int32_t _grain_high, _grain_low;
};
