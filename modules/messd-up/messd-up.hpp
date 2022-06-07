#include "encoder.hpp"
#include "interfaces.hpp"
#include "mux.hpp"
#include "seven-segment.hpp"

#include <Arduino.h>
#include <cutemodules.h>

class Module : public ModuleInterface {
private:
    messd_t messd;
    messd_ins_t ins;
    messd_outs_t outs;

    // Shift register pins (seven segment)
    int dataPinSR = 11;  // PB3
    int latchPinSR = 9;  // PB1
    int clockPinSR = 10; // PB2

	// Output shift register pins
	int dataPinOuts = 7; // PD7
	int latchPinOuts = 6; // PD6
	int clockPinOuts = 5; // PD5

	// LED shift register pins
	int dataPinLEDs = 8; // PB0
	int latchPinLEDs = A4; // PC4
	int clockPinLEDs = A5; // PC5

    // Mux controller pins
    int m0 = 4; // PD4
    int m1 = 3; // PD3
    int m2 = 2; // PD2

    // consts
    int beatsDivMin = 2;
    int beatsDivMax = 32;

    int analogMuxIn = A0;  // PC0
    int digitalMuxIn = A2; // PC2

    int digitCounter = 0;

    // Storage for the encoders
    int divEncA = 0;
    int divEncB = 0;
    int encStateDiv = 0;
    int beatsEncA = 0;
    int beatsEncB = 0;
    int encStateBeats = 0;

	// Storage for the clock switch
	int clockSwitch = LOW;
	float tapTempo = 120.0;
	float tapTempoOut = 120.0;
	unsigned long lastTapMicros = 0;
	unsigned char totalTaps = 0;
	bool displayTempo = true;

    uint16_t analogMuxOuts[8];
    uint16_t digitalMuxOuts[8];

    // Storage for the pots
    int pot0 = 0;
    int pot1 = 0;

	// Storage for output shift register
	uint8_t output_sr_val[8];
	enum class OutputNames {
		Nothing = 0,
		TruncateLED,
		DivLED,
		EoMOutput,
		TruncateOutput,
		DivOutput,
		DownbeatOutput,
		BeatOutput
	};

	// Storage for LED shift register
	uint8_t leds_sr_val[8];
	enum class LEDNames {
		ModLEDButton = 0,
		EoMLED,
		ClockLEDButton,
		DownbeatLED,
		BeatLED,
		BeatLatchLED,
		DivLatchLED,
		Nothing
	};

    // Storage for the latch switches
    int beat_latch = 0;
    int div_latch = 0;
    int beat_switch_state_prev = 0;
    int div_switch_state_prev = 0;

    mux_t analog_mux = {{m0, m1, m2}, analogMuxIn, true, analogMuxOuts, 8};
    mux_t digital_mux = {{m0, m1, m2}, digitalMuxIn, false, digitalMuxOuts, 8};

    void _scaleValues();
    void _processEncoders();
	void _processTapTempo();

public:
    Module();

    struct state {
        int beats = 4;
        int div = 7;
    } state;

    struct AnalogMux {
        int DIVIDE_ATV = 0;
        int TRUNCATE_ATV = 1;
        int LATCH_SWITCH = 2;
        int BEAT_INPUT = 3;
        int ROUND_SWITCH = 4;
        int DIVIDE_INPUT = 5;
        int MOD_INPUT = 6;
        int TRUNCATE_INPUT = 7;
    } AnalogMux;

    struct DigitalMux {
        int BEAT_ENC_A = 0;
        int BEAT_ENC_B = 1;
        int DIV_SWITCH = 2;
        int BEAT_SWITCH = 3;
        int DIVIDE_ENC_B = 4;
        int CLOCK_IN = 5;
        int DIVIDE_ENC_A = 6;
        int CLOCK_SWITCH = 7;
    } DigitalMux;

    shift_register_t seven_segment_sr = {dataPinSR, latchPinSR, clockPinSR};
	shift_register_t output_sr = {dataPinOuts, latchPinOuts, clockPinOuts};
	shift_register_t leds_sr = {dataPinLEDs, latchPinLEDs, clockPinLEDs};

    encoder_t div_encoder = {HIGH, HIGH, 0};
    encoder_t beat_encoder = {HIGH, HIGH, 0};

    void init();
    void process(float msDelta);
};
