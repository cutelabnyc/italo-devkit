#include "encoder.hpp"
#include "interfaces.hpp"
#include "mux.hpp"
#include "seven-segment.hpp"

#include <Arduino.h>
#include <cutemodules.h>

#define MAX_VOLTAGE	(1023)
#define EOM_BUFFER_MS (10)
#define EOM_LED_BUFFER_MS (250)
#define MOD_BUTTON_STROBE_SLOW (250)
#define MOD_BUTTON_RESET_TIME_MS (2000)
#define TEMPO_DISPLAY_TIME (2000)
#define OTHER_DISPLAY_TIME (2000)
#define DIV_BUTTON_HOLD_TIME (2000)
#define BEAT_BUTTON_HOLD_TIME (2000)

// #define IS_POWERED_FROM_ARDUINO

class Module : public ModuleInterface {
private:
    messd_t messd;
    messd_ins_t ins;
    messd_outs_t outs;
    unsigned long lastProcessTime = 0;
    unsigned long lastRecordedHighClockTime = 0;
    unsigned long measuredPeriod = 500000;

    // Modulation switch gets its own dedicated pin
    uint8_t modSwitchPin = A3; // PC3

    // Shift register pins (seven segment)
    uint8_t dataPinSR = 11;  // PB3
    uint8_t latchPinSR = 9;  // PB1
    uint8_t clockPinSR = 10; // PB2

    // Output shift register pins
    uint8_t dataPinOuts = 7; // PD7
    uint8_t latchPinOuts = 6; // PD6
    uint8_t clockPinOuts = 5; // PD5

    // LED shift register pins
    uint8_t dataPinLEDs = 8; // PB0
    uint8_t latchPinLEDs = A4; // PC4
    uint8_t clockPinLEDs = A5; // PC5

    // Mux controller pins
    uint8_t m0 = 4; // PD4
    uint8_t m1 = 3; // PD3
    uint8_t m2 = 2; // PD2

    // consts
    static const int beatsDivMin = 2;
    static const int beatsDivMax = 32;
    static const int tempoMin = 10;
    static const int tempoMax = 500;
    static const int tapTempoMin = 40;
    static const int tapTempoMax = 280;
    static const int inputClockDivideMin = 1;
    static const int inputClockDivideMax = 9;

    int analogMuxIn = A0;  // PC0
    int digitalMuxIn = A2; // PC2

    uint8_t digitCounter = 0;

    // Storage for the encoders
    uint8_t divEncA = 0;
    uint8_t divEncB = 0;
    uint8_t encStateDiv = 0;
    uint8_t beatsEncA = 0;
    uint8_t beatsEncB = 0;
    uint8_t encStateBeats = 0;

    // Different display types
    enum class DisplayState {
        Default = 0,
        Tempo,
        Pop,
        InputClockDivide,
		BeatMode
    };
    DisplayState displayState = DisplayState::Default;

    // Storage for the clock switch
    uint8_t clockSwitch = LOW;
    float tapTempo = 131.0;
    float scaledTempo = 120.0f;
    unsigned long lastTapMicros = 0;
    unsigned char totalTaps = 0;
    uint16_t tempoDisplayTime = TEMPO_DISPLAY_TIME;

    // Storage for the modulation switch
    uint8_t modSwitch = HIGH; // active low
    uint8_t canTriggerReset = 1;
    float eomBuffer = 0.0f;
    float modHoldTime = 0.0;

	// Holding down the div encoder switch
    float divHoldTime = 0.0;
    uint16_t inputClockDivDisplayTime = OTHER_DISPLAY_TIME;

	// Holding down the beat encoder switch
	float beatHoldTime = 0.0;
	uint16_t beatModeDisplayTime = OTHER_DISPLAY_TIME;
	uint8_t lastBeatInputValue = 0;

    // Storage for animations on the modulate button
    bool animateModulateButton = false;
    float animateModulateButtonTime = 0.0f;

    uint16_t analogMuxOuts[8];
    uint16_t digitalMuxOuts[8];

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
    uint8_t beat_latch = 0;
    uint8_t div_latch = 0;
    uint8_t initial_div_latch = 0;
	uint8_t initial_beat_latch = 0;
    uint8_t beat_switch_state_prev = 0;
    uint8_t div_switch_state_prev = 0;
	uint8_t canSwtichBeatInputModes = 1;

    mux_t analog_mux = {{m0, m1, m2}, analogMuxIn, true, analogMuxOuts, 8};
    mux_t digital_mux = {{m0, m1, m2}, digitalMuxIn, false, digitalMuxOuts, 8};

    void _scaleValues();
    void _processEncoders();
    void _processTapTempo(float msDelta);
    void _processModSwitch(float msDelta);
    void _processBeatDivSwitches(float msDelta);
    void _display();

public:
    Module();

    struct state {
        int beats = 4;
        int div = 7;
        int activeBeats = 4;
        int activeDiv = 7;
    } state;

    struct AnalogMux {
        int DIVIDE_ATV = 0;
        int TRUNCATE_ATV = 1;
        int ROUND_SWITCH = 2;
        int BEAT_INPUT = 3;
        int LATCH_SWITCH = 4;
        int DIVIDE_INPUT = 5;
        int MOD_INPUT = 6;
        int TRUNCATE_INPUT = 7;
    } AnalogMux;

    struct DigitalMux {
        int BEAT_ENC_A = 0;
        int BEAT_ENC_B = 1;
        int BEAT_SWITCH = 2;
        int DIV_SWITCH = 3;
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
    void process(float microsDelta);
};
