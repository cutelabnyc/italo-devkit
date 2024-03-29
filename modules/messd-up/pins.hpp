#if USING_MBED_RPI_PICO

#if REVISION == 2

#define MUX_S2      p13
#define MUX_S1      p14
#define MUX_S0      p15

#define OUTS_SHCP   p16
#define OUTS_STCP   p17
#define OUTS_DS     p9

#define LEDS_DS     p10
#define SSEG_STCP   p4
#define SSEG_SHCP   p3
#define SSEG_DS     p5
#define CLK_JACK_SWITCH p0
#define CLK_IN_A    p1

#define MUX_1_COM   p26
#define MUX_2_COM   p2
#define MOD_SW      p6
#define LEDS_STCP   p11
#define LEDS_SHCP   p12

const int SHIFT_PERMUTATION[8] = {4, 0, 2, 1, 3, 5, 6, 7};

#elif REVISION == 1

#define MUX_S2      p13
#define MUX_S1      p14
#define MUX_S0      p15

#define OUTS_SHCP   p16
#define OUTS_STCP   p17
#define OUTS_DS     p8

#define LEDS_DS     p10
#define SSEG_STCP   p5
#define SSEG_SHCP   p6
#define SSEG_DS     p3
#define CLK_JACK_SWITCH p0
#define CLK_IN_A    p2

#define MUX_1_COM   p26
#define MUX_2_COM   p4
#define MOD_SW      p1
#define LEDS_STCP   p12
#define LEDS_SHCP   p11

const int SHIFT_PERMUTATION[8] = {4, 0, 2, 1, 3, 5, 6, 7};

#else

#error "Undefined revision"
#endif

#else

#define MUX_S2      2
#define MUX_S1      3
#define MUX_S0      4

#define OUTS_SHCP   5
#define OUTS_STCP   6
#define OUTS_DS     7

#define LEDS_DS     8
#define SSEG_STCP   9
#define SSEG_SHCP   10
#define SSEG_DS     11
#define CLK_JACK_SWITCH 12
#define CLK_IN_A    A1

#define MUX_1_COM   A0
#define MUX_2_COM   A2
#define MOD_SW      A3
#define LEDS_STCP   A4
#define LEDS_SHCP   A5

const int SHIFT_PERMUTATION[8] = {0, 1, 2, 3, 4, 5, 6, 7};

#endif

