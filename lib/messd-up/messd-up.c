#include "messd-up.h"

void MAIN_init()
{
	MS_init(&messd);
}

void MAIN_scale(double *in)
{
	// Stuff like this etc.
	// in[TEMPO_KNOB] = (in[TEMPO_KNOB] / 1024.0f)
}

void MAIN_process(double *in, double *out)
{
	MS_process(&messd, in, &out);
}
