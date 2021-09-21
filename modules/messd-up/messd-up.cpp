// #include "interfaces.h"
#include <cutemodules.h>

// class Module : public ModuleInterface<messd_t, messd_ins_t, messd_outs_t> {
// private:
// 	uint8_t numInputs;
// 	uint8_t numOutputs;

// 	messd_t messd;
// 	messd_ins_t ins;
// 	messd_outs_t outs;

// public:
// 	void init()
// 	{
// 		MS_init(&this->messd);
// 	};

// 	void process(messd_t self, messd_ins_t ins, messd_outs_t outs)
// 	{
// 		MS_process(&this->messd, &this->ins, &this->outs);
// 	};

// 	pin_t *GPIO_in;
// 	pin_t *GPIO_out;
// };

messd_t messd;
messd_ins_t ins;
messd_outs_t outs;

void main_init()
{
	MS_init(&messd);
};

void main_process()
{
	MS_process(&messd, &ins, &outs);
};

// Module<messd_t, messd_ins_t, messd_outs_t> module;
