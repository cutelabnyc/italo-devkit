#include <interfaces.hpp>
#include <stdint.h>

class Module : public ModuleInterface {
private:
  typedef struct passthru_t {
    uint16_t dummy;
  } passthru_t;

  void PT_init(passthru_t *pt);
  void PT_destroy(passthru_t *pt);

  void PT_process(passthru_t *pt, uint16_t *in, uint16_t *out,
                  uint8_t channel_count);

public:
  Module();

  void initHardware();
  void process(float msDelta);
};
