#if USING_MBED_RPI_PICO
#define TIMER_HEADER(function_name) \
void function_name(unsigned int alarm_num) \
{ \
  TIMER_ISR_START(alarm_num);

#define TIMER_FOOTER() \
  TIMER_ISR_END(alarm_num); \
}
#else
#define TIMER_HEADER(function_name) \
void function_name() {

#define TIMER_FOOTER() }
#endif

