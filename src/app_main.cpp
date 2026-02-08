#include "app_main.h"

#include "mspm0_gpio.hpp"
#include "mspm0_timebase.hpp"
#include "thread.hpp"
#include "ti_msp_dl_config.h"

extern "C" void app_main()
{
  DL_SYSTICK_init(CPUCLK_FREQ / 1000U);
  DL_SYSTICK_enableInterrupt();
  DL_SYSTICK_enable();

  LibXR::MSPM0Timebase timebase;
  (void)timebase;

  LibXR::MSPM0GPIO led_gpio(GPIO_GRP_0_PORT, GPIO_GRP_0_PIN_0_PIN,
                            GPIO_GRP_0_PIN_0_IOMUX);

  (void)led_gpio.SetConfig(
      {LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});

  bool led_on = false;

  while (true)
  {
    led_on = !led_on;
    (void)led_gpio.Write(led_on);
    LibXR::Thread::Sleep(500);
  }
}
