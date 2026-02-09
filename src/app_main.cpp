#include "app_main.h"

#include "mspm0_gpio.hpp"
#include "mspm0_timebase.hpp"
#include "mspm0_uart.hpp"
#include "thread.hpp"
#include "ti_msp_dl_config.h"

extern "C" void app_main()
{
  LibXR::MSPM0Timebase timebase;
  UNUSED(timebase);

  LibXR::MSPM0GPIO led_gpio(GPIO_GRP_0_PORT, GPIO_GRP_0_PIN_0_PIN,
                            GPIO_GRP_0_PIN_0_IOMUX);

  (void)led_gpio.SetConfig(
      {LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});

  static uint8_t uart_rx_stage_buffer[128];
  static LibXR::MSPM0UART uart(
      MSPM0_UART_INIT(UART_0, uart_rx_stage_buffer, sizeof(uart_rx_stage_buffer), 8,
                      256));

  static const char heartbeat[] = "[mspm0-uart] heartbeat\r\n";
  static uint8_t rx_echo_buffer[1];

  bool led_on = false;
  uint32_t tick = 0;

  constexpr uint32_t LOOP_DELAY_MS = 10;
  constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;
  constexpr uint32_t LED_PERIOD_MS = 500;
  constexpr uint32_t HEARTBEAT_TICKS = HEARTBEAT_PERIOD_MS / LOOP_DELAY_MS;
  constexpr uint32_t LED_TICKS = LED_PERIOD_MS / LOOP_DELAY_MS;

  LibXR::ReadOperation::OperationPollingStatus read_status =
      LibXR::ReadOperation::OperationPollingStatus::READY;
  LibXR::ReadOperation read_op(read_status);
  LibXR::WriteOperation write_op;

  LibXR::RawData read_data = {rx_echo_buffer, sizeof(rx_echo_buffer)};

  while (true)
  {
    if ((tick % HEARTBEAT_TICKS) == 0U)
    {
      (void)uart.Write(
          {reinterpret_cast<const uint8_t*>(heartbeat), sizeof(heartbeat) - 1}, write_op);
    }

    switch (read_status)
    {
      case LibXR::ReadOperation::OperationPollingStatus::READY:
        (void)uart.Read(read_data, read_op);
        break;

      case LibXR::ReadOperation::OperationPollingStatus::RUNNING:
        break;

      case LibXR::ReadOperation::OperationPollingStatus::DONE:
        if (uart.read_port_->read_size_ > 0)
        {
          (void)uart.Write({rx_echo_buffer, uart.read_port_->read_size_}, write_op);
        }
        read_status = LibXR::ReadOperation::OperationPollingStatus::READY;
        break;
    }

    if ((tick % LED_TICKS) == 0U)
    {
      led_on = !led_on;
      (void)led_gpio.Write(led_on);
    }

    LibXR::Thread::Sleep(LOOP_DELAY_MS);
    tick++;
  }
}
