#include "app_main.h"

#include <cstddef>
#include <cstdint>

#include "mspm0_adc.hpp"
#include "mspm0_gpio.hpp"
#include "mspm0_timebase.hpp"
#include "mspm0_uart.hpp"
#include "thread.hpp"
#include "ti_msp_dl_config.h"

namespace
{

size_t AppendLiteral(char* out, size_t cap, size_t pos, const char* text)
{
  if (cap == 0)
  {
    return pos;
  }

  size_t index = 0;
  while (text[index] != '\0' && pos + 1U < cap)
  {
    out[pos++] = text[index++];
  }
  return pos;
}

size_t AppendU32Dec(char* out, size_t cap, size_t pos, uint32_t value)
{
  if (cap == 0)
  {
    return pos;
  }

  char buffer[10];
  size_t len = 0;
  do
  {
    buffer[len++] = static_cast<char>('0' + (value % 10U));
    value /= 10U;
  } while (value > 0U && len < sizeof(buffer));

  while (len > 0U && pos + 1U < cap)
  {
    out[pos++] = buffer[--len];
  }

  return pos;
}

size_t BuildAdcDiagLine(char* out, size_t cap, uint32_t mv, uint32_t min_mv,
                        uint32_t max_mv, uint32_t avg_mv, uint32_t cnt)
{
  size_t pos = 0;
  pos = AppendLiteral(out, cap, pos, "[mspm0-adc] mv=");
  pos = AppendU32Dec(out, cap, pos, mv);
  pos = AppendLiteral(out, cap, pos, " min=");
  pos = AppendU32Dec(out, cap, pos, min_mv);
  pos = AppendLiteral(out, cap, pos, " max=");
  pos = AppendU32Dec(out, cap, pos, max_mv);
  pos = AppendLiteral(out, cap, pos, " avg=");
  pos = AppendU32Dec(out, cap, pos, avg_mv);
  pos = AppendLiteral(out, cap, pos, " cnt=");
  pos = AppendU32Dec(out, cap, pos, cnt);
  pos = AppendLiteral(out, cap, pos, "\r\n");

  if (cap > 0)
  {
    out[(pos < cap) ? pos : (cap - 1U)] = '\0';
  }

  return pos;
}

}  // namespace

extern "C" void app_main()
{
  LibXR::MSPM0Timebase timebase;
  UNUSED(timebase);

  LibXR::MSPM0GPIO led_gpio(GPIO_LEDS_PORT, GPIO_LEDS_PIN_0_PIN,
                            GPIO_LEDS_PIN_0_IOMUX);
  (void)led_gpio.SetConfig(
      {LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});

  static uint8_t uart_rx_stage_buffer[256];
  static LibXR::MSPM0UART uart(
      MSPM0_UART_INIT(UART_0, uart_rx_stage_buffer, sizeof(uart_rx_stage_buffer), 16,
                      512));
  static LibXR::MSPM0ADC adc(MSPM0_ADC_INIT(ADC_0));

  static const char heartbeat[] = "[mspm0-uart] heartbeat\r\n";
  static char adc_diag[128];
  static uint8_t rx_echo_buffer[1];

  constexpr bool ENABLE_HEARTBEAT = true;
  constexpr uint32_t LOOP_DELAY_MS = 1;
  constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;
  constexpr uint32_t LED_PERIOD_MS = 500;
  constexpr uint32_t ADC_PERIOD_MS = 1000;
  constexpr uint32_t HEARTBEAT_TICKS = HEARTBEAT_PERIOD_MS / LOOP_DELAY_MS;
  constexpr uint32_t LED_TICKS = LED_PERIOD_MS / LOOP_DELAY_MS;
  constexpr uint32_t ADC_TICKS = ADC_PERIOD_MS / LOOP_DELAY_MS;

  bool led_on = false;
  uint32_t tick = 0;
  uint32_t adc_last_mv = 0;
  uint32_t adc_min_mv = UINT32_MAX;
  uint32_t adc_max_mv = 0;
  uint64_t adc_sum_mv = 0;
  uint32_t adc_count = 0;

  LibXR::ReadOperation::OperationPollingStatus read_status =
      LibXR::ReadOperation::OperationPollingStatus::READY;
  LibXR::ReadOperation read_op(read_status);
  LibXR::WriteOperation echo_write_op;
  LibXR::WriteOperation heartbeat_write_op;

  LibXR::RawData read_data = {rx_echo_buffer, sizeof(rx_echo_buffer)};

  while (true)
  {
    bool echoed_this_cycle = false;

    if (read_status == LibXR::ReadOperation::OperationPollingStatus::DONE)
    {
      const size_t read_size = uart.read_port_->read_size_;
      if (read_size > 0)
      {
        (void)uart.Write({rx_echo_buffer, read_size}, echo_write_op);
        echoed_this_cycle = true;
      }
      read_status = LibXR::ReadOperation::OperationPollingStatus::READY;
    }

    if (read_status == LibXR::ReadOperation::OperationPollingStatus::READY)
    {
      (void)uart.Read(read_data, read_op);
    }

    if (ENABLE_HEARTBEAT && !echoed_this_cycle && ((tick % HEARTBEAT_TICKS) == 0U))
    {
      (void)uart.Write(
          {reinterpret_cast<const uint8_t*>(heartbeat), sizeof(heartbeat) - 1},
          heartbeat_write_op);
    }

    if ((tick % ADC_TICKS) == 0U)
    {
      const float adc_voltage = adc.Read();
      adc_last_mv = static_cast<uint32_t>(adc_voltage * 1000.0f + 0.5f);
      if (adc_last_mv < adc_min_mv)
      {
        adc_min_mv = adc_last_mv;
      }
      if (adc_last_mv > adc_max_mv)
      {
        adc_max_mv = adc_last_mv;
      }

      adc_sum_mv += adc_last_mv;
      adc_count++;
      const uint32_t adc_avg_mv = static_cast<uint32_t>(adc_sum_mv / adc_count);

      const size_t line_len =
          BuildAdcDiagLine(adc_diag, sizeof(adc_diag), adc_last_mv, adc_min_mv,
                           adc_max_mv, adc_avg_mv, adc_count);

      if (line_len > 0U)
      {
        const size_t write_size =
            (line_len < sizeof(adc_diag)) ? line_len : (sizeof(adc_diag) - 1U);
        (void)uart.Write({reinterpret_cast<const uint8_t*>(adc_diag), write_size},
                         heartbeat_write_op);
      }
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
