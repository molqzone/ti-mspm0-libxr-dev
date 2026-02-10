#include "app_main.h"

#include <cstdio>

#include "mspm0_gpio.hpp"
#include "mspm0_spi.hpp"
#include "mspm0_timebase.hpp"
#include "mspm0_uart.hpp"
#include "semaphore.hpp"
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

  static uint8_t uart_rx_stage_buffer[256];
  static LibXR::MSPM0UART uart(
      MSPM0_UART_INIT(UART_0, uart_rx_stage_buffer, sizeof(uart_rx_stage_buffer), 16,
                      512));

  static const char heartbeat[] = "[mspm0-uart] heartbeat\r\n";
  static uint8_t rx_echo_buffer[1];

  static uint8_t spi_rx_dma_buffer[8] = {0};
  static uint8_t spi_tx_dma_buffer[8] = {0};

  struct SPIDiagnostic
  {
    uint32_t magic;
    uint32_t cycle;
    uint32_t pass_count;
    uint32_t failed_count;
    uint32_t busy_count;
    uint32_t last_error;
    uint32_t last_tx;
    uint32_t last_rx;
    uint32_t last_mismatch_tx;
    uint32_t last_mismatch_rx;
    uint32_t reserved0;
    uint32_t reserved1;
    uint32_t reserved2;
  };

  static volatile SPIDiagnostic spi_diag = {0x53504944,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0};

  static LibXR::MSPM0SPI spi(
      MSPM0_SPI_INIT(SPI_0, DMA_CH0, DMA_CH1, spi_rx_dma_buffer,
                     sizeof(spi_rx_dma_buffer), spi_tx_dma_buffer,
                     sizeof(spi_tx_dma_buffer), 3));

  LibXR::SPI::Configuration spi_config;
  spi_config.clock_polarity = LibXR::SPI::ClockPolarity::LOW;
  spi_config.clock_phase = LibXR::SPI::ClockPhase::EDGE_1;
  spi_config.prescaler = LibXR::SPI::Prescaler::DIV_4;
  spi_config.double_buffer = false;
  (void)spi.SetConfig(spi_config);

  constexpr bool ENABLE_HEARTBEAT = true;
  constexpr uint32_t LOOP_DELAY_MS = 1;
  constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;
  constexpr uint32_t LED_PERIOD_MS = 500;
  constexpr uint32_t SPI_LOOP_PERIOD_MS = 100;
  constexpr uint32_t HEARTBEAT_TICKS = HEARTBEAT_PERIOD_MS / LOOP_DELAY_MS;
  constexpr uint32_t LED_TICKS = LED_PERIOD_MS / LOOP_DELAY_MS;
  constexpr uint32_t SPI_LOOP_TICKS = SPI_LOOP_PERIOD_MS / LOOP_DELAY_MS;

  bool led_on = false;
  uint32_t tick = 0;
  uint8_t loopback_tx = 0x31;
  uint8_t loopback_rx = 0;

  LibXR::ReadOperation::OperationPollingStatus read_status =
      LibXR::ReadOperation::OperationPollingStatus::READY;
  LibXR::ReadOperation read_op(read_status);
  LibXR::WriteOperation echo_write_op;
  LibXR::WriteOperation heartbeat_write_op;
  LibXR::WriteOperation spi_report_write_op;

  LibXR::Semaphore spi_sem(0);
  LibXR::SPI::OperationRW spi_block_op(spi_sem, 20);

  LibXR::RawData read_data = {rx_echo_buffer, sizeof(rx_echo_buffer)};
  char spi_report[128] = {0};

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

    if ((tick % LED_TICKS) == 0U)
    {
      led_on = !led_on;
      (void)led_gpio.Write(led_on);
    }

    if ((tick % SPI_LOOP_TICKS) == 0U)
    {
      loopback_tx = static_cast<uint8_t>(0x31U + (spi_diag.cycle & 0x3FU));
      loopback_rx = 0x00U;

      LibXR::ErrorCode ans =
          spi.ReadAndWrite({&loopback_rx, 1}, {&loopback_tx, 1}, spi_block_op);

      spi_diag.cycle++;
      spi_diag.last_error = static_cast<uint32_t>(ans);
      spi_diag.last_tx = static_cast<uint32_t>(loopback_tx);
      spi_diag.last_rx = static_cast<uint32_t>(loopback_rx);

      if (ans == LibXR::ErrorCode::OK)
      {
        if (loopback_rx == loopback_tx)
        {
          spi_diag.pass_count++;
        }
        else
        {
          spi_diag.failed_count++;
          spi_diag.last_mismatch_tx = static_cast<uint32_t>(loopback_tx);
          spi_diag.last_mismatch_rx = static_cast<uint32_t>(loopback_rx);
        }
      }
      else
      {
        spi_diag.failed_count++;
        if (ans == LibXR::ErrorCode::BUSY)
        {
          spi_diag.busy_count++;
        }
      }

      if ((spi_diag.cycle % 10U) == 0U)
      {
        int n = snprintf(
            spi_report, sizeof(spi_report),
            "[spi-loop] cyc=%lu pass=%lu fail=%lu busy=%lu tx=0x%02lX rx=0x%02lX err=%lu\r\n",
            static_cast<unsigned long>(spi_diag.cycle),
            static_cast<unsigned long>(spi_diag.pass_count),
            static_cast<unsigned long>(spi_diag.failed_count),
            static_cast<unsigned long>(spi_diag.busy_count),
            static_cast<unsigned long>(spi_diag.last_tx),
            static_cast<unsigned long>(spi_diag.last_rx),
            static_cast<unsigned long>(spi_diag.last_error));

        if (n > 0)
        {
          (void)uart.Write({reinterpret_cast<const uint8_t*>(spi_report),
                            static_cast<size_t>(n)},
                           spi_report_write_op);
        }
      }
    }

    LibXR::Thread::Sleep(LOOP_DELAY_MS);
    tick++;
  }
}
