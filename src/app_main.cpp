#include "app_main.h"

#include <cstdio>
#include <cstring>

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

  static uint8_t spi_rx_dma_buffer[64] = {0};
  static uint8_t spi_tx_dma_buffer[64] = {0};

  struct SPIP0Diagnostic
  {
    uint32_t magic;
    uint32_t round;
    uint32_t total_pass;
    uint32_t total_fail;
    uint32_t ext_pass;
    uint32_t ext_fail;
    uint32_t polling_pass;
    uint32_t polling_fail;
    uint32_t mem_pass;
    uint32_t mem_fail;
    uint32_t err_pass;
    uint32_t err_fail;
    uint32_t last_case;
    uint32_t last_step;
    uint32_t last_error;
    uint32_t last_expect;
    uint32_t last_observe;
    uint32_t last_tx;
    uint32_t last_rx;
  };

  static volatile SPIP0Diagnostic spi_diag = {0x53504930, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                              0,          0, 0, 0, 0, 0, 0, 0, 0};

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
  constexpr uint32_t P0_ROUND_PERIOD_MS = 1000;
  constexpr size_t SPI_FRAME_SIZE = 8;
  constexpr size_t SPI_ASYNC_FRAME_SIZE = 32;
  constexpr uint8_t SPI_DMA_MIN_SIZE = 3;
  constexpr uint32_t CASE_EXTERNAL = 1;
  constexpr uint32_t CASE_POLLING = 2;
  constexpr uint32_t CASE_MEM = 3;
  constexpr uint32_t CASE_ERROR = 4;
  constexpr uint32_t HEARTBEAT_TICKS = HEARTBEAT_PERIOD_MS / LOOP_DELAY_MS;
  constexpr uint32_t LED_TICKS = LED_PERIOD_MS / LOOP_DELAY_MS;
  constexpr uint32_t P0_ROUND_TICKS = P0_ROUND_PERIOD_MS / LOOP_DELAY_MS;

  bool led_on = false;
  uint32_t tick = 0;
  uint8_t loopback_tx[SPI_ASYNC_FRAME_SIZE] = {0};
  uint8_t loopback_rx[SPI_ASYNC_FRAME_SIZE] = {0};
  uint8_t shadow_tx[SPI_ASYNC_FRAME_SIZE] = {0};
  uint8_t shadow_rx[SPI_ASYNC_FRAME_SIZE] = {0};

  LibXR::ReadOperation::OperationPollingStatus read_status =
      LibXR::ReadOperation::OperationPollingStatus::READY;
  LibXR::ReadOperation read_op(read_status);
  LibXR::WriteOperation echo_write_op;
  LibXR::WriteOperation heartbeat_write_op;
  LibXR::WriteOperation p0_report_write_op;

  LibXR::Semaphore spi_sem(0);
  LibXR::SPI::OperationRW spi_block_op(spi_sem, 20);

  LibXR::RawData read_data = {rx_echo_buffer, sizeof(rx_echo_buffer)};
  char p0_report[256] = {0};

  auto set_loopback_mode = [&](bool enable)
  {
    DL_SPI_disable(SPI_0_INST);
    if (enable)
    {
      DL_SPI_enableLoopbackMode(SPI_0_INST);
    }
    else
    {
      DL_SPI_disableLoopbackMode(SPI_0_INST);
    }
    DL_SPI_enable(SPI_0_INST);
  };

  auto all_equal = [](const uint8_t* lhs, const uint8_t* rhs, size_t len) -> bool
  {
    return (len == 0) || (memcmp(lhs, rhs, len) == 0);
  };

  auto all_zero = [](const uint8_t* data, size_t len) -> bool
  {
    for (size_t i = 0; i < len; ++i)
    {
      if (data[i] != 0)
      {
        return false;
      }
    }
    return true;
  };

  auto record_result = [&](uint32_t case_id, uint32_t step_id, bool ok, LibXR::ErrorCode ec,
                           uint32_t expected, uint32_t observed,
                           volatile uint32_t& group_pass, volatile uint32_t& group_fail,
                           uint32_t tx0, uint32_t rx0)
  {
    spi_diag.last_case = case_id;
    spi_diag.last_step = step_id;
    spi_diag.last_error = static_cast<uint32_t>(ec);
    spi_diag.last_expect = expected;
    spi_diag.last_observe = observed;
    spi_diag.last_tx = tx0;
    spi_diag.last_rx = rx0;

    if (ok)
    {
      group_pass++;
      spi_diag.total_pass++;
    }
    else
    {
      group_fail++;
      spi_diag.total_fail++;
    }
  };

  auto run_external_test = [&]()
  {
    uint8_t tx[SPI_FRAME_SIZE] = {0};
    uint8_t rx[SPI_FRAME_SIZE] = {0};
    for (size_t i = 0; i < SPI_FRAME_SIZE; ++i)
    {
      tx[i] = static_cast<uint8_t>(0xA0U + i);
    }

    set_loopback_mode(false);
    LibXR::ErrorCode ec =
        spi.ReadAndWrite({rx, SPI_FRAME_SIZE}, {tx, SPI_FRAME_SIZE}, spi_block_op);
    set_loopback_mode(true);

    bool transfer_ok = (ec == LibXR::ErrorCode::OK);
    record_result(CASE_EXTERNAL, 1, transfer_ok, ec, static_cast<uint32_t>(tx[0]),
                  static_cast<uint32_t>(rx[0]), spi_diag.ext_pass, spi_diag.ext_fail,
                  static_cast<uint32_t>(tx[0]), static_cast<uint32_t>(rx[0]));

    bool echo_ok = transfer_ok && all_equal(rx, tx, SPI_FRAME_SIZE);
    record_result(CASE_EXTERNAL, 2, echo_ok, ec, static_cast<uint32_t>(tx[0]),
                  static_cast<uint32_t>(rx[0]), spi_diag.ext_pass, spi_diag.ext_fail,
                  static_cast<uint32_t>(tx[0]), static_cast<uint32_t>(rx[0]));
  };

  auto run_polling_boundary_test = [&]()
  {
    set_loopback_mode(true);

    const size_t test_sizes[] = {1, 2, 3, 4};
    for (size_t i = 0; i < sizeof(test_sizes) / sizeof(test_sizes[0]); ++i)
    {
      const size_t n = test_sizes[i];
      for (size_t j = 0; j < n; ++j)
      {
        shadow_tx[j] = static_cast<uint8_t>((0x30U + n + j) & 0xFFU);
        shadow_rx[j] = 0U;
      }

      LibXR::ErrorCode ec = spi.ReadAndWrite({shadow_rx, n}, {shadow_tx, n}, spi_block_op);
      bool ok = (ec == LibXR::ErrorCode::OK) && all_equal(shadow_rx, shadow_tx, n);
      record_result(CASE_POLLING, static_cast<uint32_t>(n), ok, ec,
                    static_cast<uint32_t>(shadow_tx[0]),
                    static_cast<uint32_t>(shadow_rx[0]), spi_diag.polling_pass,
                    spi_diag.polling_fail, static_cast<uint32_t>(shadow_tx[0]),
                    static_cast<uint32_t>(shadow_rx[0]));
    }
  };

  auto run_mem_rw_test = [&]()
  {
    set_loopback_mode(true);

    const uint8_t write_poll[2] = {0x11, 0x22};
    LibXR::ErrorCode ec = spi.MemWrite(0x2AU, {write_poll, sizeof(write_poll)}, spi_block_op);
    bool write_poll_ok = (ec == LibXR::ErrorCode::OK) && (spi_tx_dma_buffer[0] == 0x2A) &&
                         (spi_tx_dma_buffer[1] == 0x11) && (spi_tx_dma_buffer[2] == 0x22);
    record_result(CASE_MEM, 1, write_poll_ok, ec, 0x2A, spi_tx_dma_buffer[0], spi_diag.mem_pass,
                  spi_diag.mem_fail, spi_tx_dma_buffer[1], spi_tx_dma_buffer[2]);

    const uint8_t write_dma[7] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
    ec = spi.MemWrite(0x16U, {write_dma, sizeof(write_dma)}, spi_block_op);
    bool write_dma_ok =
        (ec == LibXR::ErrorCode::OK) && (spi_tx_dma_buffer[0] == 0x16) &&
        (memcmp(&spi_tx_dma_buffer[1], write_dma, sizeof(write_dma)) == 0);
    record_result(CASE_MEM, 2, write_dma_ok, ec, 0x16, spi_tx_dma_buffer[0], spi_diag.mem_pass,
                  spi_diag.mem_fail, spi_tx_dma_buffer[1],
                  spi_tx_dma_buffer[sizeof(write_dma)]);

    uint8_t read_poll[2] = {0xAA, 0xAA};
    ec = spi.MemRead(0x33U, {read_poll, sizeof(read_poll)}, spi_block_op);
    bool read_poll_ok = (ec == LibXR::ErrorCode::OK) && (spi_tx_dma_buffer[0] == 0xB3) &&
                        all_zero(read_poll, sizeof(read_poll));
    record_result(CASE_MEM, 3, read_poll_ok, ec, 0xB3, spi_tx_dma_buffer[0], spi_diag.mem_pass,
                  spi_diag.mem_fail, read_poll[0], read_poll[1]);

    uint8_t read_dma[6] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    ec = spi.MemRead(0x55U, {read_dma, sizeof(read_dma)}, spi_block_op);
    bool read_dma_ok = (ec == LibXR::ErrorCode::OK) && (spi_tx_dma_buffer[0] == 0xD5) &&
                       all_zero(read_dma, sizeof(read_dma));
    record_result(CASE_MEM, 4, read_dma_ok, ec, 0xD5, spi_tx_dma_buffer[0], spi_diag.mem_pass,
                  spi_diag.mem_fail, read_dma[0], read_dma[1]);
  };

  auto run_error_injection_test = [&]()
  {
    set_loopback_mode(true);

    LibXR::SPI::Configuration backup = spi.GetConfig();
    LibXR::SPI::Configuration invalid = backup;
    invalid.prescaler = LibXR::SPI::Prescaler::DIV_1;
    LibXR::ErrorCode ec = spi.SetConfig(invalid);
    bool invalid_cfg_ok = (ec == LibXR::ErrorCode::NOT_SUPPORT);
    record_result(CASE_ERROR, 1, invalid_cfg_ok, ec,
                  static_cast<uint32_t>(LibXR::ErrorCode::NOT_SUPPORT),
                  static_cast<uint32_t>(ec), spi_diag.err_pass, spi_diag.err_fail, 0, 0);
    (void)spi.SetConfig(backup);

    for (size_t i = 0; i < SPI_ASYNC_FRAME_SIZE; ++i)
    {
      loopback_tx[i] = static_cast<uint8_t>((0x70U + i) & 0xFFU);
      loopback_rx[i] = 0U;
      shadow_tx[i] = static_cast<uint8_t>((0x90U + i) & 0xFFU);
      shadow_rx[i] = 0U;
    }

    LibXR::SPI::OperationRW::OperationPollingStatus async_status =
        LibXR::SPI::OperationRW::OperationPollingStatus::READY;
    LibXR::SPI::OperationRW async_op(async_status);

    NVIC_DisableIRQ(SPI_0_INST_INT_IRQN);
    ec = spi.ReadAndWrite({loopback_rx, SPI_ASYNC_FRAME_SIZE},
                          {loopback_tx, SPI_ASYNC_FRAME_SIZE}, async_op);
    bool async_start_ok = (ec == LibXR::ErrorCode::OK);
    record_result(CASE_ERROR, 2, async_start_ok, ec,
                  static_cast<uint32_t>(LibXR::ErrorCode::OK), static_cast<uint32_t>(ec),
                  spi_diag.err_pass, spi_diag.err_fail, loopback_tx[0], loopback_rx[0]);

    LibXR::ErrorCode busy_ec = spi.ReadAndWrite({shadow_rx, SPI_FRAME_SIZE},
                                                {shadow_tx, SPI_FRAME_SIZE}, spi_block_op);
    bool busy_ok = (busy_ec == LibXR::ErrorCode::BUSY);
    record_result(CASE_ERROR, 3, busy_ok, busy_ec,
                  static_cast<uint32_t>(LibXR::ErrorCode::BUSY),
                  static_cast<uint32_t>(busy_ec), spi_diag.err_pass, spi_diag.err_fail,
                  shadow_tx[0], shadow_rx[0]);
    NVIC_EnableIRQ(SPI_0_INST_INT_IRQN);

    uint32_t wait_ms = 0;
    while ((async_status != LibXR::SPI::OperationRW::OperationPollingStatus::DONE) &&
           (wait_ms < 20))
    {
      LibXR::Thread::Sleep(1);
      wait_ms++;
    }
    bool async_done_ok =
        (async_status == LibXR::SPI::OperationRW::OperationPollingStatus::DONE);
    record_result(CASE_ERROR, 4, async_done_ok, LibXR::ErrorCode::OK, 1,
                  static_cast<uint32_t>(async_status), spi_diag.err_pass, spi_diag.err_fail,
                  loopback_tx[0], loopback_rx[0]);

    LibXR::ErrorCode recover_ec =
        spi.ReadAndWrite({shadow_rx, SPI_FRAME_SIZE}, {shadow_tx, SPI_FRAME_SIZE}, spi_block_op);
    bool recover_ok =
        (recover_ec == LibXR::ErrorCode::OK) && all_equal(shadow_rx, shadow_tx, SPI_FRAME_SIZE);
    record_result(CASE_ERROR, 5, recover_ok, recover_ec,
                  static_cast<uint32_t>(shadow_tx[0]), static_cast<uint32_t>(shadow_rx[0]),
                  spi_diag.err_pass, spi_diag.err_fail, shadow_tx[0], shadow_rx[0]);
  };

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

    if ((tick % P0_ROUND_TICKS) == 0U)
    {
      run_external_test();
      run_polling_boundary_test();
      run_mem_rw_test();
      run_error_injection_test();

      spi_diag.round++;

      int n = snprintf(
          p0_report, sizeof(p0_report),
          "[spi-p0] r=%lu total=%lu/%lu ext=%lu/%lu poll=%lu/%lu mem=%lu/%lu err=%lu/%lu last=(%lu.%lu) err=%ld exp=0x%02lX obs=0x%02lX tx0=0x%02lX rx0=0x%02lX dmaMin=%u\r\n",
          static_cast<unsigned long>(spi_diag.round),
          static_cast<unsigned long>(spi_diag.total_pass),
          static_cast<unsigned long>(spi_diag.total_fail),
          static_cast<unsigned long>(spi_diag.ext_pass),
          static_cast<unsigned long>(spi_diag.ext_fail),
          static_cast<unsigned long>(spi_diag.polling_pass),
          static_cast<unsigned long>(spi_diag.polling_fail),
          static_cast<unsigned long>(spi_diag.mem_pass),
          static_cast<unsigned long>(spi_diag.mem_fail),
          static_cast<unsigned long>(spi_diag.err_pass),
          static_cast<unsigned long>(spi_diag.err_fail),
          static_cast<unsigned long>(spi_diag.last_case),
          static_cast<unsigned long>(spi_diag.last_step),
          static_cast<long>(static_cast<int>(spi_diag.last_error)),
          static_cast<unsigned long>(spi_diag.last_expect),
          static_cast<unsigned long>(spi_diag.last_observe),
          static_cast<unsigned long>(spi_diag.last_tx),
          static_cast<unsigned long>(spi_diag.last_rx),
          static_cast<unsigned>(SPI_DMA_MIN_SIZE));

      if (n > 0)
      {
        (void)uart.Write({reinterpret_cast<const uint8_t*>(p0_report),
                          static_cast<size_t>(n)},
                         p0_report_write_op);
      }
    }

    LibXR::Thread::Sleep(LOOP_DELAY_MS);
    tick++;
  }
}
