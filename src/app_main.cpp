#include "app_main.h"

#include <cstdio>

#include "mspm0_gpio.hpp"
#include "mspm0_i2c.hpp"
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

  static uint8_t i2c0_stage_buffer[64] = {0};

  static LibXR::MSPM0I2C i2c0(
      MSPM0_I2C_INIT(I2C_0, i2c0_stage_buffer, sizeof(i2c0_stage_buffer), 8));

  constexpr uint16_t I2C_TEST_DEVICE_ADDR_8BIT = 0xA0U;
  constexpr uint16_t I2C_TEST_MEM_REG = 0x00U;
  constexpr LibXR::I2C::MemAddrLength I2C_TEST_MEM_ADDR_LEN =
      LibXR::I2C::MemAddrLength::BYTE_8;

#if defined(I2C_1_INST)
  struct I2CTargetLoopback
  {
    uint8_t stream_data;
    uint8_t memory[256];
    uint8_t current_reg;
  };

  static volatile I2CTargetLoopback i2c_target_loopback = {
      0x5AU,
      {0},
      I2C_TEST_MEM_REG,
  };
#endif

  struct I2CDiagnostic
  {
    uint32_t magic;
    uint32_t cycle;
    uint32_t write_ok;
    uint32_t read_ok;
    uint32_t mem_write_ok;
    uint32_t mem_read_ok;
    uint32_t failed;
    uint32_t busy;
    uint32_t last_error;
    uint32_t last_read_value;
    uint32_t last_mem_read_value;
  };

  static volatile I2CDiagnostic i2c_diag0 = {
      0x49324344, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  };

  constexpr bool ENABLE_HEARTBEAT = true;
  constexpr uint32_t LOOP_DELAY_MS = 1;
  constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;
  constexpr uint32_t LED_PERIOD_MS = 500;
  constexpr uint32_t I2C_LOOP_PERIOD_MS = 1000;
  constexpr uint32_t HEARTBEAT_TICKS = HEARTBEAT_PERIOD_MS / LOOP_DELAY_MS;
  constexpr uint32_t LED_TICKS = LED_PERIOD_MS / LOOP_DELAY_MS;
  constexpr uint32_t I2C_LOOP_TICKS = I2C_LOOP_PERIOD_MS / LOOP_DELAY_MS;

  bool led_on = false;
  uint32_t tick = 0;

  LibXR::ReadOperation::OperationPollingStatus read_status =
      LibXR::ReadOperation::OperationPollingStatus::READY;
  LibXR::ReadOperation read_op(read_status);
  LibXR::WriteOperation echo_write_op;
  LibXR::WriteOperation heartbeat_write_op;
  LibXR::WriteOperation i2c_report_write_op;

  LibXR::Semaphore i2c0_sem(0);
  LibXR::ReadOperation i2c0_block_read_op(i2c0_sem, 30);
  LibXR::WriteOperation i2c0_block_write_op(i2c0_sem, 30);
  uint8_t i2c0_wr = 0;
  uint8_t i2c0_rd = 0;
  uint8_t i2c0_mem_wr = 0;
  uint8_t i2c0_mem_rd = 0;

  LibXR::RawData read_data = {rx_echo_buffer, sizeof(rx_echo_buffer)};
  char i2c_report[192] = {0};

#if defined(I2C_1_INST)
  DL_I2C_disableController(I2C_1_INST);
  DL_I2C_setTargetAddressingMode(I2C_1_INST, DL_I2C_TARGET_ADDRESSING_MODE_7_BIT);
  DL_I2C_setTargetOwnAddress(I2C_1_INST, (I2C_TEST_DEVICE_ADDR_8BIT >> 1U) & 0x7FU);
  DL_I2C_enableTargetOwnAddress(I2C_1_INST);
  DL_I2C_setTargetTXFIFOThreshold(I2C_1_INST, DL_I2C_TX_FIFO_LEVEL_BYTES_1);
  DL_I2C_setTargetRXFIFOThreshold(I2C_1_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
  DL_I2C_enableTargetClockStretching(I2C_1_INST);
  DL_I2C_enableTarget(I2C_1_INST);
  DL_I2C_clearInterruptStatus(I2C_1_INST, 0xFFFFFFFFU);
  DL_I2C_enableInterrupt(
      I2C_1_INST,
      DL_I2C_INTERRUPT_TARGET_START | DL_I2C_INTERRUPT_TARGET_STOP |
          DL_I2C_INTERRUPT_TARGET_RX_DONE | DL_I2C_INTERRUPT_TARGET_TX_DONE |
          DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
          DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
          DL_I2C_INTERRUPT_TARGET_TXFIFO_EMPTY |
          DL_I2C_INTERRUPT_TARGET_RXFIFO_OVERFLOW |
          DL_I2C_INTERRUPT_TARGET_TXFIFO_UNDERFLOW);
  DL_I2C_transmitTargetDataCheck(I2C_1_INST, i2c_target_loopback.stream_data);
#endif

  auto run_i2c_diag =
      [&](const char* bus_name, LibXR::MSPM0I2C& i2c_dev,
          LibXR::WriteOperation& block_write_op, LibXR::ReadOperation& block_read_op,
          volatile I2CDiagnostic& diag, uint8_t& wr, uint8_t& rd, uint8_t& mem_wr,
          uint8_t& mem_rd)
  {
    diag.cycle++;

    wr = static_cast<uint8_t>(0x30U + (diag.cycle & 0x3FU));
    rd = 0;
    mem_wr = static_cast<uint8_t>(0x80U + (diag.cycle & 0x1FU));
    mem_rd = 0;

    LibXR::ErrorCode ans_write =
        i2c_dev.Write(I2C_TEST_DEVICE_ADDR_8BIT, {&wr, 1}, block_write_op);
    diag.last_error = static_cast<uint32_t>(ans_write);
    if (ans_write == LibXR::ErrorCode::OK)
    {
      diag.write_ok++;
    }
    else
    {
      diag.failed++;
      if (ans_write == LibXR::ErrorCode::BUSY)
      {
        diag.busy++;
      }
    }

    LibXR::ErrorCode ans_read =
        i2c_dev.Read(I2C_TEST_DEVICE_ADDR_8BIT, {&rd, 1}, block_read_op);
    diag.last_error = static_cast<uint32_t>(ans_read);
    diag.last_read_value = static_cast<uint32_t>(rd);
    if (ans_read == LibXR::ErrorCode::OK)
    {
      diag.read_ok++;
    }
    else
    {
      diag.failed++;
      if (ans_read == LibXR::ErrorCode::BUSY)
      {
        diag.busy++;
      }
    }

    LibXR::ErrorCode ans_mem_write =
        i2c_dev.MemWrite(I2C_TEST_DEVICE_ADDR_8BIT, I2C_TEST_MEM_REG, {&mem_wr, 1},
                         block_write_op, I2C_TEST_MEM_ADDR_LEN);
    diag.last_error = static_cast<uint32_t>(ans_mem_write);
    if (ans_mem_write == LibXR::ErrorCode::OK)
    {
      diag.mem_write_ok++;
    }
    else
    {
      diag.failed++;
      if (ans_mem_write == LibXR::ErrorCode::BUSY)
      {
        diag.busy++;
      }
    }

    LibXR::ErrorCode ans_mem_read =
        i2c_dev.MemRead(I2C_TEST_DEVICE_ADDR_8BIT, I2C_TEST_MEM_REG, {&mem_rd, 1},
                        block_read_op, I2C_TEST_MEM_ADDR_LEN);
    diag.last_error = static_cast<uint32_t>(ans_mem_read);
    diag.last_mem_read_value = static_cast<uint32_t>(mem_rd);
    if (ans_mem_read == LibXR::ErrorCode::OK)
    {
      diag.mem_read_ok++;
    }
    else
    {
      diag.failed++;
      if (ans_mem_read == LibXR::ErrorCode::BUSY)
      {
        diag.busy++;
      }
    }

    int n = snprintf(
        i2c_report, sizeof(i2c_report),
        "[i2c-diag][%s] cyc=%lu wr=%lu rd=%lu mwr=%lu mrd=%lu fail=%lu busy=%lu "
        "r=0x%02lX mr=0x%02lX err=%lu\\r\\n",
        bus_name, static_cast<unsigned long>(diag.cycle),
        static_cast<unsigned long>(diag.write_ok),
        static_cast<unsigned long>(diag.read_ok),
        static_cast<unsigned long>(diag.mem_write_ok),
        static_cast<unsigned long>(diag.mem_read_ok),
        static_cast<unsigned long>(diag.failed), static_cast<unsigned long>(diag.busy),
        static_cast<unsigned long>(diag.last_read_value),
        static_cast<unsigned long>(diag.last_mem_read_value),
        static_cast<unsigned long>(diag.last_error));

    if (n > 0)
    {
      (void)uart.Write({reinterpret_cast<const uint8_t*>(i2c_report),
                        static_cast<size_t>(n)},
                       i2c_report_write_op);
    }
  };

#if defined(I2C_1_INST)
  auto i2c_target_loopback_irq = [&]()
  {
    for (uint32_t round = 0; round < 64; ++round)
    {
      const DL_I2C_IIDX iidx = DL_I2C_getPendingInterrupt(I2C_1_INST);
      if (iidx == DL_I2C_IIDX_NO_INT)
      {
        return;
      }

      switch (iidx)
      {
        case DL_I2C_IIDX_TARGET_START:
          break;

        case DL_I2C_IIDX_TARGET_RX_DONE:
        case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
        case DL_I2C_IIDX_TARGET_RXFIFO_FULL:
          while (!DL_I2C_isTargetRXFIFOEmpty(I2C_1_INST))
          {
            const uint8_t value = DL_I2C_receiveTargetData(I2C_1_INST);
            const uint32_t status = DL_I2C_getTargetStatus(I2C_1_INST);

            if ((status & DL_I2C_TARGET_STATUS_RECEIVE_REQUEST) != 0U)
            {
              i2c_target_loopback.current_reg = value;
            }
            else
            {
              i2c_target_loopback.memory[i2c_target_loopback.current_reg] = value;
              i2c_target_loopback.stream_data = value;
              i2c_target_loopback.current_reg =
                  static_cast<uint8_t>(i2c_target_loopback.current_reg + 1U);
            }
          }
          break;

        case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:
        case DL_I2C_IIDX_TARGET_TXFIFO_EMPTY:
        {
          const uint32_t status = DL_I2C_getTargetStatus(I2C_1_INST);
          uint8_t tx = i2c_target_loopback.stream_data;
          if ((status & DL_I2C_TARGET_STATUS_TRANSMIT_REQUEST) != 0U)
          {
            tx = i2c_target_loopback.memory[i2c_target_loopback.current_reg];
            i2c_target_loopback.current_reg =
                static_cast<uint8_t>(i2c_target_loopback.current_reg + 1U);
          }
          DL_I2C_transmitTargetDataCheck(I2C_1_INST, tx);
          break;
        }

        case DL_I2C_IIDX_TARGET_STOP:
        case DL_I2C_IIDX_TARGET_TX_DONE:
          break;

        case DL_I2C_IIDX_TARGET_TXFIFO_UNDERFLOW:
          DL_I2C_flushTargetTXFIFO(I2C_1_INST);
          DL_I2C_transmitTargetDataCheck(I2C_1_INST, i2c_target_loopback.stream_data);
          break;

        case DL_I2C_IIDX_TARGET_RXFIFO_OVERFLOW:
          DL_I2C_flushTargetRXFIFO(I2C_1_INST);
          break;

        default:
          break;
      }
    }
  };
#endif

  while (true)
  {
#if defined(I2C_1_INST)
    i2c_target_loopback_irq();
#endif

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

    if ((tick % I2C_LOOP_TICKS) == 0U)
    {
      run_i2c_diag("I2C_0", i2c0, i2c0_block_write_op, i2c0_block_read_op, i2c_diag0,
                   i2c0_wr, i2c0_rd, i2c0_mem_wr, i2c0_mem_rd);
    }

    LibXR::Thread::Sleep(LOOP_DELAY_MS);
    tick++;
  }
}

