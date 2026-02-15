#include "app_main.h"

#include <cstddef>
#include <cstdint>

#include "mspm0_adc.hpp"
#include "mspm0_canfd.hpp"
#include "mspm0_gpio.hpp"
#include "mspm0_timebase.hpp"
#include "mspm0_uart.hpp"
#include "thread.hpp"
#include "ti_msp_dl_config.h"

extern "C" volatile uint32_t g_can_loopback_init_stage;

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
                        uint32_t max_mv, uint32_t avg_mv, uint32_t rms_mv,
                        uint32_t cnt, uint32_t win_rms_mv, uint32_t win_id,
                        uint32_t win_fill)
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
  pos = AppendLiteral(out, cap, pos, " rms=");
  pos = AppendU32Dec(out, cap, pos, rms_mv);
  pos = AppendLiteral(out, cap, pos, " cnt=");
  pos = AppendU32Dec(out, cap, pos, cnt);
  pos = AppendLiteral(out, cap, pos, " wrms=");
  pos = AppendU32Dec(out, cap, pos, win_rms_mv);
  pos = AppendLiteral(out, cap, pos, " wid=");
  pos = AppendU32Dec(out, cap, pos, win_id);
  pos = AppendLiteral(out, cap, pos, " wfill=");
  pos = AppendU32Dec(out, cap, pos, win_fill);
  pos = AppendLiteral(out, cap, pos, "/256");
  pos = AppendLiteral(out, cap, pos, "\r\n");

  if (cap > 0)
  {
    out[(pos < cap) ? pos : (cap - 1U)] = '\0';
  }

  return pos;
}

uint32_t IsqrtU64(uint64_t value)
{
  uint64_t op = value;
  uint64_t res = 0;
  uint64_t one = static_cast<uint64_t>(1) << 62;

  while (one > op)
  {
    one >>= 2;
  }

  while (one != 0)
  {
    if (op >= res + one)
    {
      op -= (res + one);
      res += (one << 1);
    }
    res >>= 1;
    one >>= 2;
  }

  return static_cast<uint32_t>(res);
}

size_t BuildCanDiagLine(char* out, size_t cap, uint32_t tx_cnt, uint32_t rx_cnt,
                        uint32_t id, uint32_t type, uint32_t data0)
{
  size_t pos = 0;
  pos = AppendLiteral(out, cap, pos, "[mspm0-can] loopback tx=");
  pos = AppendU32Dec(out, cap, pos, tx_cnt);
  pos = AppendLiteral(out, cap, pos, " rx=");
  pos = AppendU32Dec(out, cap, pos, rx_cnt);
  pos = AppendLiteral(out, cap, pos, " id=");
  pos = AppendU32Dec(out, cap, pos, id);
  pos = AppendLiteral(out, cap, pos, " type=");
  pos = AppendU32Dec(out, cap, pos, type);
  pos = AppendLiteral(out, cap, pos, " d0=");
  pos = AppendU32Dec(out, cap, pos, data0);
  pos = AppendLiteral(out, cap, pos, "\r\n");

  if (cap > 0)
  {
    out[(pos < cap) ? pos : (cap - 1U)] = '\0';
  }

  return pos;
}

const char* UartModeToText(LibXR::MSPM0UART::RxTimeoutMode mode)
{
  switch (mode)
  {
    case LibXR::MSPM0UART::RxTimeoutMode::LIN_COMPARE:
      return "LIN";
    case LibXR::MSPM0UART::RxTimeoutMode::BYTE_INTERRUPT:
    default:
      return "BYTE";
  }
}

size_t BuildUartHeartbeatLine(char* out, size_t cap, LibXR::MSPM0UART::RxTimeoutMode mode,
                              uint32_t read_req, uint32_t timeout_count,
                              uint32_t timeout_enabled, uint32_t timeout_mis,
                              uint32_t timeout_ris, uint32_t timeout_value,
                              uint32_t rx_fifo_threshold, uint32_t read_submit_count,
                              uint32_t read_done_count, uint32_t read_partial_count,
                              uint32_t echo_frame_count, uint32_t echo_byte_count,
                              uint32_t rx_drop_count, uint32_t timeout_delta)
{
  size_t pos = 0;
  pos = AppendLiteral(out, cap, pos, "[mspm0-uart] mode=");
  pos = AppendLiteral(out, cap, pos, UartModeToText(mode));
  pos = AppendLiteral(out, cap, pos, " req=");
  pos = AppendU32Dec(out, cap, pos, read_req);
  pos = AppendLiteral(out, cap, pos, " tmo=");
  pos = AppendU32Dec(out, cap, pos, timeout_count);
  pos = AppendLiteral(out, cap, pos, " en=");
  pos = AppendU32Dec(out, cap, pos, timeout_enabled);
  pos = AppendLiteral(out, cap, pos, " mis=");
  pos = AppendU32Dec(out, cap, pos, timeout_mis);
  pos = AppendLiteral(out, cap, pos, " ris=");
  pos = AppendU32Dec(out, cap, pos, timeout_ris);
  pos = AppendLiteral(out, cap, pos, " rto=");
  pos = AppendU32Dec(out, cap, pos, timeout_value);
  pos = AppendLiteral(out, cap, pos, " th=");
  pos = AppendU32Dec(out, cap, pos, rx_fifo_threshold);
  pos = AppendLiteral(out, cap, pos, " rdsub=");
  pos = AppendU32Dec(out, cap, pos, read_submit_count);
  pos = AppendLiteral(out, cap, pos, " rddone=");
  pos = AppendU32Dec(out, cap, pos, read_done_count);
  pos = AppendLiteral(out, cap, pos, " rdpart=");
  pos = AppendU32Dec(out, cap, pos, read_partial_count);
  pos = AppendLiteral(out, cap, pos, " echof=");
  pos = AppendU32Dec(out, cap, pos, echo_frame_count);
  pos = AppendLiteral(out, cap, pos, " echob=");
  pos = AppendU32Dec(out, cap, pos, echo_byte_count);
  pos = AppendLiteral(out, cap, pos, " drop=");
  pos = AppendU32Dec(out, cap, pos, rx_drop_count);
  pos = AppendLiteral(out, cap, pos, " dtmo=");
  pos = AppendU32Dec(out, cap, pos, timeout_delta);
  pos = AppendLiteral(out, cap, pos, " heartbeat\r\n");

  if (cap > 0)
  {
    out[(pos < cap) ? pos : (cap - 1U)] = '\0';
  }

  return pos;
}

#if defined(__MSPM0_HAS_MCAN__) && defined(CANFD0_BASE)
bool CheckCanfd0SyscfgReady()
{
  if (!DL_MCAN_isPowerEnabled(CANFD0))
  {
    g_can_loopback_init_stage = 901U;
    return false;
  }
  if (!DL_MCAN_isMemInitDone(CANFD0))
  {
    g_can_loopback_init_stage = 902U;
    return false;
  }
  if (DL_MCAN_getOpMode(CANFD0) != DL_MCAN_OPERATION_MODE_NORMAL)
  {
    g_can_loopback_init_stage = 903U;
    return false;
  }

  g_can_loopback_init_stage = 910U;
  return true;
}
#endif

}  // namespace

extern "C"
{
volatile uint32_t g_adc_last_mv = 0;
volatile uint32_t g_adc_min_mv = 0;
volatile uint32_t g_adc_max_mv = 0;
volatile uint32_t g_adc_avg_mv = 0;
volatile uint32_t g_adc_rms_mv = 0;
volatile uint32_t g_adc_sample_count = 0;
volatile uint32_t g_adc_win_rms_mv = 0;
volatile uint32_t g_adc_win_avg_mv = 0;
volatile uint32_t g_adc_win_min_mv = 0;
volatile uint32_t g_adc_win_max_mv = 0;
volatile uint32_t g_adc_win_fill_count = 0;
volatile uint32_t g_adc_win_id = 0;
volatile uint32_t g_can_loopback_tx_count = 0;
volatile uint32_t g_can_loopback_rx_count = 0;
volatile uint32_t g_can_loopback_last_id = 0;
volatile uint32_t g_can_loopback_last_type = 0;
volatile uint32_t g_can_loopback_last_data0 = 0;
volatile uint32_t g_can_loopback_init_ok = 0;
volatile uint32_t g_can_loopback_init_stage = 0;
volatile uint32_t g_canfd_loopback_tx_count = 0;
volatile uint32_t g_canfd_loopback_rx_count = 0;
volatile uint32_t g_canfd_loopback_last_id = 0;
volatile uint32_t g_canfd_loopback_last_type = 0;
volatile uint32_t g_canfd_loopback_last_len = 0;
volatile uint32_t g_canfd_loopback_last_data0 = 0;
}

#if defined(__MSPM0_HAS_MCAN__) && defined(CANFD0_BASE)
void OnCanLoopback(bool in_isr, void* context, const LibXR::CAN::ClassicPack& pack)
{
  UNUSED(in_isr);
  UNUSED(context);

  g_can_loopback_rx_count++;
  g_can_loopback_last_id = pack.id;
  g_can_loopback_last_type = static_cast<uint32_t>(pack.type);
  g_can_loopback_last_data0 = pack.data[0];
}

void OnCanfdLoopback(bool in_isr, void* context, const LibXR::FDCAN::FDPack& pack)
{
  UNUSED(in_isr);
  UNUSED(context);

  g_canfd_loopback_rx_count++;
  g_canfd_loopback_last_id = pack.id;
  g_canfd_loopback_last_type = static_cast<uint32_t>(pack.type);
  g_canfd_loopback_last_len = pack.len;
  g_canfd_loopback_last_data0 = pack.data[0];
}
#endif

extern "C" void app_main()
{
  LibXR::MSPM0Timebase timebase;
  UNUSED(timebase);

  LibXR::MSPM0GPIO led_gpio(GPIO_LEDS_PORT, GPIO_LEDS_PIN_0_PIN,
                            GPIO_LEDS_PIN_0_IOMUX);
  (void)led_gpio.SetConfig(
      {LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});

  static uint8_t uart_rx_stage_buffer[256];
  constexpr uint32_t UART_TX_QUEUE_SIZE = 64;
  constexpr uint32_t UART_TX_BUFFER_SIZE = 512;
#if defined(UART_0_INST)
  static LibXR::MSPM0UART uart(
      MSPM0_UART_INIT(UART_0, uart_rx_stage_buffer, sizeof(uart_rx_stage_buffer),
                      UART_TX_QUEUE_SIZE, UART_TX_BUFFER_SIZE));
#elif defined(UART_3_INST)
  static LibXR::MSPM0UART uart(
      MSPM0_UART_INIT(UART_3, uart_rx_stage_buffer, sizeof(uart_rx_stage_buffer),
                      UART_TX_QUEUE_SIZE, UART_TX_BUFFER_SIZE));
#elif defined(UART_2_INST)
  static LibXR::MSPM0UART uart(
      MSPM0_UART_INIT(UART_2, uart_rx_stage_buffer, sizeof(uart_rx_stage_buffer),
                      UART_TX_QUEUE_SIZE, UART_TX_BUFFER_SIZE));
#elif defined(UART_1_INST)
  static LibXR::MSPM0UART uart(
      MSPM0_UART_INIT(UART_1, uart_rx_stage_buffer, sizeof(uart_rx_stage_buffer),
                      UART_TX_QUEUE_SIZE, UART_TX_BUFFER_SIZE));
#else
  static LibXR::MSPM0UART uart(
      MSPM0_UART_INIT(UART_0, uart_rx_stage_buffer, sizeof(uart_rx_stage_buffer),
                      UART_TX_QUEUE_SIZE, UART_TX_BUFFER_SIZE));
#endif
  static LibXR::MSPM0ADC adc(MSPM0_ADC_INIT(ADC_0));
#if defined(__MSPM0_HAS_MCAN__) && defined(CANFD0_BASE)
  const bool can_init_ok = CheckCanfd0SyscfgReady();
  g_can_loopback_init_ok = can_init_ok ? 1U : 0U;
  LibXR::MSPM0CANFD* can_ptr = nullptr;
  if (can_init_ok)
  {
    static LibXR::MSPM0CANFD can(
        {CANFD0, CANFD0_INT_IRQn, LibXR::MSPM0CANFD::ResolveIndex(CANFD0_INT_IRQn)}, 8);
    can.Register(LibXR::CAN::Callback::Create(OnCanLoopback, static_cast<void*>(nullptr)),
                 LibXR::CAN::Type::STANDARD, LibXR::CAN::FilterMode::ID_RANGE, 0U,
                 0x7FFU);
    can.Register(
        LibXR::FDCAN::CallbackFD::Create(OnCanfdLoopback, static_cast<void*>(nullptr)),
        LibXR::CAN::Type::STANDARD, LibXR::CAN::FilterMode::ID_RANGE, 0U, 0x7FFU);
    can_ptr = &can;
  }
#endif

  static char uart_heartbeat[96];
  static char adc_diag[128];
  static char can_diag[128];
  static uint8_t rx_echo_buffer[64];

  constexpr uint32_t READ_REQ_TIMEOUT_MODE = 16;
  constexpr uint32_t READ_REQ_BYTE_MODE = 1;

  constexpr bool ENABLE_HEARTBEAT = true;
  constexpr uint32_t LOOP_DELAY_MS = 1;
  constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;
  constexpr uint32_t LED_PERIOD_MS = 500;
  constexpr uint32_t ADC_SAMPLE_PERIOD_MS = 10;
  constexpr uint32_t ADC_REPORT_PERIOD_MS = 1000;
  constexpr uint32_t ADC_RMS_WINDOW_SIZE = 256;
  constexpr uint32_t HEARTBEAT_TICKS = HEARTBEAT_PERIOD_MS / LOOP_DELAY_MS;
  constexpr uint32_t LED_TICKS = LED_PERIOD_MS / LOOP_DELAY_MS;
  constexpr uint32_t ADC_SAMPLE_TICKS = ADC_SAMPLE_PERIOD_MS / LOOP_DELAY_MS;
  constexpr uint32_t ADC_REPORT_TICKS = ADC_REPORT_PERIOD_MS / LOOP_DELAY_MS;
#if defined(__MSPM0_HAS_MCAN__) && defined(CANFD0_BASE)
  constexpr uint32_t CAN_RETRY_PERIOD_MS = 1000;
  constexpr uint32_t CAN_RETRY_TICKS = CAN_RETRY_PERIOD_MS / LOOP_DELAY_MS;
#endif

  bool led_on = false;
  uint32_t tick = 0;
  uint32_t adc_last_mv = 0;
  uint32_t adc_min_mv = UINT32_MAX;
  uint32_t adc_max_mv = 0;
  uint64_t adc_sum_mv = 0;
  uint64_t adc_sum_sq_mv = 0;
  uint32_t adc_count = 0;
  uint64_t adc_win_sum_mv = 0;
  uint64_t adc_win_sum_sq_mv = 0;
  uint32_t adc_win_count = 0;
  uint32_t adc_win_id = 0;
  uint32_t adc_win_min_mv = UINT32_MAX;
  uint32_t adc_win_max_mv = 0;
  uint32_t adc_last_win_rms_mv = 0;
  uint32_t uart_read_submit_count = 0;
  uint32_t uart_read_done_count = 0;
  uint32_t uart_read_partial_count = 0;
  uint32_t uart_echo_frame_count = 0;
  uint32_t uart_echo_byte_count = 0;
  uint32_t uart_prev_timeout_count = 0;
#if defined(__MSPM0_HAS_MCAN__) && defined(CANFD0_BASE)
  bool can_loopback_reported = false;
  bool canfd_loopback_reported = false;
  LibXR::CAN::ClassicPack can_test_pack = {};
  can_test_pack.id = 0x123U;
  can_test_pack.type = LibXR::CAN::Type::STANDARD;
  can_test_pack.data[0] = 0xA5U;
  LibXR::FDCAN::FDPack canfd_test_pack = {};
  canfd_test_pack.id = 0x123U;
  canfd_test_pack.type = LibXR::CAN::Type::STANDARD;
  canfd_test_pack.len = 16U;
  canfd_test_pack.data[0] = 0x5AU;
  canfd_test_pack.data[15] = 0xC3U;
  if (can_ptr != nullptr && can_ptr->AddMessage(can_test_pack) == LibXR::ErrorCode::OK)
  {
    g_can_loopback_tx_count++;
  }
  if (can_ptr != nullptr && can_ptr->AddMessage(canfd_test_pack) == LibXR::ErrorCode::OK)
  {
    g_canfd_loopback_tx_count++;
  }
#endif

  LibXR::ReadOperation::OperationPollingStatus read_status =
      LibXR::ReadOperation::OperationPollingStatus::READY;
  LibXR::ReadOperation read_op(read_status);
  LibXR::WriteOperation echo_write_op;
  LibXR::WriteOperation heartbeat_write_op;

  LibXR::RawData read_data = {rx_echo_buffer, READ_REQ_TIMEOUT_MODE};

  while (true)
  {
    bool echoed_this_cycle = false;
    const LibXR::MSPM0UART::RxTimeoutMode RX_MODE = uart.GetRxTimeoutMode();
    const uint32_t READ_REQ =
        (RX_MODE == LibXR::MSPM0UART::RxTimeoutMode::BYTE_INTERRUPT)
            ? READ_REQ_BYTE_MODE
            : READ_REQ_TIMEOUT_MODE;
    const bool UART_TX_QUEUE_IDLE = (uart.write_port_->queue_info_->Size() == 0U);

    if (read_status == LibXR::ReadOperation::OperationPollingStatus::DONE)
    {
      uart_read_done_count++;
      const size_t read_size = uart.read_port_->read_size_;
      if (read_size < READ_REQ)
      {
        uart_read_partial_count++;
      }
      if (read_size > 0)
      {
        if (uart.Write({rx_echo_buffer, read_size}, echo_write_op) == LibXR::ErrorCode::OK)
        {
          uart_echo_frame_count++;
          uart_echo_byte_count += static_cast<uint32_t>(read_size);
        }
        echoed_this_cycle = true;
      }
      read_status = LibXR::ReadOperation::OperationPollingStatus::READY;
    }

    if (read_status == LibXR::ReadOperation::OperationPollingStatus::READY)
    {
      read_data.size_ = READ_REQ;
      (void)uart.Read(read_data, read_op);
      uart_read_submit_count++;
    }

    if (ENABLE_HEARTBEAT && !echoed_this_cycle && UART_TX_QUEUE_IDLE &&
        ((tick % HEARTBEAT_TICKS) == 0U))
    {
      const uint32_t TIMEOUT_COUNT = uart.GetRxTimeoutCount();
      const uint32_t TIMEOUT_DELTA = TIMEOUT_COUNT - uart_prev_timeout_count;
      uart_prev_timeout_count = TIMEOUT_COUNT;
      const size_t HB_LEN =
          BuildUartHeartbeatLine(uart_heartbeat, sizeof(uart_heartbeat), RX_MODE,
                                 READ_REQ, TIMEOUT_COUNT,
                                 uart.GetTimeoutInterruptEnabledMask(),
                                 uart.GetTimeoutInterruptMaskedStatus(),
                                 uart.GetTimeoutInterruptRawStatus(),
                                 uart.GetRxInterruptTimeoutValue(),
                                 uart.GetRxFifoThresholdValue(),
                                 uart_read_submit_count, uart_read_done_count,
                                 uart_read_partial_count, uart_echo_frame_count,
                                 uart_echo_byte_count, uart.GetRxDropCount(),
                                 TIMEOUT_DELTA);
      if (HB_LEN > 0U)
      {
        const size_t WRITE_SIZE =
            (HB_LEN < sizeof(uart_heartbeat)) ? HB_LEN : (sizeof(uart_heartbeat) - 1U);
        (void)uart.Write(
            {reinterpret_cast<const uint8_t*>(uart_heartbeat), WRITE_SIZE},
            heartbeat_write_op);
      }
    }

    if ((tick % ADC_SAMPLE_TICKS) == 0U)
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
      adc_sum_sq_mv +=
          static_cast<uint64_t>(adc_last_mv) * static_cast<uint64_t>(adc_last_mv);
      adc_count++;
      const uint32_t adc_avg_mv = static_cast<uint32_t>(adc_sum_mv / adc_count);
      const uint32_t adc_rms_mv =
          IsqrtU64(static_cast<uint64_t>(adc_sum_sq_mv / adc_count));

      if (adc_last_mv < adc_win_min_mv)
      {
        adc_win_min_mv = adc_last_mv;
      }
      if (adc_last_mv > adc_win_max_mv)
      {
        adc_win_max_mv = adc_last_mv;
      }
      adc_win_sum_mv += adc_last_mv;
      adc_win_sum_sq_mv +=
          static_cast<uint64_t>(adc_last_mv) * static_cast<uint64_t>(adc_last_mv);
      adc_win_count++;

      if (adc_win_count >= ADC_RMS_WINDOW_SIZE)
      {
        const uint32_t adc_win_avg_mv =
            static_cast<uint32_t>(adc_win_sum_mv / adc_win_count);
        adc_last_win_rms_mv =
            IsqrtU64(static_cast<uint64_t>(adc_win_sum_sq_mv / adc_win_count));
        adc_win_id++;

        g_adc_win_avg_mv = adc_win_avg_mv;
        g_adc_win_rms_mv = adc_last_win_rms_mv;
        g_adc_win_min_mv = adc_win_min_mv;
        g_adc_win_max_mv = adc_win_max_mv;
        g_adc_win_id = adc_win_id;

        adc_win_sum_mv = 0;
        adc_win_sum_sq_mv = 0;
        adc_win_count = 0;
        adc_win_min_mv = UINT32_MAX;
        adc_win_max_mv = 0;
      }

      g_adc_last_mv = adc_last_mv;
      g_adc_min_mv = adc_min_mv;
      g_adc_max_mv = adc_max_mv;
      g_adc_avg_mv = adc_avg_mv;
      g_adc_rms_mv = adc_rms_mv;
      g_adc_sample_count = adc_count;
      g_adc_win_fill_count = adc_win_count;
    }

    if (!echoed_this_cycle && UART_TX_QUEUE_IDLE && ((tick % ADC_REPORT_TICKS) == 0U))
    {
      const uint32_t adc_avg_mv =
          (adc_count > 0U) ? static_cast<uint32_t>(adc_sum_mv / adc_count) : 0U;
      const uint32_t adc_rms_mv =
          (adc_count > 0U)
              ? IsqrtU64(static_cast<uint64_t>(adc_sum_sq_mv / adc_count))
              : 0U;

      const size_t line_len =
          BuildAdcDiagLine(adc_diag, sizeof(adc_diag), adc_last_mv, adc_min_mv,
                           adc_max_mv, adc_avg_mv, adc_rms_mv, adc_count,
                           adc_last_win_rms_mv, adc_win_id, adc_win_count);

      if (line_len > 0U)
      {
        const size_t write_size =
            (line_len < sizeof(adc_diag)) ? line_len : (sizeof(adc_diag) - 1U);
        (void)uart.Write({reinterpret_cast<const uint8_t*>(adc_diag), write_size},
                         heartbeat_write_op);
      }
    }

#if defined(__MSPM0_HAS_MCAN__) && defined(CANFD0_BASE)
    if (!echoed_this_cycle && UART_TX_QUEUE_IDLE && !can_loopback_reported &&
        (g_can_loopback_rx_count > 0U))
    {
      const size_t line_len =
          BuildCanDiagLine(can_diag, sizeof(can_diag), g_can_loopback_tx_count,
                           g_can_loopback_rx_count, g_can_loopback_last_id,
                           g_can_loopback_last_type, g_can_loopback_last_data0);
      if (line_len > 0U)
      {
        const size_t write_size =
            (line_len < sizeof(can_diag)) ? line_len : (sizeof(can_diag) - 1U);
        (void)uart.Write({reinterpret_cast<const uint8_t*>(can_diag), write_size},
                         heartbeat_write_op);
      }
      can_loopback_reported = true;
    }

    if (!can_loopback_reported && ((tick % CAN_RETRY_TICKS) == 0U))
    {
      can_test_pack.data[0]++;
      if (can_ptr != nullptr && can_ptr->AddMessage(can_test_pack) == LibXR::ErrorCode::OK)
      {
        g_can_loopback_tx_count++;
      }

      canfd_test_pack.data[0]++;
      if (can_ptr != nullptr && can_ptr->AddMessage(canfd_test_pack) == LibXR::ErrorCode::OK)
      {
        g_canfd_loopback_tx_count++;
      }
    }

    if (!canfd_loopback_reported && (g_canfd_loopback_rx_count > 0U))
    {
      canfd_loopback_reported = true;
    }
#endif

    if ((tick % LED_TICKS) == 0U)
    {
      led_on = !led_on;
      (void)led_gpio.Write(led_on);
    }

    LibXR::Thread::Sleep(LOOP_DELAY_MS);
    tick++;
  }
}
