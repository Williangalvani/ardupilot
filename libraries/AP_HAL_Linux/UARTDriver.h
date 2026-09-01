#pragma once

#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/utility/RingBuffer.h>

#include "AP_HAL_Linux.h"
#include "SerialDevice.h"
#include "Semaphores.h"

namespace Linux {

class UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(bool default_console);

    static UARTDriver *from(AP_HAL::UARTDriver *uart) {
        return static_cast<UARTDriver*>(uart);
    }

    bool is_initialized() override;
    bool tx_pending() override;

    uint32_t txspace() override;

    void set_device_path(const char *path);

    bool _write_pending_bytes(void);
    virtual void _timer_tick(void) override;

    virtual enum flow_control get_flow_control(void) override
    {
        return _device->get_flow_control();
    }

    void configure_parity(uint8_t v) override;

    virtual void set_flow_control(enum flow_control flow_control_setting) override
   {
       _device->set_flow_control(flow_control_setting);
   }

    /*
      return timestamp estimate in microseconds for when the start of
      a nbytes packet arrived on the uart. This should be treated as a
      time constraint, not an exact time. It is guaranteed that the
      packet did not start being received after this time, but it
      could have been in a system buffer before the returned time.

      This takes account of the baudrate of the link. For transports
      that have no baudrate (such as USB) the time estimate may be
      less accurate.

      A return value of zero means the HAL does not support this API
     */
    uint64_t receive_time_constraint_us(uint16_t nbytes) override;

    uint32_t bw_in_bytes_per_second() const override;

    virtual uint32_t get_baud_rate() const override { return _baudrate; }

#if HAL_UART_STATS_ENABLED
    void uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms) override;
#endif

private:
    AP_HAL::OwnPtr<SerialDevice> _device;
    bool _console;
    volatile bool _in_timer;
    uint16_t _base_port;
    uint32_t _baudrate;
    char *_ip;
    char *_flag;
    bool _connected; // true if a client has connected
    bool _packetise; // true if writes should try to be on mavlink boundaries

    void _allocate_buffers(uint16_t rxS, uint16_t txS);
    void _deallocate_buffers();

    AP_HAL::OwnPtr<SerialDevice> _parseDevicePath(const char *arg);

    // timestamp for receiving data on the UART, avoiding a lock
    uint64_t _receive_timestamp[2];
    uint8_t _receive_timestamp_idx;

protected:
    const char *device_path;
    volatile bool _initialised;

    // we use in-task ring buffers to reduce the system call cost
    // of ::read() and ::write() in the main loop
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};

    virtual int _write_fd(const uint8_t *buf, uint16_t n);
    virtual int _read_fd(uint8_t *buf, uint16_t n);

    Linux::Semaphore _write_mutex;

    bool _discard_input() override;
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override WARN_IF_UNUSED;

#if HAL_UART_STATS_ENABLED
    uint32_t get_total_tx_bytes() const override { return _tx_stats_bytes; }
    uint32_t get_total_rx_bytes() const override { return _rx_stats_bytes; }
    uint32_t get_total_dropped_rx_bytes() const override { return _rx_stats_dropped_bytes; }

    void _update_serial_error_stats();

    uint32_t _tx_stats_bytes;
    uint32_t _rx_stats_bytes;
    uint32_t _rx_stats_dropped_bytes;
    uint32_t _rx_stats_framing_errors;
    uint32_t _rx_stats_overrun_errors;
    uint32_t _rx_stats_parity_errors;
    uint32_t _rx_stats_buf_overrun_errors;
    uint32_t _last_icount_overrun;
    uint32_t _last_icount_buf_overrun;
    uint32_t _last_icount_ms;
#endif
};

}
