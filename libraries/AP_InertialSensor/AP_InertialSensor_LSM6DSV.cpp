/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for ST LSM6DSV IMU

  Datasheet: https://www.st.com/resource/en/datasheet/lsm6dsv.pdf
 */

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_LSM6DSV.h"

enum LSM6DSVRegister : uint8_t {
    LSM6DSV_REG_FUNC_CFG_ACCESS  = 0x01,
    LSM6DSV_REG_IF_CFG           = 0x03,
    LSM6DSV_REG_FIFO_CTRL1       = 0x07,
    LSM6DSV_REG_FIFO_CTRL2       = 0x08,
    LSM6DSV_REG_FIFO_CTRL3       = 0x09,
    LSM6DSV_REG_FIFO_CTRL4       = 0x0A,
    LSM6DSV_REG_INT1_CTRL        = 0x0D,
    LSM6DSV_REG_WHO_AM_I         = 0x0F,
    LSM6DSV_REG_CTRL1            = 0x10,
    LSM6DSV_REG_CTRL2            = 0x11,
    LSM6DSV_REG_CTRL3            = 0x12,
    LSM6DSV_REG_CTRL6            = 0x15,
    LSM6DSV_REG_CTRL8            = 0x17,
    LSM6DSV_REG_FIFO_STATUS1     = 0x1B,
    LSM6DSV_REG_FIFO_STATUS2     = 0x1C,
    LSM6DSV_REG_STATUS_REG       = 0x1E,
    LSM6DSV_REG_OUT_TEMP_L       = 0x20,
    LSM6DSV_REG_OUTX_L_G        = 0x22,
    LSM6DSV_REG_OUTX_L_A        = 0x28,
    LSM6DSV_REG_FIFO_DATA_OUT_TAG = 0x78,
};

#define LSM6DSV_WHOAMI_VALUE      0x70

// ODR values written to CTRL1/CTRL2 lower nibble
#define LSM6DSV_ODR_OFF           0x00
#define LSM6DSV_ODR_960HZ         0x09
#define LSM6DSV_ODR_1920HZ        0x0A
#define LSM6DSV_ODR_3840HZ        0x0B
#define LSM6DSV_ODR_7680HZ        0x0C

// gyro full scale in CTRL6[3:0]
#define LSM6DSV_GY_FS_2000DPS     0x04

// accel full scale in CTRL8[1:0]
#define LSM6DSV_XL_FS_16G         0x03

// FIFO batch rates in FIFO_CTRL3 (same encoding as ODR)
#define LSM6DSV_FIFO_BDR_1920HZ   0x0A

// FIFO mode in FIFO_CTRL4[2:0]
#define LSM6DSV_FIFO_MODE_STREAM   0x06

// FIFO tag sensor IDs (upper 5 bits of tag byte, shifted right by 3)
#define LSM6DSV_TAG_GY_NC          0x01
#define LSM6DSV_TAG_XL_NC          0x02
#define LSM6DSV_TAG_TEMPERATURE    0x03

// CTRL3 flags
#define LSM6DSV_CTRL3_SW_RESET     (1U << 0)
#define LSM6DSV_CTRL3_IF_INC       (1U << 2)
#define LSM6DSV_CTRL3_BDU          (1U << 6)

#define LSM6DSV_BACKEND_SAMPLE_RATE 1920
#define LSM6DSV_MAX_FIFO_SAMPLES    8

static const uint32_t BACKEND_PERIOD_US = 1000000UL / LSM6DSV_BACKEND_SAMPLE_RATE;

extern const AP_HAL::HAL& hal;

AP_InertialSensor_LSM6DSV::AP_InertialSensor_LSM6DSV(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                     enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_LSM6DSV::probe(AP_InertialSensor &imu,
                                 AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                 enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    auto sensor = NEW_NOTHROW AP_InertialSensor_LSM6DSV(imu, std::move(dev), rotation);
    if (!sensor || !sensor->hardware_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_InertialSensor_Backend *
AP_InertialSensor_LSM6DSV::probe(AP_InertialSensor &imu,
                                 AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                 enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    auto sensor = NEW_NOTHROW AP_InertialSensor_LSM6DSV(imu, std::move(dev), rotation);
    if (!sensor || !sensor->hardware_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_LSM6DSV::start()
{
    _dev->get_semaphore()->take_blocking();

    configure_accel();
    configure_gyro();
    configure_fifo();

    _dev->get_semaphore()->give();

    if (!_imu.register_accel(accel_instance, LSM6DSV_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSV)) ||
        !_imu.register_gyro(gyro_instance, LSM6DSV_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSV))) {
        return;
    }

    set_gyro_orientation(gyro_instance, _rotation);
    set_accel_orientation(accel_instance, _rotation);

    _periodic_handle = _dev->register_periodic_callback(
        BACKEND_PERIOD_US, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DSV::read_fifo, void));
}

bool AP_InertialSensor_LSM6DSV::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

void AP_InertialSensor_LSM6DSV::configure_accel()
{
    // CTRL1: ODR=1920Hz, high-performance mode (op_mode_xl=0)
    _dev->write_register(LSM6DSV_REG_CTRL1, LSM6DSV_ODR_1920HZ);

    // CTRL8: FS=16g
    _dev->write_register(LSM6DSV_REG_CTRL8, LSM6DSV_XL_FS_16G);
}

void AP_InertialSensor_LSM6DSV::configure_gyro()
{
    // CTRL2: ODR=1920Hz, high-performance mode (op_mode_g=0)
    _dev->write_register(LSM6DSV_REG_CTRL2, LSM6DSV_ODR_1920HZ);

    // CTRL6: FS=2000dps, LPF1 default bandwidth
    _dev->write_register(LSM6DSV_REG_CTRL6, LSM6DSV_GY_FS_2000DPS);
}

void AP_InertialSensor_LSM6DSV::configure_fifo()
{
    // FIFO_CTRL1: watermark = 0 (unused, we poll)
    _dev->write_register(LSM6DSV_REG_FIFO_CTRL1, 0x00);

    // FIFO_CTRL2: no compression, no stop-on-WTM
    _dev->write_register(LSM6DSV_REG_FIFO_CTRL2, 0x00);

    // FIFO_CTRL3: batch both accel and gyro at 1920Hz
    _dev->write_register(LSM6DSV_REG_FIFO_CTRL3,
                         (LSM6DSV_FIFO_BDR_1920HZ << 4) | LSM6DSV_FIFO_BDR_1920HZ);

    // FIFO_CTRL4: stream mode, no temperature or timestamp batching
    _dev->write_register(LSM6DSV_REG_FIFO_CTRL4, LSM6DSV_FIFO_MODE_STREAM);

    fifo_reset();
}

void AP_InertialSensor_LSM6DSV::fifo_reset()
{
    // set bypass mode to flush FIFO, then restore stream mode
    _dev->write_register(LSM6DSV_REG_FIFO_CTRL4, 0x00);
    _dev->write_register(LSM6DSV_REG_FIFO_CTRL4, LSM6DSV_FIFO_MODE_STREAM);

    notify_accel_fifo_reset(accel_instance);
    notify_gyro_fifo_reset(gyro_instance);
}

void AP_InertialSensor_LSM6DSV::read_fifo()
{
    // read FIFO level: 9-bit count across STATUS1/STATUS2
    uint8_t status[2];
    if (!_dev->read_registers(LSM6DSV_REG_FIFO_STATUS1, status, 2)) {
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
        return;
    }

    uint16_t fifo_count = status[0] | ((status[1] & 0x01) << 8);

    // check for FIFO overrun
    if (status[1] & (1U << 6)) {
        fifo_reset();
        return;
    }

    if (fifo_count == 0) {
        return;
    }

    if (fifo_count > LSM6DSV_MAX_FIFO_SAMPLES) {
        fifo_count = LSM6DSV_MAX_FIFO_SAMPLES;
    }

    // adjust the periodic callback to be synchronous with incoming data
    _dev->adjust_periodic_callback(_periodic_handle, BACKEND_PERIOD_US);

    // accel: 16-bit at ±16g
    const float accel_scale = (GRAVITY_MSS * 16.0f) / 32768.0f;
    // gyro: 16-bit at ±2000 dps
    const float gyro_scale = radians(2000.0f) / 32768.0f;

    for (uint16_t i = 0; i < fifo_count; i++) {
        uint8_t fifo_entry[7];
        if (!_dev->read_registers(LSM6DSV_REG_FIFO_DATA_OUT_TAG, fifo_entry, 7)) {
            _inc_accel_error_count(accel_instance);
            _inc_gyro_error_count(gyro_instance);
            return;
        }

        const uint8_t tag = (fifo_entry[0] >> 3) & 0x1F;
        const int16_t x = int16_t(uint16_t(fifo_entry[1] | (fifo_entry[2] << 8)));
        const int16_t y = int16_t(uint16_t(fifo_entry[3] | (fifo_entry[4] << 8)));
        const int16_t z = int16_t(uint16_t(fifo_entry[5] | (fifo_entry[6] << 8)));

        switch (tag) {
        case LSM6DSV_TAG_XL_NC: {
            Vector3f accel{float(x), float(y), float(z)};
            accel *= accel_scale;
            _rotate_and_correct_accel(accel_instance, accel);
            _notify_new_accel_raw_sample(accel_instance, accel);
            break;
        }
        case LSM6DSV_TAG_GY_NC: {
            Vector3f gyro{float(x), float(y), float(z)};
            gyro *= gyro_scale;
            _rotate_and_correct_gyro(gyro_instance, gyro);
            _notify_new_gyro_raw_sample(gyro_instance, gyro);
            break;
        }
        default:
            break;
        }
    }

    // read temperature periodically (~every 100 calls = ~50ms at 1920Hz)
    if (_temperature_counter++ >= 100) {
        _temperature_counter = 0;
        uint8_t tbuf[2];
        if (_dev->read_registers(LSM6DSV_REG_OUT_TEMP_L, tbuf, 2)) {
            int16_t raw_temp = int16_t(uint16_t(tbuf[0] | (tbuf[1] << 8)));
            float temp_degc = raw_temp / 256.0f + 25.0f;
            _publish_temperature(accel_instance, temp_degc);
        }
    }
}

bool AP_InertialSensor_LSM6DSV::hardware_init()
{
    hal.scheduler->delay(5);

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        _dev->set_read_flag(0x80);

        // dummy read to switch to SPI mode after power-up
        uint8_t dummy;
        _dev->read_registers(LSM6DSV_REG_WHO_AM_I, &dummy, 1);
        hal.scheduler->delay(1);
    }

    // verify chip ID
    uint8_t whoami = 0;
    if (!_dev->read_registers(LSM6DSV_REG_WHO_AM_I, &whoami, 1) ||
        whoami != LSM6DSV_WHOAMI_VALUE) {
        return false;
    }

    // software reset
    _dev->write_register(LSM6DSV_REG_CTRL3, LSM6DSV_CTRL3_SW_RESET);
    hal.scheduler->delay(1);

    // wait for reset to complete
    for (uint8_t i = 0; i < 10; i++) {
        uint8_t ctrl3;
        if (_dev->read_registers(LSM6DSV_REG_CTRL3, &ctrl3, 1) &&
            !(ctrl3 & LSM6DSV_CTRL3_SW_RESET)) {
            break;
        }
        hal.scheduler->delay(1);
    }

    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        // dummy read again after reset to re-enter SPI mode
        uint8_t dummy;
        _dev->read_registers(LSM6DSV_REG_WHO_AM_I, &dummy, 1);
        hal.scheduler->delay(1);
    }

    // enable auto-increment and BDU
    _dev->write_register(LSM6DSV_REG_CTRL3, LSM6DSV_CTRL3_IF_INC | LSM6DSV_CTRL3_BDU, true);

    // disable I2C/I3C when on SPI
    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        _dev->write_register(LSM6DSV_REG_IF_CFG, 0x01);
    }

    _dev->setup_checked_registers(3, _dev->bus_type() == AP_HAL::Device::BUS_TYPE_I2C ? 200 : 20);

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    return true;
}
