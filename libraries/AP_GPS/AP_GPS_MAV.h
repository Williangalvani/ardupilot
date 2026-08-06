/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//  Mavlink GPS driver which accepts gps position data from an external
//  companion computer
//
#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_MAV_ENABLED

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Device.h>

#include "AP_GPS.h"
#include "GPS_Backend.h"

class AP_GPS_MAV : public AP_GPS_Backend {
public:

    AP_GPS_MAV(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
        AP_GPS_Backend(_gps, _params, _state, _port)
    {
        set_bus_id(AP_HAL::Device::make_bus_id(
                       AP_HAL::Device::BUS_TYPE_UNKNOWN,
                       0,
                       state.instance,
                       uint8_t(DevType::MAV)));
    }

    bool read() override;

    static bool _detect(struct MAV_detect_state &state, uint8_t data);

    void handle_msg(const mavlink_message_t &msg) override;

    const char *name() const override { return "MAV"; }

private:
    bool _new_data;
    uint32_t first_week;
    JitterCorrection jitter{2000};
};

#endif  // AP_GPS_MAV_ENABLED
