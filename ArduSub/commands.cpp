#include "Sub.h"

// checks if we should update ahrs/RTL home position from the EKF
void Sub::update_home_from_EKF()
{
    // exit immediately if home already set
    if (ahrs.home_is_set()) {
        return;
    }
    if (!set_home_to_current_location(false)) {
        // ignore this failure
    }
}

// set_home_to_current_location - set home to current GPS location
bool Sub::set_home_to_current_location(bool lock)
{
    // get current location from EKF
    Location temp_loc;
    if (!ahrs.get_location(temp_loc)) {
        return false;
    }

    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    // Make home always at the water's surface.
    // This allows disarming and arming again at depth.
    // This also ensures that mission items with relative altitude frame, are always
    // relative to the water's surface, whether in a high elevation lake, or at sea level.
    // Use the EKF vertical datum rather than GPS AMSL from get_location().
    postype_t posD;
    if (ahrs.get_relative_position_D_origin(posD)) {
        temp_loc.set_alt_cm(
            ekf_origin.alt - posD * 100.0 + barometer.get_altitude() * 100.0,
            Location::AltFrame::ABSOLUTE);
    } else {
        temp_loc.offset_up_m(barometer.get_altitude());
    }

    return set_home(temp_loc, lock);
}

// set_home - sets ahrs home (used for RTL) to specified location
//  returns true if home location set successfully
bool Sub::set_home(const Location& loc, bool lock)
{
    // check if EKF origin has been set
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    // set ahrs home (used for RTL)
    if (!ahrs.set_home(loc)) {
        return false;
    }

    // lock home position
    if (lock) {
        ahrs.lock_home();
    }

    // return success
    return true;
}
