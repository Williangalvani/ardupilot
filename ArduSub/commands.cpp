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
    if (ahrs.get_location(temp_loc)) {
        // Home at the current horizontal location, on the surface (alt 0,
        // surface-relative; set_home() anchors it). Lets the vehicle disarm and
        // re-arm at depth and keeps relative-altitude mission items referenced
        // to the surface, in a high lake or at sea level alike.
        temp_loc.set_alt_cm(0, Location::AltFrame::ABSOLUTE);
        return set_home(temp_loc, lock);
    }
    return false;
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

    // ArduSub home altitude is relative to the water surface (0 = surface,
    // negative = below), not raw AMSL. Anchoring it to the baro-derived surface
    // keeps the reported depth and relative-altitude mission/RTL targets correct
    // despite the GPS altitude error in the EKF origin.
    Location home = loc;
    Location surface;
    if (ahrs.get_location(surface)) {
        // surface plus the commanded surface-relative altitude, above the vehicle
        surface.offset_up_m(-barometer.get_altitude() + loc.alt * 0.01f);
        home.set_alt_cm(surface.alt, surface.get_alt_frame());
    }

    // set ahrs home (used for RTL)
    if (!ahrs.set_home(home)) {
        return false;
    }

    // lock home position
    if (lock) {
        ahrs.lock_home();
    }

    // return success
    return true;
}
