-- This script adds a custom mode to Sub that flies a body frame velocity from the pilot sticks
-- forward, lateral and throttle sticks set the velocity, yaw is handled by guided mode from the yaw stick
local MODE_NUMBER = 100

-- Register as mode 100
local VELOCITY_MODE_STATE = assert(vehicle:register_custom_mode(MODE_NUMBER, "Script Velocity", "SVEL"))

-- Get input channels
local pilot_forward = assert(rc:get_channel(assert(param:get("RCMAP_FORWARD"))))
local pilot_lateral = assert(rc:get_channel(assert(param:get("RCMAP_LATERAL"))))
local pilot_throttle = assert(rc:get_channel(assert(param:get("RCMAP_THROTTLE"))))

-- Speed at full stick deflection
local MAX_SPEED_XY = 1.0 -- m/s
local MAX_SPEED_Z = 0.5 -- m/s

local function run()
    -- body frame velocity, throttle stick up is up
    local forward = pilot_forward:norm_input_dz() * MAX_SPEED_XY
    local lateral = pilot_lateral:norm_input_dz() * MAX_SPEED_XY
    local up = pilot_throttle:norm_input_dz() * MAX_SPEED_Z

    -- rotate into north, east, down
    local yaw = ahrs:get_yaw_rad()
    local vel_ned = Vector3f()
    vel_ned:x(forward * math.cos(yaw) - lateral * math.sin(yaw))
    vel_ned:y(forward * math.sin(yaw) + lateral * math.cos(yaw))
    vel_ned:z(-up)

    vehicle:set_target_velocity_NED(vel_ned)
end

local function update()
    -- Only allow entry when armed
    VELOCITY_MODE_STATE:allow_entry(arming:is_armed())

    if vehicle:get_mode() == MODE_NUMBER then
        run()
    end

    -- Run at 20Hz
    return update, 50
end

return update()
