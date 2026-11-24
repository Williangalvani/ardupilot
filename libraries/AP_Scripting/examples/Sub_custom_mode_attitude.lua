-- This script adds a custom mode to Sub that holds a pitch and heading chosen by the script
-- the pilot flies with the forward, lateral and throttle sticks
-- holding pitch needs a frame with pitch authority, such as the vectored 6DOF frame
local MODE_NUMBER = 101

-- Register as mode 101
local ATTITUDE_MODE_STATE = assert(vehicle:register_custom_mode(MODE_NUMBER, "Script Attitude", "SATT"))
ATTITUDE_MODE_STATE:allow_entry(true)

-- Get input channels
local pilot_forward = assert(rc:get_channel(assert(param:get("RCMAP_FORWARD"))))
local pilot_lateral = assert(rc:get_channel(assert(param:get("RCMAP_LATERAL"))))
local pilot_throttle = assert(rc:get_channel(assert(param:get("RCMAP_THROTTLE"))))

-- Pitch to hold, negative is nose down
local TARGET_PITCH_DEG = -20.0

-- Heading to hold, set to the heading on entry
local target_yaw_deg

local function init()
    target_yaw_deg = math.deg(ahrs:get_yaw_rad())
end

local function run()
    -- throttle is 0 to 1 with 0.5 for no vertical thrust
    local throttle = 0.5 + 0.5 * pilot_throttle:norm_input_dz()

    -- Hold level roll, the target pitch and heading with zero rates
    vehicle:set_target_angle_and_rate_and_throttle(0.0, TARGET_PITCH_DEG, target_yaw_deg, 0.0, 0.0, 0.0, throttle)

    -- Must be set after the angle target, forward and lateral thrust are from -1 to 1
    sub:set_target_forward_lateral(pilot_forward:norm_input_dz(), pilot_lateral:norm_input_dz())
end

-- Used to check for mode changes
local last_mode_number

local function update()
    local mode = vehicle:get_mode()

    if mode == MODE_NUMBER then
        if last_mode_number ~= MODE_NUMBER then
            -- First call after entering
            init()
        end
        run()
    end
    last_mode_number = mode

    -- Run at 20Hz
    return update, 50
end

return update()
