-- This script adds a custom mode to Sub that works like STABILIZE
-- roll and pitch sticks set lean angles, the yaw stick sets a yaw rate and the throttle stick is direct throttle
-- the forward and lateral sticks are passed through as forward and lateral thrust
local MODE_NUMBER = 102

-- Register as mode 102
local STABILIZE_MODE_STATE = assert(vehicle:register_custom_mode(MODE_NUMBER, "Script Stabilize", "SSTB"))
STABILIZE_MODE_STATE:allow_entry(true)

-- Get input channels
local pilot_roll = assert(rc:get_channel(assert(param:get("RCMAP_ROLL"))))
local pilot_pitch = assert(rc:get_channel(assert(param:get("RCMAP_PITCH"))))
local pilot_yaw = assert(rc:get_channel(assert(param:get("RCMAP_YAW"))))
local pilot_throttle = assert(rc:get_channel(assert(param:get("RCMAP_THROTTLE"))))
local pilot_forward = assert(rc:get_channel(assert(param:get("RCMAP_FORWARD"))))
local pilot_lateral = assert(rc:get_channel(assert(param:get("RCMAP_LATERAL"))))

-- Lean angle at full stick, the same as STABILIZE
local ANGLE_MAX_DEG = assert(param:get("ATC_ANGLE_MAX"))
-- Yaw rate at full stick
local MAX_YAW_RATE_DEGS = 90.0
-- Update period in seconds
local DT = 0.05

-- Heading target, moved by the yaw stick
local target_yaw_deg

local function init()
    target_yaw_deg = math.deg(ahrs:get_yaw_rad())
end

local function run()
    local roll_deg = pilot_roll:norm_input_dz() * ANGLE_MAX_DEG
    local pitch_deg = pilot_pitch:norm_input_dz() * ANGLE_MAX_DEG

    -- move the heading target at the pilot's yaw rate and feed the rate forward
    local yaw_rate_degs = pilot_yaw:norm_input_dz() * MAX_YAW_RATE_DEGS
    if not arming:is_armed() then
        target_yaw_deg = math.deg(ahrs:get_yaw_rad())
    end
    target_yaw_deg = (target_yaw_deg + yaw_rate_degs * DT + 180.0) % 360.0 - 180.0

    -- throttle is 0 to 1 with 0.5 for no vertical thrust
    local throttle = 0.5 + 0.5 * pilot_throttle:norm_input()

    vehicle:set_target_angle_and_rate_and_throttle(roll_deg, pitch_deg, target_yaw_deg, 0.0, 0.0, yaw_rate_degs, throttle)

    -- Must be set after the angle target, forward and lateral thrust are from -1 to 1
    sub:set_target_forward_lateral(pilot_forward:norm_input(), pilot_lateral:norm_input())
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
