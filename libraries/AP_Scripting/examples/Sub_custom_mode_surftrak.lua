-- This script adds a custom mode to Sub that works like SURFTRAK, holding a distance above the seafloor
-- the throttle stick climbs or descends and moves the distance target, the yaw stick sets a yaw rate
-- the forward and lateral sticks are passed through as forward and lateral thrust
local MODE_NUMBER = 103

-- Register as mode 103
local SURFTRAK_MODE_STATE = assert(vehicle:register_custom_mode(MODE_NUMBER, "Script Surftrak", "SSTK"))

-- Get input channels
local pilot_yaw = assert(rc:get_channel(assert(param:get("RCMAP_YAW"))))
local pilot_throttle = assert(rc:get_channel(assert(param:get("RCMAP_THROTTLE"))))
local pilot_forward = assert(rc:get_channel(assert(param:get("RCMAP_FORWARD"))))
local pilot_lateral = assert(rc:get_channel(assert(param:get("RCMAP_LATERAL"))))

local DOWNWARD = 25 -- rangefinder orientation, ROTATION_PITCH_270
local KP = 1.0 -- climb rate in m/s per meter of distance error
local MAX_CLIMB_RATE = 0.5 -- m/s, also the pilot's climb rate at full throttle stick
local MAX_YAW_RATE_DEGS = 90.0 -- deg/s at full yaw stick
local DT = 0.05 -- update period in seconds

-- distance to hold is set from the first reading
local target_dist, target_yaw_deg, warned, last_report_ms

local function init()
    target_dist = nil
    target_yaw_deg = math.deg(ahrs:get_yaw_rad())
    warned = false
    last_report_ms = millis()
end

local function run()
    local healthy = rangefinder:has_data_orient(DOWNWARD)
    local dist = rangefinder:distance_orient(DOWNWARD)
    local climb_rate = pilot_throttle:norm_input_dz() * MAX_CLIMB_RATE

    if healthy then
        warned = false
        if climb_rate ~= 0 or target_dist == nil then
            -- the target follows the measured distance while the pilot climbs or descends
            target_dist = dist
        else
            -- too close to the bottom gives a positive error and a climb
            climb_rate = math.max(-MAX_CLIMB_RATE, math.min(MAX_CLIMB_RATE, KP * (target_dist - dist)))
        end
    elseif not warned then
        warned = true
        gcs:send_text(4, "SSTK: rangefinder unhealthy, holding depth")
    end

    -- hold level with a heading moved by the yaw stick
    local yaw_rate_degs = pilot_yaw:norm_input_dz() * MAX_YAW_RATE_DEGS
    target_yaw_deg = (target_yaw_deg + yaw_rate_degs * DT + 180.0) % 360.0 - 180.0
    vehicle:set_target_angle_and_climbrate(0.0, 0.0, target_yaw_deg, climb_rate, true, yaw_rate_degs)

    -- Must be set after the angle target, forward and lateral thrust are from -1 to 1
    sub:set_target_forward_lateral(pilot_forward:norm_input_dz(), pilot_lateral:norm_input_dz())

    if healthy and millis() - last_report_ms >= 2000 then
        last_report_ms = millis()
        gcs:send_text(6, string.format("SSTK target %.2f m rangefinder %.2f m", target_dist, dist))
    end
end

-- Used to check for mode changes
local last_mode_number

local function update()
    -- Only allow entry with a rangefinder reading
    SURFTRAK_MODE_STATE:allow_entry(rangefinder:has_data_orient(DOWNWARD))

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
