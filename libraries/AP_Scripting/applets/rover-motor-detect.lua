--[[
  Rover Motor Detect (MOTOR_DETECT)

  For skid-steer rovers, determines whether the ThrottleLeft and
  ThrottleRight output functions are swapped and whether each motor is
  reversed. Optionally fixes SERVOn_FUNCTION / SERVOn_REVERSED and then
  checks that forward drives forward and steering right turns right.

  Requires a position/velocity estimate (GPS) as the test drives the
  rover in Guided mode.

  See rover-motor-detect.md for instructions
--]]

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil

local mavlink_msgs = require("MAVLink/mavlink_msgs")

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local MAV_RESULT = {ACCEPTED=0, TEMPORARILY_REJECTED=1, DENIED=2, UNSUPPORTED=3, FAILED=4}

local MAV_CMD_USER_1 = 31010
local MODE_GUIDED = 15
local K_THROTTLE_LEFT = 73
local K_THROTTLE_RIGHT = 74
local MOT_PWM_TYPE_BRUSHED_WITH_RELAY = 3

local UPDATE_RUN_MS = 20
local UPDATE_IDLE_MS = 100
local SAMPLE_START = 0.4            -- ignore the first part of each pulse while motors spin up
local SETTLE_HOLD_MS = 500          -- vehicle must be still for this long before a pulse
local SETTLE_TIMEOUT_MS = 10000
local APPLY_WAIT_MS = 2500          -- SRV_Channels picks up function changes at 1Hz

local PARAM_TABLE_KEY = 63
local PARAM_TABLE_PREFIX = "MDET_"

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), "MDET: could not add param table")

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("MDET: could not add param %s", name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: MDET_START
  // @DisplayName: Motor detect start
  // @Description: Set to 1 to start the motor detection test. The vehicle must be armed and have a position estimate. Set back to 0 to abort. The script sets this to 0 when the test finishes.
  // @Values: 0:Idle,1:Start
  // @User: Standard
--]]
local MDET_START = bind_add_param("START", 1, 0)

--[[
  // @Param: MDET_APPLY
  // @DisplayName: Motor detect apply
  // @Description: When enabled, swaps SERVOn_FUNCTION and toggles SERVOn_REVERSED as needed, then verifies that forward drives forward and steering right turns right. Changes are saved only if that verification passes, otherwise they are reverted.
  // @Values: 0:Report only,1:Apply and verify
  // @User: Standard
--]]
local MDET_APPLY = bind_add_param("APPLY", 2, 0)

--[[
  // @Param: MDET_PWR
  // @DisplayName: Motor detect power
  // @Description: Motor output used for each test pulse, as a fraction of full output
  // @Range: 0.1 1
  // @User: Standard
--]]
local MDET_PWR = bind_add_param("PWR", 3, 0.4)

--[[
  // @Param: MDET_PULSE
  // @DisplayName: Motor detect pulse time
  // @Description: Duration of each test pulse. Keep this below CRASH_TIMEOUT if the crash check is enabled.
  // @Units: s
  // @Range: 0.5 3
  // @User: Standard
--]]
local MDET_PULSE = bind_add_param("PULSE", 4, 1.5)

--[[
  // @Param: MDET_SPD_MIN
  // @DisplayName: Motor detect minimum speed
  // @Description: Minimum mean forward or backward speed during a pulse for the response to count. Weaker responses make the test inconclusive.
  // @Units: m/s
  // @Range: 0.05 1
  // @User: Advanced
--]]
local MDET_SPD_MIN = bind_add_param("SPD_MIN", 5, 0.2)

--[[
  // @Param: MDET_YAW_MIN
  // @DisplayName: Motor detect minimum yaw rate
  // @Description: Minimum mean yaw rate during a pulse for the response to count. Weaker responses make the test inconclusive.
  // @Units: deg/s
  // @Range: 2 45
  // @User: Advanced
--]]
local MDET_YAW_MIN = bind_add_param("YAW_MIN", 6, 10)

local COMMAND_LONG_ID = mavlink_msgs.get_msgid("COMMAND_LONG")
local msg_map = {}
msg_map[COMMAND_LONG_ID] = "COMMAND_LONG"

mavlink:init(4, 1)
mavlink:register_rx_msgid(COMMAND_LONG_ID)
mavlink:block_command(MAV_CMD_USER_1)

local run = nil

local function gcs_msg(severity, fmt, ...)
    gcs:send_text(severity, "MDET: " .. string.format(fmt, ...))
end

local function sign_of(value, threshold)
    if value > threshold then
        return 1
    end
    if value < -threshold then
        return -1
    end
    return 0
end

local function body_forward_speed()
    local vel = ahrs:get_velocity_NED()
    if not vel then
        return nil
    end
    local yaw = ahrs:get_yaw_rad()
    return vel:x() * math.cos(yaw) + vel:y() * math.sin(yaw)
end

local function yaw_rate_dps()
    return math.deg(ahrs:get_gyro():z())
end

local function servo_param(chan, suffix)
    return string.format("SERVO%d_%s", chan, suffix)
end

-- returns the output channel numbers (1-based) of ThrottleLeft and ThrottleRight
local function find_motor_channels()
    local left = nil
    local right = nil
    for i = 1, 32 do
        local func = param:get(servo_param(i, "FUNCTION"))
        if func == nil then
            break
        end
        if func == K_THROTTLE_LEFT then
            if left then
                return nil, nil, "more than one ThrottleLeft output"
            end
            left = i
        elseif func == K_THROTTLE_RIGHT then
            if right then
                return nil, nil, "more than one ThrottleRight output"
            end
            right = i
        end
    end
    if not left or not right then
        return nil, nil, "needs one ThrottleLeft and one ThrottleRight output"
    end
    return left, right, nil
end

--[[
  A model describes the current wiring:
    dir_l: +1 if the motor driven by ThrottleLeft turns forward for a forward demand, -1 if reversed
    dir_r: same for ThrottleRight
    swapped: true if ThrottleLeft drives the motor on the right side
  predict() returns the expected sign of forward speed and yaw rate for
  demands l and r on ThrottleLeft and ThrottleRight. 0 means no clear expectation.
--]]
local IDENTITY_MODEL = {dir_l = 1, dir_r = 1, swapped = false}

local function predict(model, l, r)
    local phys_left = l * model.dir_l
    local phys_right = r * model.dir_r
    if model.swapped then
        phys_left, phys_right = phys_right, phys_left
    end
    return sign_of(phys_left + phys_right, 0.01), sign_of(phys_left - phys_right, 0.01)
end

local function test_power()
    return math.max(0.1, math.min(1.0, MDET_PWR:get()))
end

local function pulse_ms()
    return math.max(0.5, math.min(3.0, MDET_PULSE:get())) * 1000
end

-- the skid-steer mixer gives ThrottleLeft = throttle + steering, ThrottleRight = throttle - steering
local function command(l, r)
    return vehicle:set_steering_and_throttle((l - r) * 0.5, (l + r) * 0.5)
end

local function push(step)
    run.steps[#run.steps + 1] = step
end

local function push_pulse(name, l, r)
    push({kind = "settle"})
    push({kind = "pulse", name = name, l = l, r = r})
end

local function revert_params()
    if not run.saved then
        return
    end
    for i = #run.saved, 1, -1 do
        param:set(run.saved[i].name, run.saved[i].value)
    end
    run.saved = nil
    gcs_msg(MAV_SEVERITY.WARNING, "parameter changes reverted")
end

local function finish(ok, fmt, ...)
    if not ok then
        revert_params()
    end
    gcs_msg(ok and MAV_SEVERITY.NOTICE or MAV_SEVERITY.WARNING, fmt, ...)
    if vehicle:get_mode() == MODE_GUIDED then
        command(0, 0)
        if run.prev_mode ~= MODE_GUIDED then
            vehicle:set_mode(run.prev_mode)
        end
    end
    run = nil
    MDET_START:set(0)
end

local function abort(reason)
    finish(false, "MOTOR_DETECT aborted: %s", reason)
end

-- check measured pulses against the expectation from a model
local function check_pulses(model, names)
    local spd_min = MDET_SPD_MIN:get()
    local yaw_min = MDET_YAW_MIN:get()
    for _, name in ipairs(names) do
        local res = run.results[name]
        local exp_fwd, exp_yaw = predict(model, res.l, res.r)
        if exp_fwd ~= 0 and sign_of(res.fwd, spd_min) ~= exp_fwd then
            return false, string.format("%s speed %.2f", name, res.fwd)
        end
        if exp_yaw ~= 0 and sign_of(res.yaw, yaw_min) ~= exp_yaw then
            return false, string.format("%s yaw %.1f", name, res.yaw)
        end
    end
    return true, nil
end

local VERIFY_PULSES = {"fwd", "back", "right", "left"}

local function push_verify(prefix)
    local pwr = test_power()
    push_pulse(prefix .. "fwd", pwr, pwr)
    push_pulse(prefix .. "back", -pwr, -pwr)
    push_pulse(prefix .. "right", pwr, -pwr)
    push_pulse(prefix .. "left", -pwr, pwr)
    push({kind = "settle"})
end

local function verify_names(prefix)
    local names = {}
    for i, name in ipairs(VERIFY_PULSES) do
        names[i] = prefix .. name
    end
    return names
end

-- each function is pulsed forwards then backwards; returns motor direction and side (+1 left, -1 right)
local function classify(pos, neg)
    local spd_min = MDET_SPD_MIN:get()
    local yaw_min = MDET_YAW_MIN:get()
    local f_pos = sign_of(pos.fwd, spd_min)
    local y_pos = sign_of(pos.yaw, yaw_min)
    if f_pos == 0 or y_pos == 0 then
        return nil, string.format("weak response (%.2fm/s %.1fdeg/s)", pos.fwd, pos.yaw)
    end
    if sign_of(neg.fwd, spd_min) ~= -f_pos or sign_of(neg.yaw, yaw_min) ~= -y_pos then
        return nil, "forward and reverse pulses disagree"
    end
    -- a motor on the left side driving forwards turns the rover right (positive yaw)
    return {dir = f_pos, side = f_pos * y_pos}, nil
end

local function apply_fixes(model)
    run.saved = {}
    local function set(name, value)
        run.saved[#run.saved + 1] = {name = name, value = param:get(name)}
        param:set(name, value)
    end
    local function toggle_reversed(chan)
        local name = servo_param(chan, "REVERSED")
        set(name, param:get(name) == 1 and 0 or 1)
    end
    if model.dir_l < 0 then
        toggle_reversed(run.chan_l)
    end
    if model.dir_r < 0 then
        toggle_reversed(run.chan_r)
    end
    if model.swapped then
        set(servo_param(run.chan_l, "FUNCTION"), K_THROTTLE_RIGHT)
        set(servo_param(run.chan_r, "FUNCTION"), K_THROTTLE_LEFT)
    end
    gcs_msg(MAV_SEVERITY.INFO, "fixes applied, verifying")
end

local function check_applied()
    local ok, why = check_pulses(IDENTITY_MODEL, verify_names("fix_"))
    if not ok then
        finish(false, "verify after fix failed: %s", why)
        return
    end
    for _, entry in ipairs(run.saved) do
        param:set_and_save(entry.name, param:get(entry.name))
    end
    run.saved = nil
    finish(true, "fixed and verified, saved")
end

local function check_consistent()
    local ok, why = check_pulses(run.model, verify_names(""))
    if not ok then
        finish(false, "inconclusive: %s", why)
        return
    end
    if run.needs_fix then
        if not run.can_apply then
            finish(true, "consistent, fix by hand")
        elseif MDET_APPLY:get() == 0 then
            finish(true, "consistent, set MDET_APPLY=1 to fix")
        else
            apply_fixes(run.model)
            push({kind = "wait", ms = APPLY_WAIT_MS})
            push_verify("fix_")
            push({kind = "call", fn = check_applied})
        end
    else
        finish(true, "verified, no change needed")
    end
end

local function identify()
    local left, why_l = classify(run.results.Lpos, run.results.Lneg)
    if not left then
        finish(false, "inconclusive: ThrottleLeft %s", why_l)
        return
    end
    local right, why_r = classify(run.results.Rpos, run.results.Rneg)
    if not right then
        finish(false, "inconclusive: ThrottleRight %s", why_r)
        return
    end
    if left.side == right.side then
        finish(false, "inconclusive: both functions drive the %s side", left.side > 0 and "left" or "right")
        return
    end

    run.model = {dir_l = left.dir, dir_r = right.dir, swapped = (left.side < 0)}
    run.needs_fix = run.model.swapped or run.model.dir_l < 0 or run.model.dir_r < 0
    run.can_apply = not (run.model.swapped and param:get("MOT_PWM_TYPE") == MOT_PWM_TYPE_BRUSHED_WITH_RELAY)

    gcs_msg(MAV_SEVERITY.NOTICE, "swap:%s SERVO%d:%s SERVO%d:%s",
            run.model.swapped and "yes" or "no",
            run.chan_l, run.model.dir_l < 0 and "rev" or "ok",
            run.chan_r, run.model.dir_r < 0 and "rev" or "ok")
    if not run.can_apply then
        gcs_msg(MAV_SEVERITY.WARNING, "brushed relay: swap outputs and relays by hand")
    end

    -- confirm the model by predicting the response to normal driving commands
    push_verify("")
    push({kind = "call", fn = check_consistent})
end

local function start_run()
    if run then
        gcs_msg(MAV_SEVERITY.WARNING, "already running")
        return false
    end
    if not arming:is_armed() then
        gcs_msg(MAV_SEVERITY.WARNING, "arm the vehicle first")
        return false
    end
    if not ahrs:get_velocity_NED() then
        gcs_msg(MAV_SEVERITY.WARNING, "needs a velocity estimate")
        return false
    end
    local chan_l, chan_r, why = find_motor_channels()
    if not chan_l then
        gcs_msg(MAV_SEVERITY.WARNING, "%s", why)
        return false
    end
    local prev_mode = vehicle:get_mode()
    if not vehicle:set_mode(MODE_GUIDED) then
        gcs_msg(MAV_SEVERITY.WARNING, "could not enter Guided")
        return false
    end

    run = {
        prev_mode = prev_mode,
        chan_l = chan_l,
        chan_r = chan_r,
        steps = {},
        idx = 1,
        step_start_ms = nil,
        results = {},
    }
    MDET_START:set(1)

    local pwr = test_power()
    push_pulse("Lpos", pwr, 0)
    push_pulse("Lneg", -pwr, 0)
    push_pulse("Rpos", 0, pwr)
    push_pulse("Rneg", 0, -pwr)
    push({kind = "settle"})
    push({kind = "call", fn = identify})

    gcs_msg(MAV_SEVERITY.NOTICE, "MOTOR_DETECT started")
    return true
end

local function next_step()
    run.idx = run.idx + 1
    run.step_start_ms = nil
end

local function run_step(now)
    local step = run.steps[run.idx]
    if step == nil then
        abort("no steps left")
        return
    end
    if run.step_start_ms == nil then
        run.step_start_ms = now
        run.still_since_ms = nil
        run.acc = {fwd = 0, yaw = 0, n = 0}
    end
    local elapsed = now - run.step_start_ms

    if step.kind == "call" then
        next_step()
        step.fn()
        return
    end

    local fwd = body_forward_speed()
    if fwd == nil then
        abort("lost velocity estimate")
        return
    end
    local yaw = yaw_rate_dps()

    if step.kind == "pulse" then
        local duration_ms = pulse_ms()
        if elapsed < duration_ms then
            if not command(step.l, step.r) then
                abort("steering and throttle rejected")
                return
            end
            if elapsed >= duration_ms * SAMPLE_START then
                run.acc.fwd = run.acc.fwd + fwd
                run.acc.yaw = run.acc.yaw + yaw
                run.acc.n = run.acc.n + 1
            end
            return
        end
        command(0, 0)
        local n = math.max(run.acc.n, 1)
        run.results[step.name] = {l = step.l, r = step.r, fwd = run.acc.fwd / n, yaw = run.acc.yaw / n}
        next_step()
        return
    end

    command(0, 0)

    if step.kind == "wait" then
        if elapsed >= step.ms then
            next_step()
        end
        return
    end

    if step.kind == "settle" then
        local still = math.abs(fwd) < 0.5 * MDET_SPD_MIN:get() and math.abs(yaw) < 0.5 * MDET_YAW_MIN:get()
        if not still then
            run.still_since_ms = nil
        elseif run.still_since_ms == nil then
            run.still_since_ms = now
        end
        if run.still_since_ms ~= nil and now - run.still_since_ms >= SETTLE_HOLD_MS then
            next_step()
        elseif elapsed > SETTLE_TIMEOUT_MS then
            abort("vehicle did not come to rest")
        end
        return
    end
end

local function send_ack(chan, cmd, result)
    local ack = {
        command = cmd.command,
        result = result,
        progress = 0,
        result_param2 = 0,
        target_system = cmd.sysid,
        target_component = cmd.compid,
    }
    local msgid, payload = mavlink_msgs.encode("COMMAND_ACK", ack)
    mavlink:send_chan(chan, msgid, payload)
end

local function handle_mavlink()
    local msg, chan = mavlink:receive_chan()
    if msg == nil then
        return
    end
    local cmd = mavlink_msgs.decode(msg, msg_map)
    if cmd == nil or cmd.command ~= MAV_CMD_USER_1 then
        return
    end
    local result
    if cmd.param1 >= 0.5 then
        result = start_run() and MAV_RESULT.ACCEPTED or MAV_RESULT.FAILED
    else
        if run then
            abort("stopped by command")
        end
        result = MAV_RESULT.ACCEPTED
    end
    send_ack(chan, cmd, result)
end

local function update()
    handle_mavlink()

    if run == nil then
        if MDET_START:get() >= 1 and not start_run() then
            MDET_START:set(0)
        end
        return update, UPDATE_IDLE_MS
    end

    if MDET_START:get() < 1 then
        abort("MDET_START set to 0")
    elseif not arming:is_armed() then
        abort("disarmed")
    elseif vehicle:get_mode() ~= MODE_GUIDED then
        abort("mode changed")
    else
        run_step(millis():tofloat())
    end
    return update, UPDATE_RUN_MS
end

-- clear a start request saved from a previous boot
MDET_START:set(0)
gcs_msg(MAV_SEVERITY.INFO, "loaded")

return update()
