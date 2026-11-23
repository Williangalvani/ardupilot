# Rover Motor Detect

This applet adds a MOTOR_DETECT test for skid-steer rovers and boats. It
works out whether the ThrottleLeft (73) and ThrottleRight (74) output
functions are swapped, and whether each motor is reversed. It can
optionally fix the SERVOn_FUNCTION and SERVOn_REVERSED parameters.

The test drives the vehicle in Guided mode, so it needs a position and
velocity estimate (normally a GPS fix). The vehicle moves and turns
during the test, so use a clear, open area.

# How It Works

1. Each motor function is pulsed forwards and then backwards on its own.
   The sign of the forward speed shows whether the motor that moved is
   reversed. The sign of the yaw rate, combined with the direction,
   shows which side of the vehicle that motor is on. A motor on the
   left side driving forwards turns the vehicle right. The two
   functions must be found on opposite sides, and each forwards pulse
   must give the opposite response to its backwards pulse.
2. The vehicle is then driven forwards, backwards, and turned right and
   left. Each response is compared with what the detected wiring
   predicts.
3. If MDET_APPLY is 1 and a fix is needed, the applet swaps the
   SERVOn_FUNCTION values and toggles SERVOn_REVERSED as needed. It then
   repeats step 2 and requires that forward drives forward, backwards
   drives backwards, and steering right turns right. The changes are
   saved only if this check passes. Otherwise they are reverted.

The vehicle is brought to rest before each pulse. A weak or inconsistent
response ends the test as inconclusive and leaves all parameters
unchanged.

# Parameters

- MDET_START: set to 1 to start the test, or back to 0 to abort. The
  applet sets this to 0 when the test finishes.
- MDET_APPLY: 0 (default) reports only. 1 applies the fix, verifies it,
  and saves it.
- MDET_PWR: motor output for each pulse, as a fraction of full output
  (default 0.4).
- MDET_PULSE: duration of each pulse in seconds (default 1.5). Keep
  this below CRASH_TIMEOUT if FS_CRASH_CHECK is enabled.
- MDET_SPD_MIN: minimum mean speed in m/s for a response to count
  (default 0.2).
- MDET_YAW_MIN: minimum mean yaw rate in deg/s for a response to count
  (default 10).

# MAVLink Command

The test can also be started with COMMAND_LONG MAV_CMD_USER_1 (31010)
with param1 = 1, and aborted with param1 = 0.

# Usage

1. Copy rover-motor-detect.lua to the APM/scripts directory on the SD
   card.
2. Copy the modules/MAVLink directory from libraries/AP_Scripting to
   APM/scripts/modules/MAVLink.
3. Set SCR_ENABLE = 1 and reboot.
4. Place the vehicle in an open area and wait for a GPS fix.
5. Arm the vehicle in any mode, for example Manual or Hold.
6. Set MDET_START = 1, or send MAV_CMD_USER_1 with param1 = 1.
7. Watch the GCS messages. The test ends by returning the vehicle to
   the mode it was in before the test started.

Changing mode or disarming at any time aborts the test immediately and
reverts any parameter changes that have not been verified.

# Messages

- "MDET: swap:yes SERVO1:rev SERVO3:ok" reports the detected wiring.
  The output functions are swapped and the motor on SERVO1 is reversed.
- "MDET: verified, no change needed": the wiring is correct.
- "MDET: consistent, set MDET_APPLY=1 to fix": a fix is needed and the
  verification drive agreed with the detected wiring.
- "MDET: fixed and verified, saved": the fix was applied and forward
  now drives forward.
- "MDET: verify after fix failed": the fix did not give correct
  driving, and the parameters were reverted.
- "MDET: inconclusive: ...": the response was too weak or inconsistent.
  Try a higher MDET_PWR or MDET_PULSE, or check the sensors.

# Limitations

- Requires exactly one ThrottleLeft and one ThrottleRight output.
  Omni frames and Motor1..4 outputs are not supported.
- With MOT_PWM_TYPE = BrushedWithRelay, the direction relays follow the
  output function. A detected swap must be fixed by hand, including
  the relays.
- The applet relies on the AHRS yaw rate and velocity estimate. An
  incorrect AHRS_ORIENTATION gives wrong results.
