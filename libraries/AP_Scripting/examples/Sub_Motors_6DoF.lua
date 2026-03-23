-- Replicate the BlueROV2 vectored frame via 6DoF scripting motor matrix
-- Motor # Roll  Pitch Yaw   Throttle Forward Lateral Reversible TestOrder

Motors_6DoF:add_motor(0,  0,     0,     1.0,  0,       -1.0,    1.0,  true, 1)
Motors_6DoF:add_motor(1,  0,     0,    -1.0,  0,       -1.0,   -1.0,  true, 2)
Motors_6DoF:add_motor(2,  0,     0,    -1.0,  0,        1.0,    1.0,  true, 3)
Motors_6DoF:add_motor(3,  0,     0,     1.0,  0,        1.0,   -1.0,  true, 4)
Motors_6DoF:add_motor(4,  1.0,   0,     0,   -1.0,      0,      0,    true, 5)
Motors_6DoF:add_motor(5, -1.0,   0,     0,   -1.0,      0,      0,    true, 6)

assert(Motors_6DoF:init(6), 'unable to setup 6 motors')

motors:set_frame_string("6DoF Sub vectored scripting")
gcs:send_text(6, "6DoF Sub vectored scripting")
