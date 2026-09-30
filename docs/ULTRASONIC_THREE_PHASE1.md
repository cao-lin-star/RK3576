# Three-ultrasonic observation stage (2026-09-20)

Additive 0x08 payload:36bytes, three records(front,left,right), each float32
distance_m,uint32 age_ms,uint16 sequence,uint8 status,uint8 reserved0, LE.
Status0=uninitialized,1=echo,2=no echo/out of range,3=fault. Legacy0x05 is unchanged.

New /ultrasonic/status JSON exposes status,age,sequence for all channels.
Range topics /range/ultrasonic_front_observe, /range/ultrasonic_left,
/range/ultrasonic_right publish each completed measurement; non-echo/fault
values are NaN, not claims of clear space. Age is subtracted from receipt
time (not a clock-synchronized MCU timestamp). Existing /range/ultrasonic
and its safety consumers are unchanged. New topics have NO costmap consumers.

URDF side poses are PROVISIONAL: x=.1556,y=+/- .1556,z=.24,yaw=+/-45deg.
Measure actual mounts before using them for safety or navigation.

Board build succeeded;6 protocol tests passed; xacro expansion passed.
Full ament suite is NOT all green:cpplint/uncrustify style failures remain.
9s live bridge check received181 US3 messages,zero payload/CRC errors,
no speed commands sent and zero wheel ticks. Timeout fault bit1 is expected
in this observation check without velocity heartbeats; no attempt was made
to clear it by commanding motion. Existing debug0x7F messages remain unparsed.

Board backup/build/test logs: /home/sky/us3-upgrade-20260920.
No autonomous task was started; mobile/Foxglove services restored idle.
Side glass/angle/corridor/crosstalk acceptance awaits installation.
Detailed wiring: G:/work/ROS2/Docs2/三超声波接线与静态验收_20260920.md
