#!/bin/bash
# End-to-end Gazebo SAR validation: real camera → YOLO → geo_localiser →
# sar_orchestrator → SITL flight → DROP → RTL → DONE.
#
# Validated 2026-05-18: produced the full state sequence above with the
# drone physically flying 11.9 m N (= 15 m survivor − 4 m drop offset)
# on Gazebo-rendered pixels. See docs/sim_evidence/.
#
# Layout (all on localhost):
#   ArduCopter SITL  --model JSON  serial0=tcp:5760 (mavlink_bridge)
#                                  serial1=tcp:5762 (pilot)
#                                  JSON FDM         udp:9002 (gz plugin)
#   Gazebo Harmonic  + ardupilot_gazebo plugin
#   ROS 2 stack      mavlink_bridge + ros_gz_bridge (camera) +
#                    sim_gimbal_state + person_detector (real YOLO) +
#                    geo_localiser + sar_orchestrator + sim_payload
#   Pilot            connects on 5762 (own MAVLink stream) so it doesn't
#                    fight mavlink_bridge for 5760
#
# Prereqs (one-time, see SIMULATION.md):
#   ardupilot_gazebo plugin built at ~/ardupilot_gazebo/build
#   GZ_SIM_RESOURCE_PATH must include the plugin's models + this repo's
#   gazebo/ directory at runtime
#   In ~/ardupilot_gazebo/models/iris_with_gimbal/model.sdf, comment out
#   the ArduPilot plugin's gimbal channel-8/9/10 control blocks (otherwise
#   servo PWM defaults overwrite /gimbal/cmd_pitch at 50 Hz)
set +e

# Kill any leftover sim processes (won't touch the running shell).
python3 - <<'EOF'
import os, glob
me=os.getpid()
pat=('arducopter','gz sim','gz-sim','ros2 launch','sitl_gz_pilot',
     'mavlink_bridge','sar_orchestrator','sim_payload','sim_gimbal_state',
     'person_detector','geo_localiser','parameter_bridge','ros_gz_bridge',
     'sitl_gazebo','tcp_pty_bridge')
for d in glob.glob('/proc/[0-9]*'):
    pid=int(d.split('/')[-1])
    if pid==me: continue
    try: cl=open(d+'/cmdline','rb').read().decode('utf-8','replace')
    except OSError: continue
    if any(p in cl for p in pat) and '/bin/bash' not in cl[:20]:
        try: os.kill(pid,9)
        except OSError: pass
EOF
sleep 3

# (1) SITL — explicit SERIAL1 on 5762 for the pilot.
mkdir -p ~/sitl_tmp && cd ~/sitl_tmp && rm -f eeprom.bin
setsid ~/ardupilot/build/sitl/bin/arducopter -w --model JSON --slave 0 \
  --serial1 tcp:5762 \
  --defaults ~/ardupilot/Tools/autotest/default_params/copter.parm \
  -I0 --home -35.363262,149.165237,584,0 > ~/sitl_run.log 2>&1 < /dev/null & disown
echo "[1] SITL up, pid $!"
sleep 4

# (2) Gazebo with the ardupilot_gazebo plugin.
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH"
WORLD="$HOME/Drone/ros2_ws/src/drone_vision/gazebo/sar_world.sdf"
setsid gz sim -s -r --headless-rendering "$WORLD" > ~/gz_run.log 2>&1 < /dev/null & disown
echo "[2] gz sim up, pid $!"
sleep 14

# (3) Wake SITL with a dummy MAVLink on 5760 — forces SERIAL1/2 bind.
# Also set params we need BEFORE mavlink_bridge connects.
~/sitl-venv/bin/python - <<'PY' 2>&1 | sed 's/^/[3] /'
from pymavlink import mavutil
import time
m = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
hb = m.wait_heartbeat(timeout=20)
print(f"wake hb={hb is not None}")
m.target_component = 1
def sp(n, v):
    m.mav.param_set_send(m.target_system, m.target_component, n.encode(),
                         float(v), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
# Loose SITL settings for the demo.
sp('ARMING_CHECK', 0); sp('FENCE_ENABLE', 0); sp('FS_GCS_ENABLE', 0)
# SERIAL1 = MAVLink2 GCS link so the pilot can use it.
sp('SERIAL1_PROTOCOL', 2); sp('SERIAL1_BAUD', 921)
time.sleep(2)
m.close()
PY

# (4) ROS 2 stack — must be sourced.
source /opt/ros/jazzy/setup.bash
source ~/Drone/ros2_ws/install/setup.bash
setsid ros2 launch drone_vision sitl_gazebo.launch.py > ~/gz_ros.log 2>&1 < /dev/null & disown
echo "[4] ROS stack up, pid $!"
sleep 18

# (5) Force gimbal to nadir. We send +1.57 because gimbal_small_3d's
# pose chain is rotated so positive pitch_joint points the camera down.
# (Negative would point up at the drone's own underside.)
for i in 1 2 3 4 5; do
  gz topic -t /gimbal/cmd_pitch -m gz.msgs.Double -p 'data: 1.57' >/dev/null 2>&1
  sleep 0.5
done
echo "[5] gimbal commanded to nadir"

# (6) Pilot: ARM + takeoff to 20 m via SERIAL1.
~/sitl-venv/bin/python \
  ~/Drone/ros2_ws/src/drone_vision/gazebo/sitl_gz_pilot.py \
  > ~/sitl_pilot.log 2>&1 &
echo "[6] pilot pid $!"

# (7) Engage SAR autonomy + watch /mission/state and detections.
echo "[7] engaging /mission/enable + watching for 120 s..."
python3 - <<'PY' 2>&1
import rclpy, time, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from drone_msgs.msg import MissionState, TargetWorld
from sensor_msgs.msg import NavSatFix
HOME = (-35.363262, 149.165237)
class W(Node):
    def __init__(self):
        super().__init__('w')
        latched = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub_en = self.create_publisher(Bool, '/mission/enable', latched)
        self.tp = self.tw = 0; self.states = []; self.last_tw = None
        self.max_disp = 0.0
        self.create_subscription(PointStamped, '/target_position',
                                 lambda m: setattr(self, 'tp', self.tp + 1), 10)
        self.create_subscription(TargetWorld, '/target/world', self.tw_cb, 10)
        self.create_subscription(MissionState, '/mission/state', self.ms_cb, 10)
        self.create_subscription(NavSatFix, '/vehicle/gps', self.gps_cb, 10)
        self.create_timer(5.0, self.report)
        self.create_timer(4.0, self.engage); self.engaged = False
        self.t = time.time()
    def engage(self):
        if self.engaged: return
        b = Bool(); b.data = True; self.pub_en.publish(b); self.engaged = True
        print("  /mission/enable = TRUE", flush=True)
    def tw_cb(self, m):
        self.tw += 1
        self.last_tw = (m.position_geo.latitude, m.position_geo.longitude, m.confidence)
    def gps_cb(self, m):
        if not (m.latitude or m.longitude): return
        R = 6371000.0
        dl = math.radians(m.latitude - HOME[0]); dn = math.radians(m.longitude - HOME[1])
        a = math.sin(dl/2)**2 + math.cos(math.radians(HOME[0])) * math.cos(math.radians(m.latitude)) * math.sin(dn/2)**2
        d = 2 * R * math.asin(math.sqrt(a))
        if d > self.max_disp: self.max_disp = d
    def ms_cb(self, m):
        if not self.states or self.states[-1] != m.state:
            self.states.append(m.state)
            print(f"  state -> {m.state}", flush=True)
    def report(self):
        el = time.time() - self.t
        print(f"  t={el:.0f}s tp={self.tp} tw={self.tw} last_tw={self.last_tw} maxN={self.max_disp:.1f}m",
              flush=True)
        if el >= 120:
            print("  state sequence:", " -> ".join(self.states), flush=True)
            rclpy.shutdown()
rclpy.init(); rclpy.spin(W())
PY
echo "[8] done"
