# Current mission state plus a few telemetry fields useful for ground-station UIs.

std_msgs/Header header

string state              # e.g. "INIT", "SEARCH", "GEO_LOCK", "APPROACH", "TERMINAL", "DELIVER", "RTL"
string sub_state          # state-specific detail (e.g. waypoint index)

float32 target_distance_m # horizontal distance to current target, NaN if no target
float32 altitude_agl_m    # current AGL altitude
float32 battery_remaining # 0..1, NaN if unknown

bool vision_locked
bool gimbal_healthy
bool gps_healthy
