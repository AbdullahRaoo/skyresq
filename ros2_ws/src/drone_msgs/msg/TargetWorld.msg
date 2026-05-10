# A target localised in the world frame.
# Source flags indicate provenance — fresh geo-lock vs continuous vision update vs propagated stale estimate.

uint8 SOURCE_GEO_LOCK = 1
uint8 SOURCE_VISION   = 2
uint8 SOURCE_STALE    = 3

std_msgs/Header header

# NED position relative to home (used in SITL and as a common frame on hardware).
geometry_msgs/PointStamped position_ned

# Geodetic position (lat/lon/alt). On hardware this is the authoritative coord;
# in SITL it is computed from position_ned via flat-earth around home_position.
sensor_msgs/NavSatFix position_geo

float32 confidence
uint8   source
