"""
Coordinate-frame helpers for geo-localisation.

Frames (PX4 conventions):
  body  — FRD: x=forward, y=right, z=down. Drone airframe.
  world — NED: x=north, y=east, z=down. Local origin at home/takeoff.
  gimbal — body frame rotated by gimbal yaw (around body z) then pitch
           (around body y). Roll assumed zero (3-axis stabilised).
  optical — ROS-standard camera optical frame mounted on the gimbal:
           x=right, y=down, z=forward. The detector reports pixels in
           this frame.

Geodetic conversion is flat-earth — adequate at <1 km radii where
2-3 m GPS drift dominates anyway. Replace with pyproj only if SAR
operations grow large enough to need it.
"""
import math


# ── Constants ───────────────────────────────────────────────────────

EARTH_R_M = 6_378_137.0       # WGS84 equatorial radius


# ── Quaternion → yaw extraction (PX4 q[w,x,y,z]) ────────────────────

def px4_quat_to_yaw(q):
    """Extract NED yaw (rotation around down axis) from PX4 attitude
    quaternion [w, x, y, z]. Returns radians in (-pi, pi]."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


# ── Rotation primitives (3x3 matrices as nested tuples) ─────────────
# We avoid numpy here on purpose — geo_localiser runs at detector rate
# (≤10 Hz) and the rotation chain is short. Saves an import on the Pi.

def _matmul3(A, B):
    return tuple(
        tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _matvec3(A, v):
    return (
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    )


def _R_yaw(rad):
    """Rotation around body z (down). Positive = right turn (PX4 FRD convention)."""
    c, s = math.cos(rad), math.sin(rad)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _R_pitch(rad):
    """Rotation around body y (right). Positive = nose up."""
    c, s = math.cos(rad), math.sin(rad)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _R_roll(rad):
    """Rotation around body x (forward). Positive = right wing down
    (ArduPilot ATTITUDE roll convention)."""
    c, s = math.cos(rad), math.sin(rad)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


# Optical frame → drone body frame (constant).
# ROS optical: x=right, y=down, z=forward
# PX4 body:    x=forward, y=right, z=down
# So: optical_x → body_y, optical_y → body_z, optical_z → body_x.
R_OPTICAL_TO_BODY = (
    (0.0, 0.0, 1.0),   # body x (forward) ← optical z (forward)
    (1.0, 0.0, 0.0),   # body y (right)   ← optical x (right)
    (0.0, 1.0, 0.0),   # body z (down)    ← optical y (down)
)


def gimbal_to_body(yaw_g_rad, pitch_g_rad):
    """Rotation from gimbal frame to drone body frame.
    R_g→b = R_yaw(ψ_g) · R_pitch(θ_g). Roll assumed zero."""
    return _matmul3(_R_yaw(yaw_g_rad), _R_pitch(pitch_g_rad))


# ── Full pixel-to-world ray ─────────────────────────────────────────

def pixel_to_world_ray(u, v, fx, fy, cx, cy,
                       gimbal_yaw_deg, gimbal_pitch_deg,
                       drone_yaw_rad,
                       drone_roll_rad=0.0, drone_pitch_rad=0.0):
    """Return the unit ray (in NED) from the camera through pixel (u,v).

    Parameters
    ----------
    u, v       : pixel coordinates in the camera image
    fx, fy     : focal lengths (px) — typically equal for square pixels
    cx, cy     : principal point (px) — image centre for an undistorted model
    gimbal_*   : gimbal angles (degrees) in body frame; pitch=-90 = nadir
    drone_yaw_rad   : drone yaw in NED (rotation around down axis)
    drone_roll_rad  : drone roll (body x). Default 0 = level (legacy).
    drone_pitch_rad : drone pitch (body y). Default 0 = level (legacy).

    Returns
    -------
    (rx, ry, rz) — direction vector in NED, unit-length.
    """
    # Step 1: pixel → optical-frame ray
    rx_o = (u - cx) / fx
    ry_o = (v - cy) / fy
    rz_o = 1.0
    # Step 2: optical → body
    r_body = _matvec3(R_OPTICAL_TO_BODY, (rx_o, ry_o, rz_o))
    # Step 3: apply gimbal rotation (gimbal frame → body frame). The optical
    # frame is mounted on the gimbal, so r_body in step 2 is actually the
    # ray in the *gimbal* body-aligned frame. We rotate by the gimbal's
    # body-frame attitude to get the ray in the drone's body frame.
    R_gb = gimbal_to_body(math.radians(gimbal_yaw_deg),
                          math.radians(gimbal_pitch_deg))
    r_body_drone = _matvec3(R_gb, r_body)
    # Step 4: body → world (NED) using the full drone attitude DCM
    # R = Rz(yaw)·Ry(pitch)·Rx(roll) (aerospace ZYX / 3-2-1). Ignoring
    # roll/pitch threw the ground hit off by tens of metres whenever the
    # drone was tilted in cruise — the dominant geolocation error after
    # camera calibration.
    R_bw = _matmul3(_R_yaw(drone_yaw_rad),
                    _matmul3(_R_pitch(drone_pitch_rad),
                             _R_roll(drone_roll_rad)))
    r_world = _matvec3(R_bw, r_body_drone)
    # Normalise
    n = math.sqrt(r_world[0] ** 2 + r_world[1] ** 2 + r_world[2] ** 2)
    if n < 1e-9:
        return (0.0, 0.0, 1.0)
    return (r_world[0] / n, r_world[1] / n, r_world[2] / n)


def ground_intersect(origin_ned, ray_ned, ground_z=0.0):
    """Intersect a ray (origin + t·dir) with the horizontal plane z=ground_z.

    Returns (north, east, ground_z) NED coords, or None if the ray points
    away from the ground (rz <= 0 in NED, i.e. up or horizontal)."""
    ox, oy, oz = origin_ned
    rx, ry, rz = ray_ned
    if rz <= 1e-6:
        return None
    t = (ground_z - oz) / rz
    if t <= 0:
        return None
    return (ox + t * rx, oy + t * ry, ground_z)


# ── Flat-earth NED ↔ geodetic ──────────────────────────────────────

def ned_to_geo(home_lat_deg, home_lon_deg, north_m, east_m):
    """Convert a NED offset (m) from a home lat/lon to absolute lat/lon.
    Flat-earth approximation — adequate within ~1 km at SAR scales."""
    lat_rad = math.radians(home_lat_deg)
    dlat_deg = math.degrees(north_m / EARTH_R_M)
    dlon_deg = math.degrees(east_m / (EARTH_R_M * math.cos(lat_rad)))
    return (home_lat_deg + dlat_deg, home_lon_deg + dlon_deg)
