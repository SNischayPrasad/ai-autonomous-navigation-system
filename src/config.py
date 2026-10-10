"""Central configuration for the navigation stack.

Every tunable number lives here so that experiments are reproducible and no
"magic numbers" are buried inside the modules. Units are SI (metres, seconds,
radians unless a name says *_deg).
"""
from dataclasses import dataclass, field


@dataclass
class SimConfig:
    dt: float = 0.1                  # simulation time step [s] -> 10 Hz control loop
    max_steps: int = 2000            # hard stop for one run (200 s of simulated time)
    truth_resolution: float = 0.1    # resolution of the ground-truth world raster [m/cell]
    seed: int = 7                    # RNG seed for sensor noise / random maps


@dataclass
class RobotConfig:
    wheelbase: float = 1.6           # distance between axles [m]
    length: float = 2.4              # body length, used for drawing [m]
    width: float = 1.4               # body width, used for drawing [m]
    radius: float = 0.9              # collision radius (circle that covers the body) [m]
    max_speed: float = 3.0           # [m/s] (typical AMR / delivery-robot speed)
    max_accel: float = 2.0           # [m/s^2]
    max_decel: float = 5.0           # [m/s^2]
    max_steer_deg: float = 35.0      # front-wheel steering limit [deg]
    max_reverse_speed: float = 1.0   # reversing is only used by the recovery behaviour [m/s]


@dataclass
class SensorConfig:
    lidar_beams: int = 120           # 360 deg scan -> 3 deg per beam
    lidar_range: float = 12.0        # [m], kept inside the camera window on purpose
    lidar_noise_std: float = 0.03    # range noise [m]
    camera_view_m: float = 24.0      # bird's-eye-view camera covers 24 m x 24 m around the robot
    camera_px: int = 240             # camera image size [px] -> 0.1 m per pixel
    camera_noise_std: float = 6.0    # pixel noise (0-255 scale)


@dataclass
class PerceptionConfig:
    hit_evidence: int = 3            # occupancy evidence added for an obstacle observation
    miss_evidence: int = 1           # evidence removed for a free-space observation
    max_evidence: int = 10           # clamp so the map can still change its mind
    occupied_threshold: int = 2      # evidence >= threshold -> cell is an obstacle
    min_blob_area_px: int = 12       # ignore colour blobs smaller than this (noise)
    dynamic_inflation: float = 0.8   # extra padding around moving objects in the costmap [m]
    prediction_horizon_s: float = 2.0  # also block where moving objects will be in the next 2 s


@dataclass
class PlannerConfig:
    algorithm: str = "astar"         # astar | dijkstra | weighted_astar
    grid_resolution: float = 0.25    # planning grid cell size [m]
    safety_margin: float = 0.3       # clearance added to the robot radius when inflating [m]
    soft_cost_distance: float = 2.0  # cells closer than this to an obstacle get extra cost [m]
    soft_cost_weight: float = 5.0    # how strongly the planner prefers the middle of an aisle
    weighted_astar_w: float = 2.0    # heuristic weight for weighted A*
    replan_interval_s: float = 1.5   # periodic replanning so the map/dynamic info stays fresh
    path_spacing: float = 0.5        # resampled waypoint spacing [m]


@dataclass
class ControlConfig:
    lookahead_base: float = 2.0      # pure-pursuit lookahead at standstill [m]
    lookahead_gain: float = 0.5      # extra lookahead per m/s of speed [s]
    speed_kp: float = 1.5            # proportional gain of the speed controller
    goal_tolerance: float = 1.2      # goal counts as reached inside this radius [m]
    stop_distance: float = 1.8       # minimum gap to an obstacle on the path [m] (+ braking distance)
    slow_down_distance: float = 7.0  # start slowing when an obstacle is this far along the path [m]
    corridor_half_width: float = 1.05  # lateral half-width of the swept path corridor [m]
    caution_radius: float = 8.0      # moving object closer than this -> caution speed
    caution_speed: float = 2.0       # [m/s]
    curvature_speed_gain: float = 2.0  # slow down in tight turns
    max_lateral_accel: float = 1.5   # comfort limit used to slow down before curves [m/s^2]
    dynamic_block_backoff_s: float = 5.0  # no path because of a moving object -> back off after this
    blocked_replan_s: float = 2.0    # if stopped this long, force a replan around the blocker
    recovery_time_s: float = 2.5     # longest reverse manoeuvre when pinned against a wall [s]
    replan_hysteresis: float = 0.85  # periodic replans must be 15 % cheaper to replace the path
    max_plan_failures: int = 150     # consecutive failed plans (15 s) before declaring FAILED


@dataclass
class Config:
    sim: SimConfig = field(default_factory=SimConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    perception: PerceptionConfig = field(default_factory=PerceptionConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
