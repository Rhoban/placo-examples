import time
import placo
import numpy as np
import warnings
import matplotlib.pyplot as plt
from footsteps_planner import draw_footsteps

warnings.filterwarnings("ignore")

DT = 0.005
model_filename = "../models/sigmaban/robot.urdf"

# Loading the robot
robot = placo.HumanoidRobot(model_filename)

# Walk parameters - if double_support_ratio is not set to 0, should be greater than replan_frequency
parameters = placo.HumanoidParameters()

# Timing parameters
parameters.single_support_duration = 0.38  # Duration of single support phase [s]
parameters.single_support_timesteps = 10  # Number of planning timesteps per single support phase
parameters.double_support_ratio = 0.0  # Ratio of double support (0.0 to 1.0)
parameters.startend_double_support_ratio = 1.5  # Ratio duration of supports for starting and stopping walk
parameters.planned_timesteps = 48  # Number of timesteps planned ahead

# Posture parameters
parameters.walk_com_height = 0.32  # Constant height for the CoM [m]
parameters.walk_foot_height = 0.04  # Height of foot rising while walking [m]
parameters.walk_trunk_pitch = 0.15  # Trunk pitch angle [rad]
parameters.walk_foot_rise_ratio = 0.2  # Time ratio for the foot swing plateau (0.0 to 1.0)

# Feet parameters
parameters.foot_length = 0.1576  # Foot length [m]
parameters.foot_width = 0.092  # Foot width [m]
parameters.feet_spacing = 0.122  # Lateral feet spacing [m]
parameters.zmp_margin = 0.02  # ZMP margin [m]
parameters.foot_zmp_target_x = 0.0  # Reference target ZMP position in the foot [m]
parameters.foot_zmp_target_y = 0.0  # Reference target ZMP position in the foot [m]

# Limit parameters
parameters.walk_max_dtheta = 1  # Maximum dtheta per step [rad]
parameters.walk_max_dy = 0.04  # Maximum dy per step [m]
parameters.walk_max_dx_forward = 0.08  # Maximum dx per step forward [m]
parameters.walk_max_dx_backward = 0.03  # Maximum dx per step backward [m]

# Creating the kinematics solver
solver = placo.KinematicsSolver(robot)
solver.enable_velocity_limits(True)
solver.dt = DT

# Creating the walk QP tasks
tasks = placo.WalkTasks()
tasks.initialize_tasks(solver, robot)

# Creating a joint task to assign DoF values for upper body
elbow = -50 * np.pi / 180
shoulder_roll = 0 * np.pi / 180
shoulder_pitch = 20 * np.pi / 180
joints_task = solver.add_joints_task()
joints_task.set_joints(
    {
        "left_shoulder_roll": shoulder_roll,
        "left_shoulder_pitch": shoulder_pitch,
        "left_elbow": elbow,
        "right_shoulder_roll": -shoulder_roll,
        "right_shoulder_pitch": shoulder_pitch,
        "right_elbow": elbow,
        "head_pitch": 0.0,
        "head_yaw": 0.0,
    }
)
joints_task.configure("joints", "soft", 1.0)

# Placing the robot in the initial position
print("Placing the robot in the initial position...")
tasks.reach_initial_pose(
    np.eye(4),
    parameters.feet_spacing,
    parameters.walk_com_height,
    parameters.walk_trunk_pitch,
)
print("Initial position reached")

# Creating the FootstepsPlanner
repetitive_footsteps_planner = placo.FootstepsPlannerRepetitive(parameters)
d_x = 0.1
d_y = 0.0
d_theta = 0.2
nb_steps = 10
repetitive_footsteps_planner.configure(d_x, d_y, d_theta, nb_steps)

# Planning footsteps
T_world_left = placo.flatten_on_floor(robot.get_T_world_left())
T_world_right = placo.flatten_on_floor(robot.get_T_world_right())
footsteps = repetitive_footsteps_planner.plan(placo.HumanoidRobot_Side.left, T_world_left, T_world_right)

supports = placo.FootstepsPlanner.make_supports(footsteps, 0.0, True, parameters.has_double_support(), True)

# Creating the pattern generator and planning
walk = placo.WalkPatternGenerator(robot, parameters)
trajectory = walk.plan(supports, robot.com_world(), 0.0)

# Plotting the trajectory
draw_footsteps(trajectory.get_supports(), show=False)

ts = np.linspace(0, trajectory.t_end, 1000)
com = np.array([trajectory.get_p_world_CoM(t)[:2] for t in ts]).T
zmp = np.array([trajectory.get_p_world_ZMP(t, placo.LIPM.compute_omega(parameters.walk_com_height)) for t in ts]).T
dcm = np.array([trajectory.get_p_world_DCM(t, placo.LIPM.compute_omega(parameters.walk_com_height)) for t in ts]).T

plt.plot(com[0], com[1], label="CoM", c="tab:red", lw=3)    
plt.plot(zmp[0], zmp[1], label="ZMP", c="tab:blue", lw=3)
plt.plot(dcm[0], dcm[1], label="DCM", c="tab:green", lw=3)

plt.legend()
plt.grid()
plt.show()
