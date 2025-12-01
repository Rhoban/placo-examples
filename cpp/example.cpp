#include "example.h"

SolverExample::SolverExample()
    : robot("../models/6axis", placo::model::RobotWrapper::IGNORE_COLLISIONS),
      solver(robot) {
  // Masking floating base
  solver.mask_fbase(true);

  // Adding an effector task
  frame_task = solver.add_frame_task("effector");
  frame_task.configure("effector", "soft", 1.0, 1.0);

  // Adding a slight regularization task
  solver.add_regularization_task(1e-6);

  robot.update_kinematics();
}

void SolverExample::update_trajectory(double t) {
  // Target expressed in world
  Eigen::Vector3d effector_world(1.5, cos(t) * 0.5, 0.75 + sin(2 * t) * 0.25);
  frame_task.position->target_world = effector_world;

  // Running the solve
  solver.solve(true);
  robot.update_kinematics();

  // Showing the computed joints
  std::cout << "Computed robot joints: " << robot.state.q.tail(6).transpose()
            << std::endl;

  // Dumping solver status to see tasks errors
  solver.dump_status();
}

placo::model::RobotWrapper &SolverExample::get_robot() { return robot; }