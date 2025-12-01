#include <placo/kinematics/kinematics_solver.h>
#include <placo/model/robot_wrapper.h>

class SolverExample {
public:
  SolverExample();

  void update_trajectory(double t);

  placo::model::RobotWrapper &get_robot();

protected:
  // Robot wrapper
  placo::model::RobotWrapper robot;

  // Kinematics solver
  placo::kinematics::KinematicsSolver solver;

  // Tasks
  placo::kinematics::FrameTask frame_task;
};