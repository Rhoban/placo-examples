# Workspace estimation

This directory contains an example of workspace estimation for the SPM model using PlaCo:

* In `workspace_estimation.py`, the robot is moved in order to find the limit in different directions. A polytope of the form $A q \le b$ is then obtained, and saved in `workspace.pkl`
* In `kinematics.py`, the polytope is used with [joint-space half-spaces](https://placo.readthedocs.io/en/latest/kinematics/joint_space_half_spaces_constraint.html) constraints, resulting in a kinematics controller able to avoid self-collisions.