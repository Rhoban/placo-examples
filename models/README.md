# PlaCo examples robot models

## 6axis

<img src="6axis/robot.png" width="200">

* **Description**: IRB6620 6-axis industrial robot
* **Joints**:
    * `r1`, `r2`, `r3`, `r4`, `r5`, `r6`
* **Frames**:
    * `effector`: effector frame

## Megabot

## Omnidirectional

## Orbita

## Quadruped

This is a simple quadruped robot with 12 DOFs. The dofs are named `leg1_a`, `leg1_b`, `leg1_c` and so
on for the three other legs.

There is a frame named `trunk` attached to the body, and one frame per leg tip (`leg1` etc.).


## Sigmaban

This is a model of the Sigmaban humanoid robot. There are 20 DOFs.

The robot is entirely pure-shape approximated for collisions geometry.

There is a frame `trunk` located un the trunk, `left_foot` and `right_foot`.

