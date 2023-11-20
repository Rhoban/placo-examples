# PlaCo examples robot models

## 6axis

This is a model of the IRB6620 6-axis robot. The 6 joints are `r1`, `r2`, `r3`, `r4`,
`r4` and `r6`.

The effector is represented by `effector` frame.

## Sigmaban

This is a model of the Sigmaban humanoid robot. There are 20 DOFs.

The robot is entirely pure-shape approximated for collisions geometry.

There is a frame `trunk` located un the trunk, `left_foot` and `right_foot`.

## Quadruped

This is a simple quadruped robot with 12 DOFs. The dofs are named `leg1_a`, `leg1_b`, `leg1_c` and so
on for the three other legs.

There is a frame named `trunk` attached to the body, and one frame per leg tip (`leg1` etc.).

