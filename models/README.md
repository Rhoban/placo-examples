# PlaCo examples robot models

## 6axis

<img src="6axis/robot.png" width="300">

* **Description**: IRB6620 6-axis industrial robot
* **Joints**:
    * `r1`, `r2`, `r3`, `r4`, `r5`, `r6`
* **Frames**:
    * `effector`: effector frame

## Megabot

<img src="megabot/robot.png" width="300">

* **Description**: Megabot quadruped robot
* **Joints**
    * **Actuated**: Prismatic joints (3 per leg)
        * `l1_c1`, `l1_c2`, `l1_c3`
        * `l2_c1`, `l2_c2`, `l2_c3`
        * `l3_c1`, `l3_c2`, `l3_c3`
        * `l4_c1`, `l4_c2`, `l4_c3`
    * **Passive**: Revolute joints (7 per leg)
        * `l1_r1`, `l1_r2`, `l1_r3`, `l1_r4`, `l1_r5`, `l1_r6`, `l1_r7`
        * `l2_r1`, `l2_r2`, `l2_r3`, `l2_r4`, `l2_r5`, `l2_r6`, `l2_r7`
        * `l3_r1`, `l3_r2`, `l3_r3`, `l3_r4`, `l3_r5`, `l3_r6`, `l3_r7`
        * `l4_r1`, `l4_r2`, `l4_r3`, `l4_r4`, `l4_r5`, `l4_r6`, `l4_r7`
* **Frames**:
    * `base`: body trunk
    * `leg_1`, `leg_2`, `leg_3`, `leg_4`: leg tips
    * Closure constraint frames (4 per leg)
        * `l1_cl1_1`, `l1_cl1_2`, `l1_cl2_1`, `l1_cl2_2`, `l1_cl3_1`, `l1_cl3_2`, `l1_cl4_1`, `l1_cl4_2`
        * `l2_cl1_1`, `l2_cl1_2`, `l2_cl2_1`, `l2_cl2_2`, `l2_cl3_1`, `l2_cl3_2`, `l2_cl4_1`, `l2_cl4_2`
        * `l3_cl1_1`, `l3_cl1_2`, `l3_cl2_1`, `l3_cl2_2`, `l3_cl3_1`, `l3_cl3_2`, `l3_cl4_1`, `l3_cl4_2`
        * `l4_cl1_1`, `l4_cl1_2`, `l4_cl2_1`, `l4_cl2_2`, `l4_cl3_1`, `l4_cl3_2`, `l4_cl4_1`, `l4_cl4_2`

## Omnidirectional

<img src="omnidirectional/robot.png" width="300">

* **Description**: Omnidirectional mobile robot
* **Joints**
    * **Actuated**: Revolute joints (3)
        * `wheel_1`, `wheel_2`, `wheel_3`
    * **Passive**: Bearings (20 per wheel)
        * `w1_1_passive`, `w1_2_passive`, `w1_3_passive`, `w1_4_passive`, `w1_5_passive`, `w1_6_passive`, `w1_7_passive`, `w1_8_passive`, `w1_9_passive`, `w1_10_passive`, `w1_11_passive`, `w1_12_passive`, `w1_13_passive`, `w1_14_passive`, `w1_15_passive`, `w1_16_passive`, `w1_17_passive`, `w1_18_passive`, `w1_19_passive`, `w1_20_passive`
        * `w2_1_passive`, `w2_2_passive`, `w2_3_passive`, `w2_4_passive`, `w2_5_passive`, `w2_6_passive`, `w2_7_passive`, `w2_8_passive`, `w2_9_passive`, `w2_10_passive`, `w2_11_passive`, `w2_12_passive`, `w2_13_passive`, `w2_14_passive`, `w2_15_passive`, `w2_16_passive`, `w2_17_passive`, `w2_18_passive`, `w2_19_passive`, `w2_20_passive`
        * `w3_1_passive`, `w3_2_passive`, `w3_3_passive`, `w3_4_passive`, `w3_5_passive`, `w3_6_passive`, `w3_7_passive`, `w3_8_passive`, `w3_9_passive`, `w3_10_passive`, `w3_11_passive`, `w3_12_passive`, `w3_13_passive`, `w3_14_passive`, `w3_15_passive`, `w3_16_passive`, `w3_17_passive`, `w3_18_passive`, `w3_19_passive`, `w3_20_passive`
* **Frames**:
    * `body`: robot body

## Orbita

## Quadruped

This is a simple quadruped robot with 12 DOFs. The dofs are named `leg1_a`, `leg1_b`, `leg1_c` and so
on for the three other legs.

There is a frame named `trunk` attached to the body, and one frame per leg tip (`leg1` etc.).


## Sigmaban

This is a model of the Sigmaban humanoid robot. There are 20 DOFs.

The robot is entirely pure-shape approximated for collisions geometry.

There is a frame `trunk` located un the trunk, `left_foot` and `right_foot`.

