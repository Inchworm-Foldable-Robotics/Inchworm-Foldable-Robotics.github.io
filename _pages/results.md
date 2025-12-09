---
layout: archive
title: "Results"
permalink: /results/
author_profile: true
---

In this section, the team displays the results of experimentation in both video and graph, and discusses the differences between trials.    



![](..\assets\04_Results\ActP4.png)  

*Figure 1. Experimental data of the inchworm at a period of 4 seconds*

![](..\assets\04_Results\ActP2.png)  

*Figure 2. Experimental data of the inchworm at a period of 2 seconds. As can be seen, some of the data got distorted during tracking. This has been taken into account and will not affect analysis*

![](..\assets\04_Results\ActP1.png)  

*Figure 3. Experimental data of the inchworm at a period of 1 second*

<iframe width="560" height="315" src="https://www.youtube.com/embed/kaO62zham5g?si=5iOVbgLosDtrCYty" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

![](..\assets\04_Results\ActP_5.png)  

*Figure 4. Experimental data of the inchworm at a period of .5 seconds*

![](..\assets\04_Results\ActPAll.png)  

*Figure 4. Experimental data of the inchworm at all periods overlayed*

---

* **Qualitative agreement:**

  * In both simulation and experiment, the end-effector x-position increases monotonically with time for the periods where the gait is well formed (4 s, 2 s, 1 s). The trajectories are staircase-like at longer periods, reflecting discrete anchor–arch–relax cycles, and become smoother, quasi-linear ramps at higher frequencies as individual steps overlap more. Decreasing the gait period increases net forward progress per unit time in the model and in the hardware up to roughly the 1 s case, although the physical 2 s trial under-performs the 4 s case because the feet slip more between steps.

    In simulation, the sweep over gait period identifies an optimal intermediate region between about 1 s and 2 s where the central Sarrus linkage and both feet complete clean lift–drag cycles and the end-effector trace has the steepest positive slope. The experimental data follow the same qualitative trend: the 1 s and 0.5 s trials clearly move faster than the 4 s and 2 s trials, and all successful runs show step-like motion consistent with alternating front/rear anchoring. The main qualitative discrepancy is at the highest frequency: in MuJoCo the 0.5 s gait degrades and the model begins to drift sideways, whereas the physical robot still achieves forward progress at 0.5 s, albeit with more slip and variability.

* **Quantitative differences:**

  * From the tracking data, the 4 s gait moves the end-effector about 0.26 m over roughly 28 s (average speed ≈ 9 mm/s) with strides on the order of 3–4 cm per cycle. The 2 s gait covers only ≈ 0.08 m in about 23 s (≈ 3–4 mm/s) and has much smaller effective strides (~5–10 mm), consistent with the visually noisier, slipping trajectory. In contrast, the 1 s and 0.5 s gaits both travel roughly 0.33–0.36 m in 11–12 s, giving average speeds near 30 mm/s; relative to the 4 s run this is about a 3× increase in speed even though the stride length per cycle is slightly reduced.

    The MuJoCo simulations produce end-effector traces with similar staircase structure but generally steeper slopes and less variability than the experimental curves, indicating that the model overestimates stride length and average speed by on the order of tens of percent, especially at the slower periods where real-world slip and compliance losses are largest. The simulation sweep also suggests a single best period in the 1–2 s range, whereas the hardware data show a relatively flat optimum between 1 s and 0.5 s. Finally, the 0.5 s simulated gait effectively stalls in x while the real robot maintains nearly the same mean speed as at 1 s, highlighting that actuator limits and contact asymmetries are not fully captured in the current model.

* **Failure modes / anomalies:**

  * In the simulation, the most obvious failure mode occurs at the shortest commanded period (0.5 s): the middle Sarrus hinge does not complete its intended ±90° motion, the feet never fully alternate between high- and low-friction states, and the model begins to “walk” sideways in the global y-direction instead of advancing in x. This behaviour is consistent with the highly symmetric geometry, the use of a single actuated mid joint, and idealized friction; with small numerical or geometric asymmetries the contact solver can preferentially push the robot sideways. At longer periods the virtual robot remains stable but still exhibits minor lateral drift and unrealistically repeatable steps.

    In hardware, the main anomalies are incomplete steps, occasional backward slips, and tracking artefacts. The 2 s dataset contains a clear outlier where the tracked point briefly jumps to roughly twice its nominal x-position before returning; this spike is treated as a measurement error and ignored when interpreting the overall trend. Across periods, the real prototype also shows cycle-to-cycle variability due to hinge sag, servo backlash, and non-uniform pad contact, which explains the rounded “stairs” and small regressions in the experimental plots. These observations point to concrete improvements: refining the MuJoCo model with better servo dynamics (torque and velocity limits, first-order response), calibrated hinge stiffness/damping, and slightly asymmetric or anisotropic friction pads to suppress sideways drift; and improving the physical robot with stiffer hinge reinforcement, higher-contrast and more durable friction pads, tighter servo timing on the ESP32, repeated trials with averaged performance, and more robust tracking (fixed camera, fiducial markers, simple smoothing), all of which should reduce the gap between simulated and measured x-position traces.