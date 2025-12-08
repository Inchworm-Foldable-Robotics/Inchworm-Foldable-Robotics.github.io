---
layout: archive
title: "MK 2"
permalink: /mk2/
author_profile: true
# redirect_from:
#   - /resume
---

# MK2: Future Development and Next Steps

Having completed the first functional prototype, the team identified a clear roadmap for a second generation design, referred to as MK2. This version will address the limitations observed in MK1 while leveraging the insights gained during testing, timing refinement, and structural optimization. The next steps focus on improving actuation efficiency, hinge architecture, sensing capability, mechanical robustness, and manufacturability.

## 1. Actuation System Improvements

One of the primary limitations in MK1 was the performance of the micro servos. While they provided a lightweight, low cost starting point, they introduced several constraints, including limited torque, slow speed, and nonlinear arc based motion. MK2 will explore alternative actuation strategies to overcome these drawbacks:

## 1.1. Linear Actuators or SMA Alternatives

Linear actuation would provide a more direct translation of contraction forces without unwanted side rotation inherent to servo horns. Options include:

* Miniature rack and pinion linear servos

* Shape memory alloy bundles for high stroke to weight ratios

* Cable driven actuation similar to biological muscle contraction

Each approach offers smoother compliance and could eliminate the need for complex printed brackets.

## 1.2 Higher Torque, Faster Servos

If rotational servos remain part of the system, MK2 will incorporate:

* More responsive micro servos with metal gears

* Higher speed models to improve gait frequency

* Servo specific mounting geometry that minimizes backlash

These upgrades aim to remove the primary bottleneck slowing the locomotion cycle.

## 2. Structural and Hinge Redesign

MK1 revealed that hinge durability, overfolding, and panel alignment remain key constraints. MK2 will pursue a redesigned hinge architecture that offers:

## 2.1 Reinforced Hinge Regions

* Wider hinge kerfs to distribute stress

* Multi layer hinge film segments for improved fatigue life

* Laser kerf patterns that reduce stress concentration

## 2.2 Integrated Overfold Protection

While MK1 relied on 3D printed fins, MK2 may move toward:

* Embedded “hard stop” geometries within the laminate

* Perimeter ribs formed directly into the pattern

* Modular printed hinge guards that snap into the foldable sheet

This would reduce reliance on glue and simplify assembly.

## 3. Improved Foot and Friction Mechanisms

MK1 used hot glue as a lightweight friction enhancer, and it worked surprisingly well. MK2 will formalize this into a repeatable, engineered solution.

## 3.1 Replace Hot Glue with Designed Foot Pads

Potential options include:

* Laser cut silicone pads

* TPU flexible 3D prints

* Replaceable rubberized inserts

These pads would offer more consistent friction, reduced wear, and adjustable traction for different surfaces.

## 3.2 Directional Friction Surfaces

Inspired by biological inchworm prolegs, MK2 may explore:

* Angled micro spines

* Anisotropic surface textures

* Soft ridged pads that resist backward motion

This improvement could dramatically boost locomotion efficiency.

## 4. Electronics, Sensing, and Control Enhancements

The MK1 control system—an ESP32 driving three servos—proved functional but lacked sensing and closed loop control. The next iteration will expand control capabilities:

## 4.1 Integrated Feedback Sensors

Possible additions include:

* Servo angle encoders

* Force sensitive resistors under feet for anchoring detection

* Hall effect sensors for cycle timing

* IMU feedback for gait stabilization

Such sensors would enable closed loop control rather than open loop timing.

## 4.2 More Advanced Gait Control Algorithms

MK2 will explore:

* Phase based gait planners

* PID timing correction

* State machine based movement logic

* Potential reinforcement learning for gait optimization

This allows the robot to adapt to surface changes, actuator variability, or slope angles.

## 5. Simulation Driven Optimization

While MK1 used MuJoCo to validate hinge kinematics, MK2 will expand simulation use:

## 5.1 Dynamic Simulation With Realistic Friction

This includes tuning:

* Sliding vs. anchoring friction coefficients

* Servo torque profiles

* Joint stiffness and damping

* Center of mass motion over the gait cycle

## 5.2 Parametric Optimization

Using simulation sweeps, MK2 can optimize:

* Link lengths

* Hinge spacing

* Actuator travel

* Duty cycle and timing

* Foot geometry and friction levels

Simulation driven design will shorten physical prototyping time and lead to a more efficient locomotion strategy.

## 6. Manufacturing and Assembly Refinements

The laminated hinge system performed well but required multiple laser cut passes and a heat press step. MK2 will refine manufacturability:

## 6.1 Consolidated Laser Passes

Adjust cut parameters to ensure:

* cleaner hinge kerfs

* reduced manual cleanup

* fewer burnt edges

## 6.2 Modular Component Design

MK2 will investigate:

* snap fit 3D printed parts

* replaceable servo mounts

* detachable foot modules

This will simplify repairs and allow faster experimentation.

## 6.3 Rapid Assembly Alignment

Alignment pins were useful in MK1. MK2 may add:

* alignment holes to all layers

* printed jigs for folding

* built in notches to help shape the triangular body

These changes improve repeatability and reduce assembly variance.

## 7. Path Toward a Fully Functional Inchworm MK2

Bringing together all planned improvements, MK2 aims to deliver:

* smoother, faster, and stronger actuation

* a more durable and reliable hinge system

* improved traction with engineered feet

* advanced sensing for closed loop control

* more efficient gait timing

* A modular and manufacturable mechanical structure

This next generation prototype will not only improve performance but also create a platform suitable for more advanced behaviors, such as climbing shallow inclines, navigating uneven terrain, or acting as a segment in a larger multi module robotic system.