---
layout: archive
title: "MK 2"
permalink: /mk2/
author_profile: true
# redirect_from:
#   - /resume
---

# MK2: Future Development and Next Steps
Having completed the first functional prototype, the team identified a clear roadmap for the second generation design known as MK2. This version will address the limitations observed in MK1 while building on the insights gained during testing, servo timing refinement, and structural optimization. The next steps focus on improving actuation efficiency, hinge architecture, sensing capability, mechanical robustness, and overall manufacturability.

## 1. Actuation System Improvements

One of the primary limitations of MK1 was the performance of the micro servos. Although lightweight and affordable, these servos introduced several constraints including limited torque, slow rotational speed, and nonlinear arc based motion that restricted gait efficiency. To overcome these drawbacks, the team plans to explore alternative actuation strategies for MK2. Linear actuators such as miniature rack and pinion mechanisms, shape memory alloy bundles, or cable driven linkages may provide more direct contraction forces without the side rotation typical of servo horns. These solutions could also offer smoother compliance and reduce the need for complex mounting brackets.
If rotational servos remain part of the system, MK2 will incorporate faster and more powerful servos with metal gears and more precise mounting interfaces. These improvements aim to reduce backlash, improve responsiveness, and eliminate the actuation bottleneck that currently limits locomotion speed.

## 2. Structural and Hinge Redesign

Testing of MK1 revealed issues with hinge durability, overfolding, and panel alignment. As a result, MK2 will adopt a redesigned hinge structure. Reinforcing hinge regions through wider kerfs, additional hinge film layers, and improved laser cut patterns will help distribute stress and extend fatigue life. Although MK1 relied on three dimensional printed fins to prevent overfolding, MK2 may integrate physical hard stop geometry directly into the laminated body or incorporate perimeter ribs and modular snap on hinge guards. These refinements will simplify assembly and improve reliability.

## 3. Improved Foot and Friction Mechanisms

MK1 used thin lines of hot glue as a makeshift friction enhancer, and while this method worked better than expected, MK2 will formalize it into a more engineered and repeatable system. The next iteration will examine laser cut silicone pads, flexible three dimensional printed materials, or rubber based inserts to create consistent friction surfaces. These new foot pads will increase traction, reduce wear, and allow tuning for different surfaces.
Inspired by biological inchworms, MK2 may also incorporate directional friction surfaces such as angled microspines, textured pads, or soft ridged feet that resist backward motion while allowing smooth forward sliding. These enhancements could greatly improve locomotion efficiency.

## 4. Electronics, Sensing, and Control Enhancements

Although the ESP32 based control system in MK1 functioned well, it lacked sensing capability and closed loop feedback. MK2 will expand the electronics platform to include additional sensors such as servo angle encoders, force sensitive resistors under the feet for anchoring detection, magnetic sensors for timing, and inertial measurement units for stability. These additions will allow MK2 to operate with adaptive closed loop control instead of relying entirely on preset timing.
On the software side, MK2 will explore phase based gait planning, proportional integral derivative timing correction, state machine control, and potentially reinforcement learning to automatically optimize gait performance. These new control systems will help the robot adapt to different surfaces, actuator variability, and uneven terrain.

## 5. Simulation Driven Optimization

While MK1 used MuJoCo primarily for hinge motion validation, MK2 will use simulation more extensively to inform design decisions. Dynamic simulations that model realistic friction, servo torque curves, hinge stiffness, damping, and center of mass movement will provide deeper insight into performance. Parameter sweeps will help identify optimal link lengths, hinge spacing, actuator travel, gait timing, and foot geometry. Simulation driven development will shorten the physical prototyping process and lead to more efficient designs.

## 6. Manufacturing and Assembly Refinements

The laminated hinge system used in MK1 performed well but required multiple laser passes and a heat press bonding process. MK2 will streamline fabrication by refining laser parameters to improve kerf quality, reduce charring, and minimize manual trimming. The team will also explore modular assembly features such as snap fit printed components, removable servo mounts, and clip on friction pads to simplify repair and experimentation.
Assembly repeatability will be improved by adding alignment holes to all layers, creating printed folding jigs, and incorporating built in guides to help shape the triangular body accurately. These changes will reduce variance between builds and improve overall consistency.

## 7. Path Toward a Fully Functional Inchworm MK2

When all planned improvements are combined, MK2 will achieve smoother, faster, and stronger actuation along with a more durable hinge system, improved traction through engineered foot pads, enhanced sensing for closed loop control, and more efficient gait timing. The modular and manufacturable body structure will support advanced behaviors such as climbing gentle inclines, navigating uneven terrain, or functioning as a segment in a larger chain of robotic modules.
Altogether, MK2 will elevate the exploratory MK1 prototype into a more refined, capable, and versatile robotic platform prepared for advanced research and performance evaluation.