---
permalink: /
title: "Home"
author_profile: true
redirect_from: 
  - /about/
  - /about.html
---

## Team and Project Overview

This site documents the semester project for RAS 557: Foldable Robotics (Fall 2025, instructor: Daniel M. Aukes). The course focuses on bio‑inspired terrestrial locomotion, foldable fabrication methods, and simulation‑driven design.

The project investigates an inchworm‑inspired crawling robot based on the locomotion of Manduca sexta larvae. The robot uses rigid foldable linkages and friction‑modulated pads to reproduce the characteristic looped, two‑anchor gait of an inchworm in a planar setting. The long‑term objective is to develop a foldable, repeatable platform where geometry, actuation, and contact properties can be systematically varied and compared between simulation and experiment.

The project team consists of:

- Sai Srinivas Tatwik Meesala  
- Colin Fricke  
- Nathan Vairora  

Together, the team is responsible for mechanism design, dynamic modeling in MuJoCo, parameter studies, and experimental validation, with this website serving as the primary public report for both Project Assignment 1 and Project Assignment 2. :contentReference[oaicite:0]{index=0}


## Team Goals

The technical goals of the project are to:

- Design a foldable, inchworm‑inspired crawler that uses rigid linkages and friction‑modulated pads to achieve reliable forward locomotion on a flat surface.
- Develop kinematic and dynamic models (MuJoCo + Python) that capture the main features of the biological gait, including alternating anchoring, lift‑and‑drag motion, and body contraction/extension.
- Define and measure performance metrics such as stride length, gait period, average speed, and basic energetic cost, in both simulation and hardware.
- Perform parameter studies on key design variables (e.g., linkage geometry, stroke amplitude, joint stiffness/damping, pad friction) and interpret their effect on locomotion performance.
- Compare simulated and experimental behavior, and use discrepancies to refine the model and hardware design.

The learning goals of the team within the context of RAS 557 are to:

- Gain practical experience with foldable robotics workflows from concept to prototype (cut, fold, assemble, actuate).
- Apply vector‑based kinematics, Jacobians, and constraint equations to a real mechanism, using the course textbook and assignments as a reference. 
- Build and tune dynamic simulations in MuJoCo, including compliance, realistic actuators, and contact/friction modeling.
- Design and execute small‑scale experiments (video tracking, timing, basic force/torque estimation) and integrate the resulting data into a modeling and optimization workflow.

**[TODO: Add final list of team goals after design review]**


## Candidate Identification and Intent of Choice

Within this project, “candidate” refers to the specific combination of:

- Biological inspiration: Manduca sexta inchworm gait (two anchors, looping contraction and extension).
- Mechanical mechanism: a planar rigid‑link crawler with two gripping pads and an internal linkage network that converts actuator stroke into coordinated lift and translation.
- Fabrication approach: laser‑cut or otherwise patterned laminate structure compatible with the foldable robotics workflow used in the course. :contentReference[oaicite:2]{index=2}

For Project Assignment 1, the team selected a candidate mechanism that approximates the inchworm’s deformable body using a small number of rigid links and hinges. The current concept uses:

- Two rigid “body” sections, each terminating in a friction pad that can act as an anchor.
- An internal linkage (inspired by Sarrus and slider–crank topologies) that provides coupled vertical lift and horizontal motion between the two pads when driven by a single linear actuator.
- Joints whose stiffness and damping can later be tuned in MuJoCo to approximate bending and compliance.

This candidate was chosen because it:

- Captures the essential two‑anchor, lift‑and‑drag pattern of inchworm locomotion in a planar, rigid‑link form.
- Is amenable to vector‑based kinematic analysis (as completed in Project Assignment 1) and to extension into a dynamic MuJoCo model.
- Fits well within the foldable fabrication constraints (laminate layers, simple hinge patterns, low part count).
- Provides clear design parameters (link lengths, stroke, joint stiffness, pad friction) for systematic study.

**[TODO: Update candidate selection rationale after next iteration]**


## Research Question and Rationale

At a high level, the project asks:

How do gait timing and geometry (period, stroke/stride length, and body configuration) influence the achievable speed and robustness of an inchworm‑inspired foldable crawler, in both simulation and experiment?

The emerging research question focuses specifically on the relationship between:

- Gait period and duty cycle (how long each pad is anchored vs sliding),
- Effective stride length per cycle (net translation per actuation stroke),
- Resulting average forward speed and basic energetic cost (work per unit distance and per unit weight),

under variations in linkage geometry and contact properties.

The rationale for emphasizing period/stride length and speed is:

- Biomechanical studies of Manduca sexta show that larvae achieve a characteristic stride‑to‑body‑length ratio and operate in a relatively narrow range of gait frequencies. Matching or exploring deviations from these values provides a clear way to connect the mechanical design back to the biological reference.
- In a foldable mechanism with limited actuator stroke, stride length is tightly constrained by geometry; understanding this mapping is critical for designing compact but effective crawlers.
- For small robots with limited onboard energy and actuation authority, average speed and cost of transport are more informative performance metrics than peak velocity alone.

The final wording of the research question will be refined once the dynamic model and initial experimental results are available.

**[TODO: Insert finalized wording of research question on period/stride length and speed]**


## Background Research and Existing Work

Existing work on inchworm and caterpillar locomotion provides the biological and mechanical context for this project. Biomechanics studies of Manduca sexta describe how segments anchor and release in sequence, how body curvature evolves during a stride, and how ground‑reaction forces and frictional asymmetry contribute to net propulsion. These results provide realistic ranges for body length, stride length, gait period, curvature, and friction ratios that guide the design and scaling of the robot.

On the robotics side, several inchworm‑style mechanisms have been proposed, including modular rigid crawlers, cable‑driven climbers, and soft crawling robots. Many of these systems implement the same fundamental pattern: two or more anchors with alternating high/low friction states and a central actuation unit that drives contraction and extension. Analytical models of slider–crank‑based crawlers provide direct relationships between link geometry and stride distance, while more recent soft‑robotic designs emphasize friction modulation and energy efficiency.

Project Assignment 1 synthesized these sources into a set of target specifications and a kinematic mechanism concept for a foldable inchworm‑style crawler. :contentReference[oaicite:3]{index=3}


### Collecting Design Criteria

From the literature and the course context, the team is collecting design criteria along several dimensions, including:

- Locomotion performance: stride length relative to body length, average speed, and repeatability over multiple cycles.
- Stability and robustness: resistance to tipping, sensitivity to surface variations, and tolerance to small geometric or fabrication errors.
- Contact and friction: required friction ratio between anchored and sliding states, pad size and pressure, and surface compatibility.
- Manufacturability and integration: panel sizes, hinge patterns, assembly complexity, and compatibility with available actuation and control hardware.
- Sim‑to‑real alignment: parameters that can be measured experimentally (mass, stiffness, damping, friction, actuator characteristics) and directly mapped into the MuJoCo model. 

These criteria will be consolidated into a structured set of requirements that drive both the simulation models and the physical prototypes.

**[TODO: Insert detailed design criteria table/list here]**


## Key Information and Figures

Several key conceptual and design figures will be introduced throughout the site to make the mechanism, gait, and performance measures clear.

The first planned figure will be a schematic of the foldable inchworm robot showing the main components: front and rear rigid body sections, friction pads, internal linkage structure, and the primary actuator. This figure will help readers connect the abstract kinematic diagrams from Project Assignment 1 to the physical layout of the foldable mechanism.

<!-- TODO: Figure 1 – Schematic of the inchworm-foldable robot and its main components. :contentReference[oaicite:5]{index=5} -->

A second planned figure will illustrate the gait cycle as a sequence of configurations, including rear‑anchor/front‑slide, both‑anchored (looped posture), and front‑anchor/rear‑slide phases. The same figure can be used later to annotate sensor sites, pad states (anchored vs sliding), and key kinematic variables (e.g., body length, lift height).

<!-- TODO: Figure 2 – Motion sequence showing key gait phases (anchor–release, lift–drag, and reset). -->

To support the focus on period, stride length, and speed, a third figure will show representative trajectories or time histories from the MuJoCo simulations: body or pad position vs time over several cycles, highlighting how stride length and period combine into an average forward speed.

<!-- TODO: Figure 3 – Example trajectories illustrating period, stride length, and average speed from MuJoCo simulations. -->

Finally, a comparison figure will later juxtapose simulation snapshots and experimental images or video frames for the same gait parameters, emphasizing sim‑to‑real agreement (and discrepancy) in stride, timing, and posture.

<!-- TODO: Figure 4 – Side-by-side comparison of simulated and experimental gait for a representative parameter set. -->


## Goal Performance Metrics and Specifications

To evaluate the design and guide parameter studies, the project will use a small set of scalar performance metrics that can be computed in both simulation and experiment. These include, at minimum:

- Stride length per cycle and per body length.
- Gait period and average forward speed.
- Basic energetic measures such as work per cycle and cost of transport (where data permit).
- Robustness indicators such as variance in stride length over multiple cycles.
- Payload capability relative to robot weight, within the limits of the foldable structure and actuators.

These metrics will be linked to a specification table that records design‑scale parameters (body length, mass, target friction ratios), actuation constraints (stroke, force/torque), and allowable ranges for joint stiffness and damping in the MuJoCo model. The specification table will be updated as the design is refined and as additional measurements are collected.

**[TODO: Add finalized performance metrics and specifications table]**


## Project Status and Next Steps

At the current stage:

- Project Assignment 1 is complete. The team has selected the inchworm‑inspired candidate, performed a focused literature review, defined preliminary performance targets, developed a kinematic model and Jacobian for the proposed mechanism, and drafted an initial specifications table. :contentReference[oaicite:6]{index=6}
- A paper prototype and mechanism sketches have been created to validate the basic lift‑and‑drag motion and to check qualitative feasibility of the linkage topology within foldable fabrication constraints.
- The public website structure is in place using the AcademicPages Jekyll template, and this “About” page serves as the top‑level entry point for the project. 

Over the remainder of the semester, the planned next steps are:

- Dynamic modeling: upgrade the kinematic mechanism into a full dynamic MuJoCo model with realistic masses, joint compliance, damping, actuators, and contact/friction parameters.
- Parameter studies: use Python scripts to perform parameter sweeps (e.g., varying stroke, stiffness, friction coefficients) and compute performance metrics via MuJoCo sensors and state data.
- Hardware build: fabricate and assemble a foldable prototype using the course’s laminate workflow, integrate actuators and simple control, and refine the design based on observed behavior.
- Experimental campaign: collect motion and timing data (e.g., via phone video and tracking tools) across a range of gait and design parameters that match the simulation cases.
- Sim‑to‑real comparison: analyze how well the dynamic model predicts stride length, period, speed, and qualitative posture, and document sources of mismatch (e.g., friction, compliance, backlash).
- Documentation: update this website with figures, tables, and short video clips summarizing the modeling, parameter study, and experimental results, and link clearly to the MuJoCo model and analysis repository.

**[TODO: Cross-check and align this overview with final Assignment 2 report structure and grading rubric.  ]**