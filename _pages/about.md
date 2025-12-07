---
permalink: /
title: "Home"
author_profile: true
redirect_from: 
  - /about/
  - /about.html
---
<img src='..\assets\01_Home\Inch_Worm.jpg'>  

## Team and Project Overview

This site documents the semester project for [RAS 557: Foldable Robotics](https://foldable-robotics.github.io/)  
Fall 2025,  
Instructor: Daniel M. Aukes

The course focuses on bio‑inspired terrestrial locomotion, foldable fabrication methods, and simulation‑driven design.

The project investigates an inchworm‑inspired crawling robot based on the locomotion of [Manduca sexta larvae](https://en.wikipedia.org/wiki/Manduca_sexta).  

The robot uses rigid foldable linkages and friction-modulated pads to realize a planar, two-anchor lift-and-drag gait whose parameters can be measured, simulated, and systematically varied.

The project team consists of:

- [Sai Srinivas Tatwik Meesala](https://tatwik19.github.io/)
- [Colin Fricke](https://www.linkedin.com/in/colin-fricke)
- Nathan Vairora  

The current work is organized around the following research question:

> **How does gait timing (period) influence the achievable speed and robustness of an inchworm-inspired foldable crawler, in both simulation and experiment?**

This home page summarizes how that question connects to the team’s goals, the biological and robotic background, and the design–build–simulate–compare workflow of Project Assignment 2.

## Team Goals

### Candidate Identification

The project takes the Manduca sexta larva (geometrid inchworm‑type crawler) as the primary biological inspiration. Manduca uses a characteristic two‑anchor looping gait: posterior prolegs anchor while the anterior body lifts and extends; then the anterior prolegs anchor while the posterior body is pulled forward, producing large body‑length‑scaled strides with relatively simple control. 

The robotic analog is a planar, inchworm‑inspired crawler with:

* Two friction‑modulated pads (front and rear) acting as anchors.
* A rigid, foldable linkage chain connecting the pads.
* A single effective actuator that shortens and lengthens the body, analogous to the biological contraction–extension cycle.
* Passive or lightly compliant joints that permit lift‑and‑drag motion without large out‑of‑plane deformation. 

This mechanism is designed so that key geometric and contact parameters (link lengths, pad spacing, friction coefficients) are explicitly modeled and can be matched between simulation and hardware.

### Intent of Choice

Inchworm‑like crawlers were chosen because they realize terrestrial locomotion through **friction modulation** and **body shape change**, rather than wheels or multi‑leg coordination. Prior work on Manduca sexta biomechanics and inchworm‑style robots demonstrates: stride lengths on the order of 0.3–0.5 body lengths, orientation‑robust gaits, and useful ground‑reaction force distributions for climbing and horizontal motion [1]–[4].

Most existing inchworm robots, however, rely on smart materials (e.g., SMA wires) or soft bodies whose internal mechanics are difficult to parameterize [4]–[6]. In contrast, this project explicitly targets:

* A **rigid‑link, foldable prototype** compatible with laser‑cut laminates and the foldable fabrication workflow taught in RAS 557.
* A geometry and actuation layout that are **easy to encode in MuJoCo**, including link inertias, joint compliance, and friction parameters.

This choice supports systematic design variations and sim‑to‑real comparisons instead of one‑off demonstrations.

### Research Question Rationale

The updated research question places **gait timing (period)**—rather than geometry or material choice—as the primary design variable:

> **How does gait timing (period) influence the achievable speed and robustness of an inchworm-inspired foldable crawler, in both simulation and experiment?**

#### Scope

The study focuses on a tightly defined problem:

* **Locomotion mode:** quasi‑1D, planar crawling along a straight line (no turning or climbing) on a flat substrate.
* **Scale:** nominal body length $$L \approx 100\ \text{mm}$$, consistent with the Assignment 1 design and performance metrics. 
* **Mechanism:** a single inchworm‑type gait family with two pads and a rigid linkage chain; link geometry, mass properties, and hinge layout are held fixed during parameter sweeps. 
* **Design variable:** gait timing (period and related timing parameters such as stance/swing partition and pad‑anchor duty cycle) is varied while actuator stroke, friction coefficients, and robot mass remain constant.
* **Environment:** flat surfaces with controlled friction ratio between “anchor” and “slide” states (target friction ratio $$\mu_{\text{anchor}} / \mu_{\text{slide}} \geq 3$$), using the same pad materials and substrates in both simulation and experiments. 
* **Sim–real alignment:** the same physical robot is modeled in MuJoCo as a dynamic system with joint stiffness/damping and a realistic actuator model, following the Project Assignment 2 specification.

#### Impact

Understanding how gait period affects **speed** and **robustness** is relevant to applications where inchworm‑inspired modules may traverse confined or cluttered spaces: cable and pipe inspection, structural health monitoring, and deployable sensor networks [4]–[6].  

For a fixed stroke and friction ratio, average forward speed can be expressed as:

$$
v = \frac{\Delta x_{\text{cycle}}}{T_{\text{gait}}}
$$

where $$\Delta x_{\text{cycle}}$$ is the net displacement per cycle and $$T_{\text{gait}}$$ is the gait period. Too fast a period may cause slip, loss of anchoring, or actuator saturation; too slow a period wastes available actuation bandwidth. Robustness—in this context—is defined in terms of:

* Repeatable forward progress across cycles (low variance in $$\Delta x_{\text{cycle}}$$).
* Tolerance to friction variation and minor geometry/fabrication errors.
* Absence of failure modes such as double‑slip or incomplete anchoring.

By mapping performance vs. timing in both simulation and hardware, the project aims to produce simple **design rules** for friction‑modulated foldable crawlers (e.g., recommended period ranges for given stroke and friction).

#### Team Fit

The research question aligns with the team’s combined skills:

* **Mechanism and CAD design:** translating Manduca‑like body kinematics into a foldable linkage with appropriate DOFs and range of motion. 
* **MuJoCo and Python:** building a dynamic model with compliant joints, actuator dynamics, and contact, then scripting parameter sweeps over gait period and logging sensor data.
* **Experiment design and data analysis:** using phone‑based motion capture, friction tests, and timing control (e.g., microcontroller‑based gait timing) to measure speed and robustness across timing conditions. 

These capabilities support the full Assignment 2 workflow: **design, build, simulate, and validate** the robot under controlled gait timing variations.

#### Topic Fit

The topic sits at the intersection of key Foldable Robotics course themes: 

* **Foldable mechanisms:** converting a continuum inchworm body into a compact linkage with laser‑cut links and laminated hinges.
* **MuJoCo simulation:** using constraint‑based dynamics, contact modeling, and joint compliance to capture the interplay of timing, friction, and inertia.
* **Biomechanics and locomotion:** grounding performance metrics (stride‑to‑body‑length ratio, friction ratio, curvature limits) in Manduca sexta data and inchworm‑robot literature [1]–[7]. 
* **System identification and optimization:** measuring real actuator and friction behavior, fitting parameters, and comparing predicted vs. observed performance as gait timing is varied.

This framing ensures that the project remains both biologically inspired and technically aligned with the course’s modeling, fabrication, and analysis goals.

## Background Research

### Existing Papers

Assignment 1 reviewed biomechanics and robotics literature relevant to inchworm‑style locomotion and friction‑modulated crawlers.   Key contributions include:

* **Manduca kinematics and orientation‑robust gait** – van Griethuijsen and Trimmer [1] measured full‑body 3D kinematics of Manduca sexta crawling horizontally and vertically, showing similar proleg timing across orientations. For this project, their reported stride period and stride‑to‑body‑length ratios provide realistic targets for gait period and net displacement per cycle.
* **Ground reaction forces and friction roles** – Lin and Trimmer [2] used multi‑contact force plates to quantify ground‑reaction forces under each proleg, showing that anterior prolegs primarily generate forward thrust while posterior prolegs often act as drag anchors. The associated friction ratios motivate the target difference between “anchor” and “slide” pad behavior.
* **Beam‑bending model of inchworm curvature** – Plaut [3] modeled the inchworm body as a flexible beam undergoing large deflection, deriving relationships between curvature, arc length, and forward displacement. These results motivate curvature and “lift height” limits for the linkage‑based robot.
* **Inchworm‑like rigid crawlers and climbers** – Wang et al. [4] and Li et al. [5] present rigid or cable‑driven climbing robots that mimic inchworm gaits using alternating grippers and variable body length. Their actuator layouts and reported speeds demonstrate that discrete linkages can reproduce inchworm‑style locomotion at practical scales.
* **Soft crawler review** – Pan et al. [6] survey soft robotic crawlers, identifying inchworm gaits as two‑anchor peristaltic mechanisms dependent on friction anisotropy and anchoring stability. Their summary of design parameters (friction ratio, stride fraction, gait frequency) guides selection of performance metrics.
* **Slider–crank inchworm line robot** – Wu [7] derives a slider‑crank kinematic model for a line‑inspection robot that emulates inchworm motion. The closed‑form relation between geometry, crank offset, and stride length informs the back‑solve of linkage parameters and actuator stroke in this project.

Together, these works indicate that gait timing, friction ratio, and stroke amplitude jointly determine speed and stability in inchworm‑type locomotion, motivating a focused study of timing for a fixed geometry and friction configuration. 

### Key Information

From Assignment 1, several quantitative and qualitative design choices directly inform the new timing‑focused study: 

* **Scale and mass:** nominal body length $$L \approx 100\ \text{mm}$$, target mass $$m \approx 50\ \text{g}$$ including structure and actuator.
* **Stride and speed targets:** stride length $$S \approx 0.25L \approx 25\ \text{mm}$$ and average speed on the order of $$v \approx 10\ \text{mm/s}$$, consistent with Manduca‑inspired scaling and practical actuation limits.
* **Friction design:** two friction states with typical design values $$\mu_{\text{anchor}} \approx 0.8$$ and $$\mu_{\text{slide}} \approx 0.2$$, giving $$\mu_{\text{anchor}}/\mu_{\text{slide}} \geq 3$$–4. Pad materials and normal loads are chosen to realize this ratio on accessible lab surfaces.
* **Mechanism concept:** two pads connected by a rigid, planar linkage driven by a single effective linear stroke, realizing a lift‑and‑drag gait with clear stance/swing phases at each pad.
* **Performance metrics:** net forward displacement per cycle, average speed, thrust force, joint torques, and mechanical work were already estimated from force–friction and slider–crank analyses.

In Assignment 2, the **primary new independent variable** becomes **gait period** and allied timing parameters, while these geometric and frictional quantities are held fixed to isolate temporal effects on speed and robustness.

### Key Figures

The Assignment 1 report contains several figures that should be referenced on the website to connect biological motion, analytical models, and the proposed mechanism. 

1. **Biological inchworm gait schematic / beam‑bending model**

   ![Biological inchworm gait schematic](..\assets\01_Home\biological-gait.jpg)
   *Figure 1: Schematic of Manduca‑style two‑anchor locomotion and its beam‑bending model, illustrating how body curvature and anchor sequencing generate net forward displacement per cycle [3]. This figure motivates the linkage’s target curvature and lift height, which in turn constrain feasible gait periods before slip and instability occur.*

2. **Manduca kinematics and gait phases**

   ![Manduca crawling kinematics](..\assets\01_Home\manducakinematics.jpg)

   *Figure 2: Representative Manduca crawling kinematics showing stance and swing phases for each proleg pair [1]. The timing of these phases relative to the overall cycle informs the definition of gait period and anchor duty factors in the robot’s control parameterization.*

3. **Slider–crank / inchworm line‑robot kinematic sketch**

   ![Slider–crank inchworm linkage](..\assets\01_Home\slidercrank.jpg)

   *Figure 3: Kinematic sketch of a slider–crank‑based inchworm line robot [7]. The relation between geometric parameters and stride length, combined with fixed actuator stroke, constrains the range of gait periods that can be tested without exceeding actuator force or joint torque limits.*

4. **Foldable linkage concept for the inchworm crawler**

   ![Foldable inchworm linkage concept](..\assets\01_Home\linkageconcept.jpg)

   *Figure 4: Simplified kinematic diagram of the foldable rigid‑link crawler, including front and rear pads, linkage chain, and actuation stroke. This figure is central for explaining how gait timing (period) is implemented in both simulation and hardware, and how anchor timing is mapped to pad friction states.*

These figures collectively provide the conceptual bridge from biological gait to rigid‑link mechanism and clarify why gait timing is a meaningful, controllable parameter in both simulation and experiment.

### Project Novelty

Relative to prior literature and Assignment 1, this project is novel in three main ways: 

1. **Focusing on gait timing (period) as the primary design variable**
   While many inchworm‑style robots emphasize geometry, friction modulation hardware, or actuation technology [4]–[7], this project fixes those aspects and systematically varies gait period and phase relationships. This isolates how timing alone trades off average speed vs. robustness (slip, failed steps, and sensitivity to friction changes).

2. **Direct sim–to–real comparison with a shared, foldable rigid‑link platform**
   A single foldable crawler design is implemented both as a detailed MuJoCo model and as a physical laminate prototype. Joint stiffness/damping, actuator dynamics, and pad friction are measured or estimated and then encoded in the simulation. The same gait timing parameterization (period, pad duty factors) is applied in both domains to enable quantitative comparisons of speed, stride length, and failure rates as timing is varied.

3. **Mechanism transparency and parameter identifiability**
   Unlike soft or SMA‑driven inchworm robots, the foldable rigid‑link design offers well‑defined link inertias, joint axes, and contact geometries, making model parameters identifiable from straightforward experiments (mass measurement, friction tests, hinge characterization). This transparency supports a **closed loop** of: literature‑informed specification → Assignment 1 kinematics and performance estimates → dynamic modeling and timing sweeps (Assignment 2) → physical experiments and sim–real comparison.

By centering the question of **“What gait periods actually work best for a given foldable crawler?”**, the project contributes timing‑oriented design guidance for friction‑modulated, inchworm‑inspired robots that extends beyond a single mechanism instance.

## References

Numbers correspond to the references used in the Assignment 1 report; full bibliographic details are provided there. 

[1] N. P. L. Griethuijsen and B. A. Trimmer, “Crawling kinematics of Manduca sexta larvae on horizontal and vertical substrates,” *J. Exp. Biol.*, 2009.

[2] H. T. Lin and B. A. Trimmer, “Multi‑contact ground reaction forces of crawling Manduca sexta,” *J. Exp. Biol.*, 2010 (with 2011 corrigendum).

[3] R. H. Plaut, “Large deflection model of inchworm locomotion using a flexible beam,” *Int. J. Non‑Linear Mech.*, 2015.

[4] X. Wang *et al.*, “A mini‑modular climbing caterpillar robot with inchworm‑like gait,” *Prog. Nat. Sci.*, 2009.

[5] Y. Li *et al.*, “An inchworm‑like climbing robot based on cable‑driven grippers,” 2024.

[6] M. Pan *et al.*, “Bio‑inspired soft crawling robots: mechanisms, actuation, and control,” *Adv. Sci.*, 2025.

[7] F. Wu, “Kinematic analysis of a line robot based on inchworm biomimicry,” 2024.
