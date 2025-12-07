---
layout: archive
title: "Jupyter Notebook"
permalink: /notebook/
author_profile: true
redirect_from:
  - /resume
---

---

# Development of the Model

This section documents how the inchworm-inspired foldable robot was converted from the Assignment 1 kinematic concept into a dynamic MuJoCo model with compliance, actuators, and performance tracking, following the expectations of Project Assignment 2.

---

## 1.1 Robot Overview and Objectives

The robot is a Sarrus linkage inchworm crawler with two friction-modulated pads and a central body that retracts and extends to achieve lift–drag locomotion.

* **Biological inspiration:** looping, two-anchor gait with alternating front/rear grip.

* **This model’s purpose:**

  * Capture the robot’s planar dynamics (forward motion + arching).
  * Include joint stiffness/damping and actuator dynamics.
  * Provide a platform for parameter sweeps and sim-to-real comparison.


---

## 1.2 Model Structure: Bodies, Joints, and Contacts

The MuJoCo model is organized around a small set of rigid bodies and joints:

* **Rigid bodies:**

  * Front pad (foot 1)
  * Rear pad (foot 2)
  * Central body links 
* **Joints:**

  * Hinge joints at mid-body to create the Sarrus linkage.
  * Servo joints between pads and body segments.
* **Contacts and friction:**

  * Distinct friction coefficients for pad–ground vs body–ground found via experimentation

A simple summary table for the model elements:

| Element type | Name  | Description                             |
| ------------ | -------------- | --------------------------------------- | 
| body         | `first_left`, `first_right`, `second_left`, `second_right`     | Central body segments      | 
| body         | `foot1`   | Front foot with low friction states | 
| body         | `foot2`    | Rear pad with low friction states  | 
| joint        | `mid_left`, `mid_right`    | Main arching hinge (compliant)          | 
| joint        | `foot1` | Pad pivot                    | 
| joint        | `foot2`  | Pad pivot                     | 
| body         | `end_effector`      |  COM tracking site      | 
| geom         | `ground`       | Ground plane                            |

---

## 1.3 Compliance and Actuator Modeling

The dynamic model includes both structural compliance and actuator dynamics.

* **Joint compliance:**

  * Flexure hinges modeled as torsional springs and dampers:

    * Stiffness $$k_\theta$$ from Assignment 5 parameter identification. 
    * Damping $$b_\theta$$ tuned to match experimental decay.


* **Actuator model:**

  * Servo-like behavior modeled as **position actuators** with gains:

    * Proportional gain $$k_p$$ (position stiffness).
    * Derivative gain $$k_v$$ (effective actuator damping).
  * Command input is joint target angle following a periodic square wave gait pattern

* **Friction model:**

  * Different `friction` parameters for:

    * Foot geoms $$low (\mu_\text{tape})$$.
    * Base geoms $$high (\mu_\text{base})$$.
  * Values based on friction experiment (Assignment 5). 


---

## 1.4 Sensing and Performance Metric Definition


* **Site:**

  * The site on this model is the `end_effector`body in the middle of the "head" of the inchworm. This position allows us to track the position of this geom over time and plot it to see how it moves during the simulation.
* **Performance metric:**

  * Average forward speed:
    $$
    v_x = \frac{x_\text{final} - x_\text{initial}}{T_\text{sim}}
    $$
  * This variable makes sense as a metric of performance for the inchworm, as it demonstrates how effective it is at moving forward.

---

## 1.5 MuJoCo XML Template 

```python
# This block defines a parameterized MuJoCo XML template as a Python string.
# It will be formatted later with specific parameter values (stiffness, damping, friction, etc.).

import os
import mujoco
import pandas
import numpy as np
import math
import mediapy as media
import matplotlib.pyplot as plt
import imageio_ffmpeg
from scipy import signal

xml_template = '''
<mujoco>
    <default>
        <light castshadow="false" diffuse="1 1 1"/>
        <camera fovy="45"/>
    </default>
     <!--<option><flag contact="disable"/></option>-->
    <option integrator="RK4"/>
    <option timestep="1e-4"/>
    <option gravity="0 0 -9.81"/>
    <worldbody>
        <light name="top" pos="0 0 2"/>
        <light name="top1" pos="1 0 2"/>
        <light name="top2" pos="2 0 2"/>
        <light name="top3" pos="3 0 2"/>
        <light name="top4" pos="4 0 2"/>
        <camera name="triangle" pos="7 0 .75" euler="90 90 0"/>
        <camera name="triangle2" pos="-2 0 .05" euler="90 -90 0"/>
        <camera name="iso" pos="-2 -3 3" euler="45 -45 -30"/>
        <camera name="iso2" pos="10 -6 6" euler="45 45 30"/>
        <geom name="ground" type="plane" pos="0 0 -.25" size="15 15 .05" rgba=".5 .5 .5 1" contype="2" conaffinity="1"/>
        <body name="base_bot" pos="0 0 0" >
            <joint type="free"/>
            <geom type="box" size=".5 .5 .05" pos=".5 0 0" rgba="1 0 0 .25" mass=".1" contype="1" conaffinity="1" 
            friction="{mu_base} 0.0 0.0"/>
            <body name="foot1" pos="0 0 0">
                 <geom type="box" size=".1 .25 .05" pos=".15 0 0" rgba="0 1 0 1" mass=".1" contype="1" conaffinity="1"
                 friction="{mu_tape} 0.0 0.0"/>
                 <joint name="foot1" type="hinge" axis="1 0 0" pos=".25 0 0"/>
            </body>
        </body>
        <body name="base_left" pos="0 .25 .433" quat=".866 -.5 0 0">
            <joint type="free"/>
            <geom type="box" size=".5 .5 .05" pos=".5 0 0" rgba="1 0 0 .15" mass="{mass}" contype="1" conaffinity="1"/>
            <body name="first_left" pos="1 0 0" axisangle="0 1 0 -45">
                <joint name="first_left" type="hinge" axis="0 1 0" pos="0 0 0" range="-45 45" stiffness="{k_joint}" damping="{b_joint}"/>
                <geom type="box" size=".5 .5 .05" pos=".5 0 0" rgba="1 1 1 .25" mass="{mass}" contype="1" conaffinity="1"/>
                <body name="second_left" pos="1 0 0" axisangle="0 1 0 90">
                    <joint name="mid_left" type="hinge" axis="0 1 0" pos="0 0 0" range="-90 90" stiffness="{k_joint}" damping="{b_joint}"/>
                    <geom type="box" size=".5 .5 .05" pos=".5 0 0" rgba=".5 .5 .5 .25" mass="{mass}" contype="1" conaffinity="1"/>
                    <body name="tail_left" pos="1 0 0" axisangle="0 1 0 -45">
                        <joint name="last_left" type="hinge" axis="0 1 0" pos="0 0 0" range="-45 45" stiffness="{k_joint}" damping="{b_joint}"/>
                        <geom type="box" size=".5 .5 .05" pos=".5 0 0" rgba="1 0 0 .25" mass="{mass}" contype="1" conaffinity="1"/>
                        <body name="tail_bot" pos="0 .25 -.433">
                            <geom type="box" size=".5 .5 .05" pos=".5 0 0" rgba="1 0 0 .25" mass=".1" quat=".866 .5 0 0" 
                            friction="{mu_base} 0.0 0.0" contype="1" conaffinity="1"/>
                            <body name="end_effector" pos="1 -.375 .2165">
                                <geom type="sphere" size=".1" pos="0 0 0" rgba="1 1 1 1" mass=".00001" contype="1" conaffinity="1"/>
                            </body>
                            <body name="foot2" pos="0 -.25 -.433" quat=".866 .5 0 0">
                                <joint name="foot2" type="hinge" axis="1 0 0" pos=".55 .5 0"/>
                                <geom type="box" size=".1 .25 .05" pos=".75 .5 0" rgba="0 1 0 1" mass=".1" 
                                    contype="1" conaffinity="1" friction="{mu_tape} 0.0 0.0"/>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>
        <body name="base_right" pos="0 -.25 .433" quat=".866 .5 0 0">
            <joint type="free"/>
            <geom type="box" size=".5 .5 .05" pos=".5 0 0" rgba="1 0 0 .25" mass="{mass}" contype="1" conaffinity="1"/>
            <body name="first_right" pos="1 0 0" axisangle="0 1 0 -45">
                <joint name="first_right" type="hinge" axis="0 1 0" pos="0 0 0" range="-45 45" stiffness="{k_joint}" damping="{b_joint}"/>
                <geom type="box" size=".5 .5 .05" pos=".5 0 0" rgba="1 1 1 .25" mass="{mass}" contype="1" conaffinity="1"/>
                <body name="second_right" pos="1 0 0" axisangle="0 1 0 90">
                    <joint name="mid_right" type="hinge" axis="0 1 0" pos="0 0 0" range="-90 90" stiffness="{k_joint}" damping="{b_joint}"/>
                    <geom type="box" size=".5 .5 .05" pos=".5 0 0" rgba=".5 .5 .5 .25" mass="{mass}" contype="1" conaffinity="1"/>
                    <body name="tail_right" pos="1 0 0" axisangle="0 1 0 -45">
                        <joint name="last_right" type="hinge" axis="0 1 0" pos="0 0 0" range="-45 45" stiffness="{k_joint}" damping="{b_joint}"/>
                        <geom type="box" size=".5 .5 .05" pos=".5 0 0" rgba="1 0 0 .25" mass="{mass}" contype="1" conaffinity="1"/>
                    </body>
                </body>
            </body>
        </body>
    </worldbody>
    <equality>
        <weld body1="tail_bot" body2="tail_right"/>
        <weld body1="tail_left" body2="tail_right"/>
        <weld body1="base_bot" body2="base_left"/>
        <weld body1="base_bot" body2="base_right"/>
        <weld body1="base_left" body2="base_right"/>
    </equality>
    <actuator>
        <position name="mid_act" joint="mid_left"  kp="2" kv="{b_motor}"/>
        <position name="foot1" joint="foot1"  kp="{k_motor}" kv="{b_motor}"/>
        <position name="foot2" joint="foot2"  kp="{k_motor}" kv="{b_motor}"/>
    </actuator>
</mujoco>
'''

```

---

## 1.6 Python: Loading and Running a Single Simulation

```python
# This block is the creation of the simulation for the Mujoco model, at a specified period of 4 seconds. It is repeated for all periods tested, but the repeat code will not be shown for brevity. A .gif of the simulation will play at the bottom.

################## Period 4 seconds #########################
xml = xml_template.format(k_motor = .5, # found through trial and error
                          k_joint = 2.392e-3, # from assignment 5
                          b_motor = 4.084e-6, # from RC servo data collection, most likely way off
                          b_joint = 3.28e-6, # from assignment 5
                          mu_tape = .856, # from assignment 5
                          mu_base = .434, # from assignment 5
                          mass = .005) # from assignment 5

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
mujoco.mj_resetData(model, data)

theta = 45 # Initial middle hinge value, 0 = middle

data.qpos[15] = -math.pi*theta/180 # Initial conditions on hinge joints only
data.qpos[16] = 2*math.pi*theta/180
data.qpos[17] = -math.pi*theta/180
data.qpos[26] = -math.pi*theta/180
data.qpos[27] = 2*math.pi*theta/180
data.qpos[28] = -math.pi*theta/180

renderer = mujoco.Renderer(model)
      
wormAmp = np.deg2rad(90)
footAmp = -np.deg2rad(90)
freq = .25
period = 1/freq

frames = []
duration = 25 # (seconds)
framerate = 20 # (Hz)

eepos4 = []
time = []

while data.time < duration:

    data.ctrl[0] = wormAmp*(signal.square(freq*2*math.pi*(data.time+(period/4)), duty=.5)+1)/2
    data.ctrl[1] = footAmp*(signal.square(freq*2*math.pi*data.time, duty=.5)+1)/2
    data.ctrl[2] = footAmp*(signal.square(freq*2*math.pi*(data.time+(period/2)), duty=.5)+1)/2
    
    mujoco.mj_step(model, data)
    
    if len(frames) < data.time * framerate:
        renderer.update_scene(data, "iso2")
        pixels = renderer.render()
        frames.append(pixels)
        eepos4.append(data.body('end_effector').xpos[0])
        time.append(data.time)
        #print(100*len(frames)/(framerate*duration),"%")

media.set_ffmpeg(imageio_ffmpeg.get_ffmpeg_exe())
media.show_video(frames, fps = framerate)
```

---

![Gif of crawl at P4]()
<img src='..\assets\04_Results\inchworm_crawlP4.gif' width="600" height="400">  
*Figure 1. MuJoCo model schematic showing the inchworm crawling at a period of 4 seconds

### Repeated code for a period of 2 seconds

![Gif of crawl at P2]()
<img src='..\assets\04_Results\inchworm_crawlP2.gif' width="600" height="400">  
*Figure 2. MuJoCo model schematic showing the inchworm crawling at a period of 2 seconds

### Period of 1 second

![Gif of crawl at P1]()
<img src='..\assets\04_Results\inchworm_crawlP1.gif' width="600" height="400">  
*Figure 3. MuJoCo model schematic showing the inchworm crawling at a period of 1 second

### Period of .5 seconds

![Gif of crawl at P.5]()
<img src='..\assets\04_Results\inchworm_crawlP_5.gif' width="600" height="400">  
*Figure 4. MuJoCo model schematic showing the inchworm crawling at a period of .5 seconds

This sweep of the independent parameter highlights the optimized gait period in the simulation

---

# Plotting Simulation Results

Below is a simple plot of the x poisiton of the end effector on the inchworm over time, to see how it reacts for different gait periods

```python
plt.plot(time,eepos4)
plt.xlabel("Time [s]")
plt.ylabel("X position of end effector")
plt.title("Tracking End Effector Position over time. Period 4s")
plt.grid(True)
```

---
### Period of 4 seconds

![png of xpos at P4](..\assets\04_Results\SimP4.png)
*Figure 5. Tracking data of the inchworm crawling at a period of 4 seconds

### Period of 2 seconds

![png of xpos at P4](..\assets\04_Results\SimP2.png)
*Figure 6. Tracking data of the inchworm crawling at a period of 2 seconds

### Period of 1 second

![png of xpos at P4](..\assets\04_Results\SimP1.png)
*Figure 7. Tracking data of the inchworm crawling at a period of 1 second

### Period of .5 seconds

![png of xpos at P4](..\assets\04_Results\SimP_5.png)
*Figure 8. Tracking data of the inchworm crawling at a period of .5 seconds

### Combination plot of all periods

![png of xpos at P4](..\assets\04_Results\SimPAll.png)
*Figure 9. Tracking data of the inchworm crawling at all periods for comparison


---


---

# Results, Summary, and Discussion

This section is where you will present and interpret both simulation and experimental results, compare them, and discuss the sim-to-real gap. It mirrors the expectations of the Project Assignment 2 report and video. 

---

## 3.1 Simulation Results

Summarize the key trends from the parameter sweep:

* How does the performance metric e.g., $$v_x$$ change with the design parameter?
* Is there a clear optimum or plateau?
* Are there regions where the gait fails (e.g., no net forward motion, excessive slip, instability)?

**Suggested bullets:**

* **Trend:**

  * TODO: Describe whether performance increases, decreases, or has a peak vs. parameter.
* **Optimal region:**

  * TODO: State approximate best parameter value(s) and corresponding performance.
* **Qualitative motion:**

  * TODO: Note anything unusual in body posture or slip at different settings.

**TODO:** Write 1–2 short paragraphs interpreting your simulation-only plots.

---

![Simulation snapshots of the inchworm model at different phases of the gait](path/to/sim_snapshots.png)
*Figure 3. Simulation snapshots over one gait cycle, showing lift–drag locomotion for a representative parameter setting. TODO: render and capture images from MuJoCo.*

---

## 3.2 Experimental Methods and Data Processing

Describe how you collected physical data for the same parameter range:

* **Experimental variable:** same parameter as in simulation (e.g., gait frequency, joint stiffness via hinge thickness, etc.).
* **Setup:**

  * Folded robot, actuation via servo/ESP32 control.
  * Flat surface with known friction properties.
  * Overhead or side-view camera for tracking.
* **Measurement approach:**

  * Obtain time–position data using Tracker (video analysis), IMU, or other sensors.
  * Extract front pad or COM x-position vs. time.
  * Compute average speed, stride length, etc., analogously to simulation.

```python
# This block shows an example of loading experimental tracking data from CSV
# and computing average forward speed in the same way as the simulation.

import pandas as pd

def load_experimental_data(csv_path, x_column="x", t_column="t", x_offset=0.0):
    """
    Load experimental tracking data from a CSV file.

    Parameters
    ----------
    csv_path : str
        Path to the CSV file (e.g. exported from Tracker).
    x_column : str
        Column name for x-position.
    t_column : str
        Column name for time.
    x_offset : float
        Optional offset to align x with simulation origin.

    Returns
    -------
    t_exp, x_exp : np.ndarray
        Time and x-position arrays.
    """
    df = pd.read_csv(csv_path)
    t_exp = df[t_column].to_numpy()
    x_exp = (df[x_column].to_numpy() + x_offset)
    return t_exp, x_exp

def compute_average_speed_from_data(t, x):
    """Compute average forward speed from experimental or simulated data."""
    x_start = x[0]
    x_end = x[-1]
    t_end = t[-1]
    return (x_end - x_start) / t_end

# Example usage for one trial (TODO: adapt names and paths)
# t_exp, x_exp = load_experimental_data("trial_f0p35.csv", x_column="x", t_column="t")
# vx_exp = compute_average_speed_from_data(t_exp, x_exp)
# print(f"Experimental average speed = {vx_exp:.4f} m/s")
```

**TODO:**

* Document how many trials you ran per parameter value.
* Note sources of noise (camera jitter, lighting, tracking errors) and how you mitigated them (e.g., smoothing, discarding outliers).

---

![Photo of the physical inchworm robot on the test surface, with tracking markers visible](path/to/experimental_setup.png)
*Figure 4. Experimental setup used to record inchworm locomotion: foldable robot, tracking markers, and camera geometry. TODO: insert actual photo.*

---

## 3.3 Sim-to-Real Comparison

Compare simulation and experiment for the **same** parameter values:

* Overlay performance vs parameter plots (simulation vs experiment).
* Wherever possible, overlay time–position traces for representative cases.

```python
# This block illustrates how to overlay simulation and experimental performance
# as a function of the same design parameter.

# Example: suppose you have experimental average speeds stored in arrays
# param_vals_exp and vx_exp, aligned with param_vals_sim.

plt.figure()
plt.plot(param_vals_sim, vx_sim, "o-", label="Simulation")
# TODO: load or compute vx_exp and param_vals_exp from CSVs
# plt.plot(param_vals_exp, vx_exp, "s--", label="Experiment")

plt.xlabel("Gait frequency [Hz]")  # or other parameter
plt.ylabel("Average forward speed v_x [m/s]")
plt.title("Simulation vs Experiment: Performance vs parameter")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
```

**Discussion prompts (fill in as short paragraphs):**

* **Qualitative agreement:**

  * TODO: Describe whether the general trend (increasing, decreasing, peak) is consistent between simulation and experiment.
* **Quantitative differences:**

  * TODO: Note approximate percentage differences at key parameter values.
* **Failure modes / anomalies:**

  * TODO: Discuss any conditions where the robot behaves differently than predicted (e.g., stalling, sideways slip, saturation).

---

![Overlay of simulation and experimental average speed vs. parameter](path/to/sim_vs_exp_speed.png)
*Figure 5. Comparison of average forward speed from simulation and experiment across the parameter range. TODO: generate this figure once both datasets are available.*

---

## 3.4 Sources of Error and Model Limitations

Summarize the likely reasons for discrepancies between simulation and experiment, tying back to modeling choices:

* **Geometry and mass properties:**

  * Simplified link shapes vs real folded shell geometry.
  * Unmodeled mass distribution (e.g., wires, connectors).
* **Compliance modeling:**

  * Single torsional spring vs distributed flexure behavior.
  * Nonlinear stiffness or hysteresis not included.
* **Contact and friction:**

  * Coulomb friction with fixed coefficients vs real material behavior (velocity dependence, stick–slip).
  * Sensitivity to pad normal force and surface roughness.
* **Actuator and control:**

  * Idealized position actuator vs real servo limits (backlash, saturation, deadband).
  * Timing and phase errors in microcontroller code.
* **Measurement noise:**

  * Camera tracking, frame rate limitations, IMU drift.

**TODO:** Add 1–2 paragraphs here discussing which of these are dominant for your robot and how they affect the results.

---

## 3.5 Conclusions and Future Work

Wrap up with a brief, structured summary:

* **Key findings from simulation:**

  * TODO: e.g., “There exists an optimal gait frequency around X Hz that maximizes speed under the modeled friction and compliance.”
* **Key findings from experiment:**

  * TODO: e.g., “Experimentally, the best performance occurs at Y Hz, with maximum speed of Z mm/s.”
* **Sim-to-real alignment:**

  * TODO: comment on whether the model is “good enough” for design iteration.
* **Proposed improvements (V2):**

  * More accurate geometry and mass modeling.
  * Multi-joint pseudo-rigid-body model for the flexure.
  * Better friction characterization and implementation.
  * Closed-loop feedback control in both simulation and experiment.

Finally, note where this content will appear on the project website and in the video:

* **Website:**

  * Model description and parameter study in Assignment 2 section.
  * Embedded plots and GIFs from both simulation and experiment.
* **Video:**

  * Short explanation of the model and parameter choice.
  * Side-by-side clips of simulation and physical robot for a few parameter values. 
