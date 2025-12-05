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

This section documents how the inchworm-inspired foldable robot was converted from the Assignment 1 kinematic concept into a dynamic MuJoCo model with compliance, actuators, and sensors, following the expectations of Project Assignment 2.

---

## 1.1 Robot Overview and Objectives

The robot is a Manduca sexta–inspired inchworm crawler with two friction-modulated pads and a central body that arches and extends to achieve lift–drag locomotion.

* **Biological inspiration:** looping, two-anchor gait with alternating front/rear grip.
* **Scale and target metrics (from Assignment 1, to be confirmed):**

  * Body length $L \approx 100\,\text{mm}$  
  * Target stride $S \approx 0.25L$
  * Target average speed $v \approx 10\ \text{mm/s}$
  * Friction ratio $\mu_\text{anchor} / \mu_\text{slide} \gtrsim 3$ 
* **This model’s purpose:**

  * Capture the robot’s planar dynamics (forward motion + arching).
  * Include joint stiffness/damping and actuator dynamics.
  * Provide a platform for parameter sweeps and sim-to-real comparison.

**TODO:** Replace the bullet list above with the final numbers and language from your Assignment 1 report.

---

## 1.2 Model Structure: Bodies, Joints, and Contacts

The MuJoCo model is organized around a small set of rigid bodies and joints:

* **Rigid bodies:**

  * Front pad (foot A)
  * Rear pad (foot B)
  * Central body links (e.g., 2–3 segments forming an arch)
* **Joints:**

  * Hinge joints at mid-body to create arching.
  * Optional hinge joints between pads and body segments.
* **Contacts and friction:**

  * Distinct friction coefficients for pad–ground vs body–ground.
  * Contact geometry chosen to approximate real pad sizes.

A simple summary table for the model elements:

| Element type | Name (example) | Description                             | Notes / TODO                |
| ------------ | -------------- | --------------------------------------- | --------------------------- |
| body         | `body_mid`     | Central body segment(s)                 | TODO: length, mass          |
| body         | `foot_front`   | Front pad with high/low friction states | TODO: size, friction values |
| body         | `foot_rear`    | Rear pad with high/low friction states  | TODO                        |
| joint        | `mid_hinge`    | Main arching hinge (compliant)          | TODO: stiffness, damping    |
| joint        | `foot_front_h` | Pad pivot (if used)                     | optional                    |
| joint        | `foot_rear_h`  | Pad pivot (if used)                     | optional                    |
| site         | `ee_site`      | “End-effector” / COM tracking site      | used for sensors            |
| geom         | `ground`       | Ground plane                            | base friction               |

**TODO:** Fill in the geometric and inertial parameters once CAD and mass measurements are finalized.

---

## 1.3 Compliance and Actuator Modeling

The dynamic model includes both structural compliance and actuator dynamics.

* **Joint compliance:**

  * Flexure hinges modeled as torsional springs and dampers:

    * Stiffness $k_\theta$ from Assignment 5 parameter identification. 
    * Damping $b_\theta$ tuned to match experimental decay.
  * Represented in MuJoCo via `joint` attributes (`springref`, `stiffness`, `damping`) or via `tendon` springs.

* **Actuator model:**

  * Servo-like behavior modeled as **position actuators** with gains:

    * Proportional gain $k_p$ (position stiffness).
    * Derivative gain $k_v$ (effective actuator damping).
  * Command input is joint target angle following a periodic gait pattern (e.g., square/triangular wave).

* **Friction model:**

  * Different `friction` parameters for:

    * Pad geoms $high (\mu_\text{anchor})$.
    * Base/body geoms $low (\mu_\text{slide})$.
  * Values based on friction experiment (Assignment 5) and/or tracker-based tests. 

**TODO:**

* Insert the final values of $ (k_\theta), (b_\theta), (k_p), (k_v) $, and friction coefficients once parameter identification is complete.
* Note which joint(s) use compliant vs rigid modeling.

---

## 1.4 Sensing and Performance Metric Definition

To support parameter studies, the model must expose measurable signals.

* **Sites and sensors:**

  * A `site` is attached to either:

    * The end-effector body (e.g., front pad) or
    * Approximate center-of-mass body.
  * A `framepos` or `framevel` sensor is used to read the position/velocity of this site.
* **Performance metric (example):**

  * Average forward speed:
    $$
    v_x = \frac{x_\text{final} - x_\text{initial}}{T_\text{sim}}
    $$
  * Optionally: distance per cycle, cost of transport, or power-per-distance.

---

## 1.5 MuJoCo XML Template (Parameterizable)

Below is a **sketch** of a parameterized XML template (in Python) for your model. It shows how to expose design parameters such as joint stiffness, damping, and friction. The geometry is deliberately simplified; adapt it to your final linkage design.

```python
# This block defines a parameterized MuJoCo XML template as a Python string.
# It will be formatted later with specific parameter values (stiffness, damping, friction, etc.).

xml_template = r"""
<mujoco>
  <option integrator="RK4" timestep="0.0005" gravity="0 0 -9.81"/>

  <default>
    <geom rgba="0.8 0.8 0.8 1" condim="3"/>
    <joint damping="{joint_damping}" stiffness="{joint_stiffness}"/>
  </default>

  <worldbody>
    <!-- Ground plane -->
    <geom name="ground" type="plane" pos="0 0 0" size="5 5 0.1"
          friction="{mu_base} 0 0"/>

    <!-- Rear pad (anchor/slide pad) -->
    <body name="foot_rear" pos="0 0 0.01">
      <joint name="foot_rear_free" type="free"/>
      <geom name="rear_pad" type="box" size="0.02 0.01 0.005"
            friction="{mu_pad} 0 0" rgba="0 0.6 0 1"/>
    </body>

    <!-- Central body and front pad; simplified 2-link arch -->
    <body name="body_mid" pos="0.05 0 0.02">
      <joint name="mid_hinge" type="hinge" axis="0 1 0" pos="0 0 0"
             springref="0" limited="true" range="-1.57 1.57"/>
      <geom name="mid_link" type="box" size="0.05 0.01 0.005"
            rgba="0.8 0 0 0.3"/>

      <!-- Front link segment -->
      <body name="body_front" pos="0.1 0 0">
        <geom name="front_link" type="box" size="0.05 0.01 0.005"
              rgba="0.8 0 0 0.3"/>

        <!-- Front pad -->
        <body name="foot_front" pos="0.05 0 0">
          <geom name="front_pad" type="box" size="0.02 0.01 0.005"
                friction="{mu_pad} 0 0" rgba="0 0.6 0 1"/>
        </body>
      </body>

      <!-- Site used to measure end-effector / COM position -->
      <site name="ee_site" pos="0.15 0 0" size="0.005"/>
    </body>
  </worldbody>

  <!-- Sensors for position and velocity at ee_site -->
  <sensor>
    <framepos name="ee_pos" site="ee_site"/>
    <framevel name="ee_vel" site="ee_site"/>
  </sensor>

  <!-- Actuators controlling the mid hinge (servo-like) -->
  <actuator>
    <position name="mid_act" joint="mid_hinge"
              kp="{actuator_kp}" kv="{actuator_kv}"/>
  </actuator>
</mujoco>
"""
```

**TODO:**

* Replace the simple 2-link model above with your final Manduca-style linkage (Sarrus/four-bar hybrid).
* Add additional joints and actuators if your mechanism uses more than one DOF.
* Incorporate any weld constraints or equality constraints used in your Assignment 1 model.

---

## 1.6 Python: Loading and Running a Single Simulation

```python
# This block loads the parameterized MuJoCo model and runs a single simulation.
# It demonstrates how to:
# 1) Format the XML with specific parameter values,
# 2) Create the model and data structures,
# 3) Run the time-stepping loop while applying a simple gait command,
# 4) Record end-effector position over time for later analysis.

import mujoco
import numpy as np
import math
import matplotlib.pyplot as plt

def build_model_xml(joint_stiffness, joint_damping,
                    mu_pad, mu_base,
                    actuator_kp, actuator_kv):
    """Return a formatted XML string with the given model parameters."""
    xml = xml_template.format(
        joint_stiffness=joint_stiffness,
        joint_damping=joint_damping,
        mu_pad=mu_pad,
        mu_base=mu_base,
        actuator_kp=actuator_kp,
        actuator_kv=actuator_kv,
    )
    return xml

def run_single_simulation(xml_string,
                          gait_frequency_hz,
                          duration_s=10.0,
                          worm_amp_deg=45.0):
    """
    Run one simulation with a given gait frequency and return time + ee_site x-position.
    The actuator is commanded as a square-wave position pattern.

    Parameters
    ----------
    xml_string : str
        Complete MuJoCo XML description of the robot.
    gait_frequency_hz : float
        Frequency of the arching motion (Hz).
    duration_s : float
        Total simulation time (seconds).
    worm_amp_deg : float
        Peak amplitude of mid-hinge rotation (degrees).
    """
    # Create the model and data
    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    # Pre-compute control parameters
    worm_amp_rad = np.deg2rad(worm_amp_deg)
    omega = 2.0 * math.pi * gait_frequency_hz

    # Storage for results
    time_history = []
    x_history = []

    # Main simulation loop
    while data.time < duration_s:
        t = data.time

        # Simple square-wave gait: +/- worm_amp_rad
        # duty cycle 50%, zero-mean, phase aligned with time
        phase = math.sin(omega * t)
        target_angle = worm_amp_rad * np.sign(phase)

        # First actuator (mid_act) is assumed to be index 0 in data.ctrl
        data.ctrl[0] = target_angle

        # Advance simulation by one step
        mujoco.mj_step(model, data)

        # Record time and x-position of ee_site
        ee_pos = data.site('ee_site').xpos  # 3D position of the site
        time_history.append(t)
        x_history.append(ee_pos[0])        # x-component

    return np.array(time_history), np.array(x_history)

# Example usage (placeholder values for parameters)
# TODO: Replace with your final parameters from Assignment 5 and build script.
xml = build_model_xml(
    joint_stiffness=0.3,   # [N·m/rad] placeholder
    joint_damping=0.02,    # [N·m·s/rad] placeholder
    mu_pad=0.8,            # anchor friction (placeholder)
    mu_base=0.2,           # sliding friction (placeholder)
    actuator_kp=2.0,       # actuator stiffness (placeholder)
    actuator_kv=0.01       # actuator damping (placeholder)
)

t_sim, x_sim = run_single_simulation(
    xml_string=xml,
    gait_frequency_hz=0.35,
    duration_s=10.0,
    worm_amp_deg=45.0
)

# Quick diagnostic plot of end-effector x-position vs time
plt.figure()
plt.plot(t_sim, x_sim)
plt.xlabel("Time [s]")
plt.ylabel("End-effector x [m]")
plt.title("Single Simulation: End-effector x-position vs time")
plt.grid(True)
plt.tight_layout()
plt.show()
```

**TODO:**

* Check that the motion qualitatively matches your intended gait (lift–drag pattern).
* Adjust the gait command (e.g., phase shifts, duty cycles, additional actuators) to match your experimental controller.
* Once stable, freeze these as your “baseline” model parameters.

---

![Screenshot of the MuJoCo inchworm model highlighting bodies, joints, and contact pads](path/to/model_schematic.png)
*Figure 1. MuJoCo model schematic showing the inchworm-inspired crawler, body segments, and friction pads. TODO: capture an actual screenshot from the MuJoCo viewer.*

---

# Model Optimization / Parameter Study

This section describes how the dynamic model is used to explore how performance depends on key design parameters, and how the same variation is implemented experimentally.

---

## 2.1 Design Parameters and Performance Metric

**Candidate design parameters** (choose one primary, optionally a secondary):

* **Gait frequency / cycle period**:

  * Parameter: (f) [Hz] or period $T = 1/f$.
  * Motivation: affects stride length and slip.
* **Joint stiffness (k_\theta)**:

  * Parameter: flexure stiffness (N·m/rad).
  * Motivation: trade-off between energy storage, controllability, and arching amplitude.
* **Pad friction $\mu_\text{anchor} / \mu_\text{slide}$**:

  * Parameter: friction ratio via material choice or pad normal load.
  * Motivation: determines net thrust vs slip.

**Primary performance metric (example):**

* Average forward speed:  
  $$
  v_x = \frac{x_\text{end} - x_\text{start}}{T_\text{sim}}
  $$
* Secondary metrics (if needed):

  * Distance per cycle.
  * Mechanical work per distance (cost of transport).
  * Maximum payload while maintaining (v_x) within tolerance.

**TODO:**

* Explicitly state the final parameter(s) you chose to vary and why.
* Define the simulation duration and number of gait cycles used for metric computation.

---

## 2.2 Helper Function: Compute Performance for One Parameter Value

```python
# This block wraps a single simulation + metric computation into one function.
# It:
# 1) Builds the XML for a given parameter value,
# 2) Runs the simulation,
# 3) Computes average forward speed vx,
# 4) Returns vx and any other quantities of interest.

def evaluate_design_parameter(param_value,
                              param_name="gait_frequency_hz",
                              duration_s=10.0):
    """
    Evaluate performance for one parameter value.

    Parameters
    ----------
    param_value : float
        Value of the design parameter (e.g. gait frequency in Hz or joint stiffness).
    param_name : str
        Which parameter is being varied. Used to route param_value.
        Examples: "gait_frequency_hz", "joint_stiffness".
    duration_s : float
        Simulation duration (s).

    Returns
    -------
    metrics : dict
        Dictionary containing:
        - 'param_value': numeric parameter value
        - 'vx_avg': average forward speed
        - 'x_start', 'x_end': initial and final x position
        - 't_end': total simulated time
    """
    # Base parameters (TODO: replace with final values)
    joint_stiffness = 0.3
    joint_damping = 0.02
    mu_pad = 0.8
    mu_base = 0.2
    actuator_kp = 2.0
    actuator_kv = 0.01

    # Route the parameter value into the model or gait timing
    if param_name == "joint_stiffness":
        joint_stiffness = param_value
        gait_frequency_hz = 0.35  # fixed
    elif param_name == "gait_frequency_hz":
        gait_frequency_hz = param_value
    else:
        raise ValueError(f"Unknown param_name: {param_name}")

    # Build XML with updated parameters
    xml = build_model_xml(
        joint_stiffness=joint_stiffness,
        joint_damping=joint_damping,
        mu_pad=mu_pad,
        mu_base=mu_base,
        actuator_kp=actuator_kp,
        actuator_kv=actuator_kv
    )

    # Run simulation
    t, x = run_single_simulation(
        xml_string=xml,
        gait_frequency_hz=gait_frequency_hz,
        duration_s=duration_s,
        worm_amp_deg=45.0
    )

    # Compute performance metric (average speed)
    x_start = x[0]
    x_end = x[-1]
    t_end = t[-1]
    vx_avg = (x_end - x_start) / t_end

    metrics = {
        "param_value": param_value,
        "vx_avg": vx_avg,
        "x_start": x_start,
        "x_end": x_end,
        "t_end": t_end,
    }
    return metrics
```

---

## 2.3 Parameter Sweep Loop

```python
# This block performs a global sweep over a list of parameter values.
# It:
# 1) Defines a list of parameter values to test,
# 2) Calls evaluate_design_parameter() for each,
# 3) Stores the resulting metrics in arrays for plotting.

def sweep_parameter(param_values,
                    param_name="gait_frequency_hz",
                    duration_s=10.0):
    """
    Run a parameter sweep and collect metrics.

    Parameters
    ----------
    param_values : array-like
        List or array of parameter values to test.
    param_name : str
        Name of the parameter being swept.
    duration_s : float
        Simulation duration for each run.

    Returns
    -------
    param_values_out : np.ndarray
        Parameter values as tested.
    vx_avg_out : np.ndarray
        Corresponding average forward speeds.
    """
    vx_list = []
    param_list = []

    for val in param_values:
        print(f"Running simulation for {param_name} = {val}")
        metrics = evaluate_design_parameter(
            param_value=val,
            param_name=param_name,
            duration_s=duration_s
        )
        param_list.append(metrics["param_value"])
        vx_list.append(metrics["vx_avg"])

    return np.array(param_list), np.array(vx_list)

# Example sweep over gait frequency from 0.1 to 1.0 Hz
# TODO: choose ranges based on your actuator capabilities and desired speeds.
param_values = np.linspace(0.1, 1.0, 6)  # [0.1, 0.28, ..., 1.0] Hz
param_name = "gait_frequency_hz"

param_vals_sim, vx_sim = sweep_parameter(
    param_values=param_values,
    param_name=param_name,
    duration_s=10.0
)
```

---

## 2.4 Plotting Simulation Results

```python
# This block plots the performance metric (e.g. average speed) vs. parameter value.
# It:
# 1) Uses arrays from sweep_parameter(),
# 2) Produces a figure suitable for direct inclusion on the project website,
# 3) Identifies the "best" parameter value according to vx_avg.

plt.figure()
plt.plot(param_vals_sim, vx_sim, marker="o")
plt.xlabel("Gait frequency [Hz]")  # TODO: update label if parameter is different
plt.ylabel("Average forward speed v_x [m/s]")
plt.title("Simulation: Performance vs. gait frequency")
plt.grid(True)
plt.tight_layout()
plt.show()

# Identify the best parameter (max speed)
best_idx = np.argmax(vx_sim)
best_param = param_vals_sim[best_idx]
best_speed = vx_sim[best_idx]
print(f"Best {param_name} = {best_param:.3f} Hz, v_x = {best_speed:.4f} m/s")

# TODO: copy this information into your Results section once finalized.
```

---

![Line plot of average forward speed vs. gait frequency, highlighting the optimal value](path/to/sim_speed_vs_frequency.png)
*Figure 2. Example of performance metric (average forward speed) vs. gait frequency from simulation. TODO: replace with your actual plot.*

---

## 2.5 Optional: Fine-Grained Search / Optimization

If you find a promising region in the sweep, you can refine with a narrower search or an optimizer.

```python
# This block performs a second sweep around the best parameter to refine the optimum.
# It is a simple "zoom-in" approach instead of fully general optimization.

def refine_parameter_search(best_param,
                            window=0.2,
                            num_points=5,
                            param_name="gait_frequency_hz",
                            duration_s=10.0):
    """
    Refine the parameter search around the current best value.

    Parameters
    ----------
    best_param : float
        Previously identified best parameter value.
    window : float
        Half-width of the refinement interval around best_param.
    num_points : int
        Number of samples in the refinement interval.
    """
    lower = max(0.0, best_param - window)
    upper = max(lower + 1e-3, best_param + window)
    param_values_refined = np.linspace(lower, upper, num_points)

    return sweep_parameter(
        param_values=param_values_refined,
        param_name=param_name,
        duration_s=duration_s
    )

# Example refinement (TODO: only run after you have a first-pass optimum)
# param_vals_refined, vx_refined = refine_parameter_search(
#     best_param=best_param,
#     window=0.1,
#     num_points=7,
#     param_name=param_name,
#     duration_s=10.0
# )
```

**TODO:** Decide whether you need this refinement or if a coarse sweep is sufficient for Assignment 2.

---

# Results, Summary, and Discussion

This section is where you will present and interpret both simulation and experimental results, compare them, and discuss the sim-to-real gap. It mirrors the expectations of the Project Assignment 2 report and video. 

---

## 3.1 Simulation Results

Summarize the key trends from the parameter sweep:

* How does the performance metric e.g., $v_x$ change with the design parameter?
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
