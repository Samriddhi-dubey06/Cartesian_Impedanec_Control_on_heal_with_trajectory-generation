# 🤖 Cartesian Impedance Control for Robot Manipulation

This project implements a **Cartesian Impedance Controller** for a robotic manipulator collaboratively lifting and placing a box with a human partner. The robot operates in task space (Cartesian space) and exhibits compliant, human-friendly behavior.

---

## 🧠 Core Concept

The end-effector is modeled as a **mass-spring-damper system** in 3D space. This allows the robot to follow smooth position and orientation trajectories while **adapting compliantly** to external forces — ideal for tasks involving human-robot interaction (HRI).

---

## 📦 Application Scenario

A **human and a robot** collaboratively lift a box and move it across three predefined Cartesian poses:

1. Approach and align with the box  
2. Lift the box upward  
3. Move the box to the desired drop location  

The robot follows smooth, dynamically generated trajectories and holds the box steady using impedance behavior.


---

### 🧮 Kinematics Handling (via KDL)

- Computes **forward kinematics** for pose tracking  
- Computes **Jacobian** for mapping Cartesian forces to joint torques  
- Updates pose and velocity of the robot's **end-effector** in real-time

---

### 🚀 Trajectory Generation

- **Position**: Uses **quintic polynomial interpolation** for smooth acceleration/deceleration  
- **Orientation**: Uses **SLERP** (Spherical Linear Interpolation) between quaternions

\[
p(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5
\]

Ensures:
- Zero start/end velocity and acceleration  
- Smooth and jerk-free transitions  
- Interpolation over **15 seconds per pose**

---

## 🧭 Execution Flow

1. **Initialize Pose** – Approach the box with precise alignment  
2. **Lift Pose** – Raise the box upwards with compliance  
3. **Place Pose** – Move the box to the drop-off location  
4. **Hold** – Maintain final pose using impedance-based force control

### 🔁 Loop Details

- Runs at **500 Hz** for high responsiveness  
- Continuously computes Cartesian pose/velocity errors  
- Applies Cartesian impedance torques via Jacobian transpose  
- Detects when target is reached (within a tolerance) before moving to the next pose

---

## 🤝 Human-Robot Collaboration

This control strategy allows the robot to:

- Work safely and compliantly with a human lifting the other side of the box  
- Automatically adapt to small external disturbances or human corrections  
- Avoid excessive joint forces through natural impedance-based compliance  

Perfect for:
- Co-manipulation  
- Physical Human-Robot Interaction (pHRI)  
- Assistive lifting tasks

---

