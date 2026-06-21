# Multi-DOF Robotic Arm: Geometric Modeling & Kinematic Framework

A 3-Degree-of-Freedom (3-DOF) anthropomorphic articulated robotic arm modeled using first-principles geometric design in SolidWorks. This repository serves as the structural foundation for link coordinate tracking, forward kinematics modeling, and mass-properties extraction necessary for high-level dynamic simulations.

---

## ⚙️ Engineering & Design Features
* **Kinematic Configuration:** 3-DOF Anthropomorphic (Revolute-Revolute-Revolute) design producing an approximately spherical reachable workspace.
* **Mechanical Constraints:** Fully constrained mates establishing operational limits for physical joint rotation without inter-link interference.
* **Dynamic Simulation Readiness:** Links are modeled with consistent density assumptions to facilitate preliminary center of mass (CoM) and inertia matrix estimation.

---

## 📐 Kinematic Architecture & DH Convention

To systematically track the position and orientation of each link relative to the base frame, the structural assembly is mapped using the standard **Denavit-Hartenberg (DH) convention**. 

Each DH frame is assigned such that the z-axis coincides with the corresponding joint axis of rotation.

### The Denavit-Hartenberg Parameter Table

| Link ($i$) | Joint Angle ($\theta_i$) | Link Twist ($\alpha_i$) | Link Length ($a_i$) | Joint Offset ($d_i$) |
| :--- | :--- | :--- | :--- | :--- |
| **Base (1)** | $\theta_1$ | $90^\circ$ | $a_1$ | $d_1$ |
| **Upper Arm (2)**| $\theta_2$ | $0^\circ$ | $a_2$ | $0$ |
| **Forearm (3)** | $\theta_3$ | $0^\circ$ | $a_3$ | $0$ |

### Mathematical Framework: Homogeneous Transformations

For each link, the individual joint transition is modeled by a $4 \times 4$ homogeneous transformation matrix $A_i$, computed via successive rotations and translations:

$$A_i = Rot_{z,\theta_i} \cdot Trans_{z,d_i} \cdot Trans_{x,a_i} \cdot Rot_{x,\alpha_i}$$

Evaluating this matrix multiplication yields the standard DH transformation matrix structure used in our analytical solver:

$$A_i = \begin{bmatrix} 
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\ 
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\ 
0 & \sin\alpha_i & \cos\alpha_i & d_i \\ 
0 & 0 & 0 & 1 
\end{bmatrix}$$

### Complete Forward Kinematics Chain
To find the absolute position and orientation of the end-effector (hand) relative to the global base frame, the individual transformation matrices are sequentially multiplied:

$$T_0^3 = A_1(\theta_1) \cdot A_2(\theta_2) \cdot A_3(\theta_3)$$

By expanding the translation column of the final composite transformation matrix $T_0^3$, the closed-form solutions for the end-effector coordinate position $(x, y, z)$ within the workspace are explicitly derived as:

$$x = \cos\theta_1 \left( a_2\cos\theta_2 + a_3\cos(\theta_2+\theta_3) \right)$$

$$y = \sin\theta_1 \left( a_2\cos\theta_2 + a_3\cos(\theta_2+\theta_3) \right)$$

$$z = d_1 + a_2\sin\theta_2 + a_3\sin(\theta_2+\theta_3)$$

---

## 📁 Repository Structure
* `/Assembly/Arm.SLDASM`: Main assembly file defining joint axes and spatial configurations.
* `/Parts/*.SLDPRT`: Individual component geometries (Base, Link 1, Link 2, Link 3) detailing strict mechanical dimensions.

---

## 🚀 Future Roadmap & Control Pipeline
1. **[Phase 1 - Geometric Design]:** Complete assembly modeling and clearance verification (Completed).
2. **[Phase 2 - Mass Properties]:** Assign realistic physical material matrices (Aluminum 6061) to extract exact 3x3 inertia tensors.
3. **[Phase 3 - Kinematics Engine]:** Deploy MATLAB scripts to solve analytical Inverse Kinematics for trajectory tracking.

---

## 🛠️ How to View and Use
1. Clone or download this repository to your local machine.
2. Ensure that the `/Parts` and `/Assembly` folders remain in the same relative directory structure to prevent broken file references within SolidWorks.
3. Open the primary assembly file `Arm.SLDASM` in **SolidWorks**.
