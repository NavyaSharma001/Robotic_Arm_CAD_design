# Multi-DOF Robotic Arm: Geometric Modeling & Kinematic Framework

A 3-Degree-of-Freedom (3-DOF) anthropomorphic articulated robotic arm modeled using first-principles geometric design in SolidWorks[cite: 5]. This repository serves as the structural foundation for link coordinate tracking, forward kinematics modeling, and mass-properties extraction necessary for high-level dynamic simulations.

---

## ⚙️ Engineering & Design Features
* **Kinematic Configuration:** 3-DOF Anthropomorphic (Revolute-Revolute-Revolute) configuration optimizing a spherical workspace profile.
* **Mechanical Constraints:** Fully constrained mates establishing operational limits for physical joint rotation without inter-link interference.
* **Dynamic Simulation Readiness:** Designed with uniform material density properties to enable accurate center of mass (CoM) and inertia tensor extraction.

---

## 📐 Kinematic Architecture & DH Convention

To systematically track the position and orientation of each link relative to the base frame, the structural assembly is mapped using the standard **Denavit-Hartenberg (DH) convention**. 

Each joint is assigned a local coordinate frame according to strict rules:
1. The $z_i$-axis is aligned along the axis of rotation for joint $i+1$.
2. The $x_i$-axis is directed along the common normal from $z_{i-1}$ to $z_i$.

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

The resulting matrix $T_0^3$ outputs the exact 3D coordinates $(x, y, z)$ of the tool tip within the workspace as a function of the joint angles:

$$\begin{bmatrix} x \\ y \\ z \end{bmatrix} = \begin{bmatrix} T_0^3(1,4) \\ T_0^3(2,4) \\ T_0^3(3,4) \end{bmatrix}$$

---

## 📁 Repository Structure
* `/Assembly/Arm.SLDASM`: Main assembly file defining joint axes and spatial configurations.
* `/Parts/*.SLDPRT`: Individual component geometries (Base, Link 1, Link 2, Link 3) detailing strict mechanical dimensions[cite: 5].

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
