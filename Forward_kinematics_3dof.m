%% 3-DOF Articulated Robotic Arm Forward Kinematics Engine
% Author: Navya Sharma
% Configuration: Anthropomorphic (Revolute-Revolute-Revolute)
% Reference frames derived via Denavit-Hartenberg (DH) Convention

clc; clear; close all;

%% 1. Define Robot Physical Dimensions (Geometric Constraints)
% Replace these values with your exact dimensions from the SolidWorks parts
d1 = 0.35;  % Base height / Joint 1 offset (meters)
a2 = 0.40;  % Upper arm link length (meters)
a3 = 0.30;  % Forearm link length (meters)

fprintf('====== 3-DOF Anthropomorphic Robotic Arm Framework ======\n');
fprintf('System Geometry Initialized:\n');
fprintf('  Base Offset (d1): %0.3f m | Link 2 (a2): %0.3f m | Link 3 (a3): %0.3f m\n\n', d1, a2, a3);

%% 2. Define Input Joint States (Test Angles)
% Input angles are specified in degrees for readability, then converted to radians
theta1_deg = 45;   % Base rotation axis (Z1)
theta2_deg = 30;   % Shoulder rotation axis (Y2)
theta3_deg = -60;  % Elbow rotation axis (Y3)

% Convert to radians for trigonometric calculation
t1 = deg2rad(theta1_deg);
t2 = deg2rad(theta2_deg);
t3 = deg2rad(theta3_deg);

%% 3. Approach A: Analytical Closed-Form Position Evaluation
% Solved using trigonometric reduction from first principles
x_calc = cos(t1) * (a2*cos(t2) + a3*cos(t2 + t3));
y_calc = sin(t1) * (a2*cos(t2) + a3*cos(t2 + t3));
z_calc = d1 + a2*sin(t2) + a3*sin(t2 + t3);

%% 4. Approach B: Matrix Chain Multiplication Verification (DH Convention)
% Standard DH row mapping: [theta, alpha, a, d]
% Matrix structure transformation: Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)

A1 = dh_transform(t1, pi/2,   0, d1); % Base Frame to Link 1 Frame
A2 = dh_transform(t2,    0,  a2,  0); % Link 1 Frame to Link 2 Frame
A3 = dh_transform(t3,    0,  a3,  0); % Link 2 Frame to End-Effector Tip

% Compute full transformation matrix from Global Base (0) to Tool Tip (3)
T0_3 = A1 * A2 * A3;

% Extract position entries from the translation vector column (1 to 3, row 4)
x_mat = T0_3(1,4);
y_mat = T0_3(2,4);
z_mat = T0_3(3,4);

%% 5. Display Kinematic Validation Results
fprintf('Input Operational Coordinates:\n');
fprintf('  [Joint 1]: %0.1f deg | [Joint 2]: %0.1f deg | [Joint 3]: %0.1f deg\n\n', theta1_deg, theta2_deg, theta3_deg);

fprintf('--- Kinematic Verification ---\n');
fprintf('Analytical Position Solver Output:\n  X = %0.4f m | Y = %0.4f m | Z = %0.4f m\n', x_calc, y_calc, z_calc);
fprintf('Homogeneous DH Matrix Solver Output:\n  X = %0.4f m | Y = %0.4f m | Z = %0.4f m\n\n', x_mat, y_mat, z_mat);

% Check mathematical convergence between analytical and numerical calculations
if max(abs([x_calc-x_mat, y_calc-y_mat, z_calc-z_mat])) < 1e-6
    fprintf('>> Status: MATCH VALIDATED. System configuration converges perfectly.\n');
else
    fprintf('>> Warning: Numerical divergence detected. Verify DH table assignment rules.\n');
end

%% 6. Local Helper Function: Homogeneous DH Transformation Matrix Generator
function A = dh_transform(theta, alpha, a, d)
    % Generates standard 4x4 matrix for joint transformations
    A = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
          sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                   0,             sin(alpha),             cos(alpha),           d;
                   0,                      0,                      0,           1 ];
end
