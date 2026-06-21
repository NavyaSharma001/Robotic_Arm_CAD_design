%% 3-DOF Articulated Robotic Arm Inverse Kinematics Engine
% Author: Navya Sharma
% Resolves target spatial coordinates (X, Y, Z) back into joint space variables

clc; clear; close all;

%% 1. Physical Link Dimensions (Must match FK exactly)
d1 = 0.350;  
a2 = 0.400;  
a3 = 0.300;  

%% 2. Target Input Coordinates (Taken from your validated FK output)
X_target = 0.4287;
Y_target = 0.4287;
Z_target = 0.4000;

fprintf('====== 3-DOF Inverse Kinematics Solver ======\n');
fprintf('Target Spatial Input: X = %0.4f m | Y = %0.4f m | Z = %0.4f m\n\n', X_target, Y_target, Z_target);

%% 3. Inverse Kinematics Analytical Algorithm
% Step 1: Base Rotation Angle
theta1 = atan2(Y_target, X_target);

% Step 2: Project target into the 2D manipulator plane
r = sqrt(X_target^2 + Y_target^2);
z_rel = Z_target - d1;

% Step 3: Solve for Joint 3 (Elbow Angle) using Law of Cosines
cos_theta3 = (r^2 + z_rel^2 - a2^2 - a3^2) / (2 * a2 * a3);

% Choosing the 'Elbow-Up' configuration (+) for calculation validation
sin_theta3 = -sqrt(1 - cos_theta3^2); 
theta3 = atan2(sin_theta3, cos_theta3);

% Step 4: Solve for Joint 2 (Shoulder Angle)
phi1 = atan2(z_rel, r);
phi2 = atan2(a3 * sin(theta3), a2 + a3 * cos(theta3));
theta2 = phi1 - phi2;

%% 4. Convert Radian Outputs back to Degrees for Verification
theta1_deg = rad2deg(theta1);
theta2_deg = rad2deg(theta2);
theta3_deg = rad2deg(theta3);

%% 5. Display Resolved Joint Space States
fprintf('--- Resolved System Configuration ---\n');
fprintf('  Calculated Joint 1: %0.1f deg (Expected: 45.0 deg)\n', theta1_deg);
fprintf('  Calculated Joint 2: %0.1f deg (Expected: 30.0 deg)\n', theta2_deg);
fprintf('  Calculated Joint 3: %0.1f deg (Expected: -60.0 deg)\n\n', theta3_deg);

if abs(theta1_deg - 45) < 0.5 && abs(theta2_deg - 30) < 0.5
    fprintf('>> Status: INVERSE KINEMATICS VALIDATED. System closed-loop is functional.\n');
else
    fprintf('>> Warning: Geometrical singularity or configuration divergence.\n');
end
