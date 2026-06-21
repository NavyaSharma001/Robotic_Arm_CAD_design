%% 3-DOF Articulated Robotic Arm Workspace Visualization Engine
% Author: Navya Sharma
% Generates a dense 3D point cloud mapping the boundary of the reachable workspace

clc; clear; close all;

%% 1. Physical Link Dimensions (Must match geometric parameters exactly)
d1 = 0.350;  % Base offset (meters)
a2 = 0.400;  % Upper arm length (meters)
a3 = 0.300;  % Forearm length (meters)

%% 2. Define Joint Operational Boundaries (Mechanical Constraints)
% Edit these ranges to reflect the physical limits of your SolidWorks mates
theta1_range = [-180, 180]; % Full rotation of the base
theta2_range = [-45, 90];   % Shoulder joint limit (preventing self-collision)
theta3_range = [-120, 120]; % Elbow joint limit

%% 3. Monte Carlo Workspace Sampling Setup
N = 30000; % Number of random configurations to generate (Scale up for denser plots)

% Pre-allocate coordinate vectors for high-speed memory execution
X = zeros(N, 1);
Y = zeros(N, 1);
Z = zeros(N, 1);

fprintf('====== 3-DOF Robot Workspace Boundary Engine ======\n');
fprintf('Sampling %d random configurations within mechanical boundaries...\n', N);

%% 4. Execution Loop (Vectorized Random State Space Generation)
% Generate uniformly distributed random joint states within defined limits
t1 = deg2rad(theta1_range(1) + (theta1_range(2) - theta1_range(1)) * rand(N, 1));
t2 = deg2rad(theta2_range(1) + (theta2_range(2) - theta2_range(1)) * rand(N, 1));
t3 = deg2rad(theta3_range(1) + (theta3_range(2) - theta3_range(1)) * rand(N, 1));

% Vectorized Forward Kinematics Evaluation
% Maps every joint space array row straight to Cartesian coordinates
X = cos(t1) .* (a2 .* cos(t2) + a3 .* cos(t2 + t3));
Y = sin(t1) .* (a2 .* cos(t2) + a3 .* cos(t2 + t3));
Z = d1 + a2 .* sin(t2) + a3 .* sin(t2 + t3);

fprintf('Point cloud generation complete. Compiling graphical workspace boundary...\n');

%% 5. 3D Graphical Visualization
figure('Name', '3-DOF Robotic Arm Reachable Workspace', 'NumberTitle', 'off');
set(gcf, 'Color', [1 1 1]); % Set background to clean white

% Plot the workspace coordinates as a point cloud
% Color-mapped by the Z-height to clearly display vertical reach envelopes
scatter3(X, Y, Z, 2, Z, 'filled');
colormap(jet); 
colorbar;
hold on;

% Mark the reference origin (Robot Base Position)
plot3(0, 0, 0, 'kx', 'MarkerSize', 12, 'LineWidth', 3);
text(0, 0, 0, '  Base Origin (0,0,0)', 'FontSize', 10, 'FontWeight', 'bold');

%% 6. Graph Formatting and Spatial Boundaries
title(sprintf('3-DOF Anthropomorphic Workspace Envelope \\newline (N = %d Samples)', N), 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X-Axis Reach (meters)', 'FontSize', 11);
ylabel('Y-Axis Reach (meters)', 'FontSize', 11);
zlabel('Z-Axis Height (meters)', 'FontSize', 11);

% Ensure axis scaling is uniform to prevent geometric distortion
axis equal;
grid on;
view(45, 20); % Set clean initial 3D viewing perspective perspective

% Calculate and display absolute operational reach boundaries
max_radial_reach = max(sqrt(X.^2 + Y.^2));
min_z = min(Z);
max_z = max(Z);

fprintf('\n--- Extracted Reach Specifications ---\n');
fprintf('  Maximum Horizontal Envelope Radius: %0.4f m\n', max_radial_reach);
fprintf('  Minimum Vertical Envelope Boundary: %0.4f m\n', min_z);
fprintf('  Maximum Vertical Envelope Boundary: %0.4f m\n\n', max_z);
