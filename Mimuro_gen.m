% =========================================================================
% STANDALONE SCRIPT: MODIFIED MIMURO PLOT (NATURAL FREQUENCY ON +X)
% Uses Vehicle Parameters from MUS / ABS Simulation
% =========================================================================
clear; clc; close all;

% 1. Vehicle Parameters (From your ABS Script)
veh_params.m = 1600; 
veh_params.Iz = 2600; 
veh_params.lf = 1.3; 
veh_params.lr = 1.3; 
veh_params.ks_f = 345000; 
veh_params.ks_r = 410000;

% Benchmarking Conditions
V_kmh = 100;                % Standard benchmarking speed (km/h)
V_x = V_kmh / 3.6;          % Convert to m/s
f_bench = 1.0;              % Standard steering input frequency (Hz)
w_bench = 2 * pi * f_bench; % Rad/s

% 2. State-Space Matrices Construction (2-DOF Bicycle Model)
A11 = -(veh_params.ks_f + veh_params.ks_r) / (veh_params.m * V_x);
A12 = ((veh_params.ks_r * veh_params.lr - veh_params.ks_f * veh_params.lf) / (veh_params.m * V_x)) - V_x;
A21 = (veh_params.ks_r * veh_params.lr - veh_params.ks_f * veh_params.lf) / (veh_params.Iz * V_x);
A22 = -(veh_params.ks_f * veh_params.lf^2 + veh_params.ks_r * veh_params.lr^2) / (veh_params.Iz * V_x);

A = [A11, A12; A21, A22];
B = [veh_params.ks_f / veh_params.m; (veh_params.ks_f * veh_params.lf) / veh_params.Iz];

% Output Matrices
% C_r  -> Yaw Rate (State 2)
% C_ay -> Lateral Accel (v_dot + V_x*r)
C_r = [0, 1];
D_r = 0;
C_ay = [A11, (A12 + V_x)];
D_ay = B(1);

% Define Systems
sys_r = ss(A, B, C_r, D_r);
sys_ay = ss(A, B, C_ay, D_ay);

% 3. Extract Plot Points
% Steady-State Gains 
Gr_ss_rad = dcgain(sys_r);               
Gr_ss_deg = Gr_ss_rad; % deg/s per deg of steer (ratio is identical)

Gay_ss_ms2 = dcgain(sys_ay);             
Gay_ss_g = (Gay_ss_ms2 / 9.81) * (pi/180) * 100; % G per 100 deg of steer

% Natural Frequency (Replaces Yaw Rate Phase Lag on +X)
[Wn, ~] = damp(A);
f_n = Wn(1) / (2*pi); % Natural frequency in Hz

% Lateral Acceleration Phase Lag at 1 Hz (on -X)
[~, phase_ay] = bode(sys_ay, w_bench);
lag_ay = -squeeze(phase_ay); 

% 4. Normalization for Radar Chart Plotting
% Define maximum bounds to scale the axes visually
max_Gr = 0.5;    % Max expected Yaw Gain (deg/s/deg)
max_Gay = 1.5;   % Max expected Lat Accel Gain (G/100deg)
max_fn = 2.5;    % Max expected Natural Frequency (Hz)
max_lag = 60;    % Max expected Phase Lag (deg)

% Normalize values (0 to 1 scale)
norm_Gr = min(Gr_ss_deg / max_Gr, 1.0);
norm_Gay = min(Gay_ss_g / max_Gay, 1.0);
norm_fn = min(f_n / max_fn, 1.0);
norm_lag_ay = min(lag_ay / max_lag, 1.0);

% Define Polygon Coordinates (Top, Right, Bottom, Left)
X_poly = [0, norm_fn, 0, -norm_lag_ay, 0];
Y_poly = [norm_Gr, 0, -norm_Gay, 0, norm_Gr];

% 5. Render Modified Mimuro Plot
figure('Name', 'Modified Mimuro Handling Plot', 'Color', 'w', 'Position', [100, 100, 800, 800]);
hold on; axis equal; axis([-1.2 1.2 -1.2 1.2]); axis off;

% Draw Axis Crosshairs
plot([-1.1 1.1], [0 0], 'k-', 'LineWidth', 1.5); % Horizontal
plot([0 0], [-1.1 1.1], 'k-', 'LineWidth', 1.5); % Vertical

% Plot the Handling Diamond
fill(X_poly, Y_poly, [0.1 0.5 0.8], 'FaceAlpha', 0.3, 'EdgeColor', [0.05 0.3 0.6], 'LineWidth', 2.5);

% Plot the Vertices
plot(X_poly(1:4), Y_poly(1:4), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8, 'LineWidth', 1.5);

% Add Data Labels to Vertices
text(0.05, norm_Gr + 0.1, sprintf('%.3f deg/s/deg', Gr_ss_deg), 'FontWeight', 'bold');
text(norm_fn + 0.05, 0.05, sprintf('%.2f Hz', f_n), 'FontWeight', 'bold');
text(0.05, -norm_Gay - 0.1, sprintf('%.3f G/100deg', Gay_ss_g), 'FontWeight', 'bold');
text(-norm_lag_ay - 0.25, 0.05, sprintf('%.1f\\circ', lag_ay), 'FontWeight', 'bold');

% Add Axis Titles
text(0, 1.2, 'Yaw Rate Gain (Steady-State)', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 11);
text(0, -1.2, 'Lateral Accel Gain (Steady-State)', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 11);
text(1.2, 0, 'Natural Frequency', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 11);
text(-1.2, 0, sprintf('Lat Accel Phase Lag\n(@ %.1f Hz)', f_bench), 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 11);

% Main Title
title(sprintf('Modified Mimuro Plot | Speed: %d km/h', V_kmh), 'FontSize', 14, 'FontWeight', 'bold', 'Position', [0, 1.4, 0]);