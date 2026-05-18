% =========================================================================
% MASTER SIMULATION & TELEMETRY SCRIPT: STEP STEER (STR) A/B TESTING
% Models: STR_2a (Dual Motor / TV) vs STR_2b (Single Motor / Open Diff)
% =========================================================================
clear; clc; close all;

%% 1. USER INPUT & INITIALIZATION
disp('====================================================');
disp('   ISO 7401 STEP STEER (STR) - A/B TEST CONTROLLER  ');
disp('====================================================');
% Prompt user for speed
test_speed_kmh = 100; %input('Enter the target steady-state speed for the Step Steer test (km/h): ');
test_speed_ms = test_speed_kmh / 3.6;

% Assign to base workspace so Simulink and init scripts can access it
assignin('base', 'V_test', test_speed_ms);

% Call your initialization script
disp('-> Loading Vehicle Parameters & Ramp-Step Profile...');
% Ensure this script generates steer_profile and vx_target_array!
% init_step_steer; 

%% 2. EXECUTE SIMULATIONS (With Workspace Wipe Protection)
disp(['-> Running Dual Motor Simulation (STR_2a) at ', num2str(test_speed_kmh), ' km/h...']);
out_a = sim('STR_2a', 'ReturnWorkspaceOutputs', 'on');
% Instantly save it to a temporary file before STR_2b or any init scripts can wipe it
save('temp_out_a.mat', 'out_a'); 
disp('   [STR_2a Complete & Saved]');

disp(['-> Running Single Motor Simulation (STR_2b) at ', num2str(test_speed_kmh), ' km/h...']);
out_b = sim('STR_2b', 'ReturnWorkspaceOutputs', 'on');
disp('   [STR_2b Complete]');

% Reload out_a back into the workspace for plotting
load('temp_out_a.mat');
% Clean up the temporary file
delete('temp_out_a.mat'); 

%% 3. DATA EXTRACTION: CHASSIS DYNAMICS
disp('-> Extracting Chassis Telemetry Data...');
% Time vectors
t_a = out_a.tout;
t_b = out_b.tout;

% ----------------- VEHICLE A (DUAL MOTOR) -----------------
X_a = squeeze(out_a.X_curr_a.Data);
Y_a = squeeze(out_a.Y_curr_a.Data);
vx_a = squeeze(out_a.vx_curr_a.Data);
vy_a = squeeze(out_a.vy_curr_a.Data);
r_a_rad = squeeze(out_a.r_curr_a.Data);
r_a_deg = r_a_rad * (180/pi);
r_ref_deg = squeeze(out_a.r_ref_a.Data) * (180/pi); % Baseline target
delta_a_deg = squeeze(out_a.delta_a.Data) * (180/pi);
Trr_m_a = squeeze(out_a.Trr_m_a.Data);
Trl_m_a = squeeze(out_a.Trl_m_a.Data);
beta_a_deg = atan2(vy_a, vx_a) * (180/pi);

% ----------------- VEHICLE B (SINGLE MOTOR) ---------------
X_b = squeeze(out_b.X_curr_b.Data);
Y_b = squeeze(out_b.Y_curr_b.Data);
vx_b = squeeze(out_b.vx_curr_b.Data);
vy_b = squeeze(out_b.vy_curr_b.Data);
r_b_rad = squeeze(out_b.r_curr_b.Data);
r_b_deg = r_b_rad * (180/pi);
delta_b_deg = squeeze(out_b.delta_b.Data) * (180/pi);
Trr_m_b = squeeze(out_b.Trr_m_b.Data);
Trl_m_b = squeeze(out_b.Trl_m_b.Data);
T_total_b = Trr_m_b + Trl_m_b;

% ---> ADD THESE TWO LINES <---
Trr_fric_b = squeeze(out_b.Trr_fric_b.Data);
Trl_fric_b = squeeze(out_b.Trl_fric_b.Data);

beta_b_deg = atan2(vy_b, vx_b) * (180/pi);

%% 4. DATA EXTRACTION: TIRE KINEMATICS & DYNAMICS
disp('-> Extracting Tire Dynamics & Calculating Kinematics...');
% Physical Constants 
Rw = 0.34;      % Tire Radius (m)
tw = 1.65;      % Track Width (m)
mu_max = 1.2;   % Peak Friction Coefficient (Adjust if necessary)
lf = 1.3;       % Distance from CG to front axle (m)
lr = 1.3;       % Distance from CG to rear axle (m)

% --- Vehicle A (Dual Motor TV) Corner Data ---
Fx_rr_a = squeeze(out_a.Fx_rr_a.Data); Fx_rl_a = squeeze(out_a.Fx_rl_a.Data);
Fx_fr_a = squeeze(out_a.Fx_fr_a.Data); Fx_fl_a = squeeze(out_a.Fx_fl_a.Data);
Fy_rr_a = squeeze(out_a.Fy_rr_a.Data); Fy_rl_a = squeeze(out_a.Fy_rl_a.Data);
Fy_fr_a = squeeze(out_a.Fy_fr_a.Data); Fy_fl_a = squeeze(out_a.Fy_fl_a.Data);
Fz_rr_a = squeeze(out_a.Fz_rr_a.Data); Fz_rl_a = squeeze(out_a.Fz_rl_a.Data);
Fz_fr_a = squeeze(out_a.Fz_fr_a.Data); Fz_fl_a = squeeze(out_a.Fz_fl_a.Data);
w_rr_a = squeeze(out_a.w_rr_a.Data); w_rl_a = squeeze(out_a.w_rl_a.Data);
w_fr_a = squeeze(out_a.w_fr_a.Data); w_fl_a = squeeze(out_a.w_fl_a.Data);

% --- Vehicle B (Single Motor) Corner Data ---
Fx_rr_b = squeeze(out_b.Fx_rr_b.Data); Fx_rl_b = squeeze(out_b.Fx_rl_b.Data);
Fx_fr_b = squeeze(out_b.Fx_fr_b.Data); Fx_fl_b = squeeze(out_b.Fx_fl_b.Data);
Fy_rr_b = squeeze(out_b.Fy_rr_b.Data); Fy_rl_b = squeeze(out_b.Fy_rl_b.Data);
Fy_fr_b = squeeze(out_b.Fy_fr_b.Data); Fy_fl_b = squeeze(out_b.Fy_fl_b.Data);
Fz_rr_b = squeeze(out_b.Fz_rr_b.Data); Fz_rl_b = squeeze(out_b.Fz_rl_b.Data);
Fz_fr_b = squeeze(out_b.Fz_fr_b.Data); Fz_fl_b = squeeze(out_b.Fz_fl_b.Data);
w_rr_b = squeeze(out_b.w_rr_b.Data); w_rl_b = squeeze(out_b.w_rl_b.Data);
w_fr_b = squeeze(out_b.w_fr_b.Data); w_fl_b = squeeze(out_b.w_fl_b.Data);

% =========================================================================
% --- CALCULATIONS: Friction Limits & Resultant Forces ---
% =========================================================================
% Dynamic Grip Limits (mu * Fz)
Fz_lim_rr_a = mu_max * Fz_rr_a; Fz_lim_rl_a = mu_max * Fz_rl_a;
Fz_lim_fr_a = mu_max * Fz_fr_a; Fz_lim_fl_a = mu_max * Fz_fl_a;
Fz_lim_rr_b = mu_max * Fz_rr_b; Fz_lim_rl_b = mu_max * Fz_rl_b;
Fz_lim_fr_b = mu_max * Fz_fr_b; Fz_lim_fl_b = mu_max * Fz_fl_b;

% Resultant Total Tire Force (Fres = sqrt(Fx^2 + Fy^2))
F_res_fl_a = sqrt(Fx_fl_a.^2 + Fy_fl_a.^2);
F_res_fr_a = sqrt(Fx_fr_a.^2 + Fy_fr_a.^2);
F_res_rl_a = sqrt(Fx_rl_a.^2 + Fy_rl_a.^2);
F_res_rr_a = sqrt(Fx_rr_a.^2 + Fy_rr_a.^2);

F_res_fl_b = sqrt(Fx_fl_b.^2 + Fy_fl_b.^2);
F_res_fr_b = sqrt(Fx_fr_b.^2 + Fy_fr_b.^2);
F_res_rl_b = sqrt(Fx_rl_b.^2 + Fy_rl_b.^2);
F_res_rr_b = sqrt(Fx_rr_b.^2 + Fy_rr_b.^2);

% =========================================================================
% --- CALCULATIONS: Longitudinal Slip (Kappa) ---
% =========================================================================
% Hub X-Velocities A
v_hub_rr_a = max(vx_a + (tw/2)*r_a_rad, 1.0); v_hub_rl_a = max(vx_a - (tw/2)*r_a_rad, 1.0);
v_hub_fr_a = max(vx_a + (tw/2)*r_a_rad, 1.0); v_hub_fl_a = max(vx_a - (tw/2)*r_a_rad, 1.0);

% Slip A
kappa_rr_a = (w_rr_a * Rw - v_hub_rr_a) ./ v_hub_rr_a; kappa_rl_a = (w_rl_a * Rw - v_hub_rl_a) ./ v_hub_rl_a;
kappa_fr_a = (w_fr_a * Rw - v_hub_fr_a) ./ v_hub_fr_a; kappa_fl_a = (w_fl_a * Rw - v_hub_fl_a) ./ v_hub_fl_a;

% Hub X-Velocities B
v_hub_rr_b = max(vx_b + (tw/2)*r_b_rad, 1.0); v_hub_rl_b = max(vx_b - (tw/2)*r_b_rad, 1.0);
v_hub_fr_b = max(vx_b + (tw/2)*r_b_rad, 1.0); v_hub_fl_b = max(vx_b - (tw/2)*r_b_rad, 1.0);

% Slip B
kappa_rr_b = (w_rr_b * Rw - v_hub_rr_b) ./ v_hub_rr_b; kappa_rl_b = (w_rl_b * Rw - v_hub_rl_b) ./ v_hub_rl_b;
kappa_fr_b = (w_fr_b * Rw - v_hub_fr_b) ./ v_hub_fr_b; kappa_fl_b = (w_fl_b * Rw - v_hub_fl_b) ./ v_hub_fl_b;

% =========================================================================
% --- CALCULATIONS: Lateral Slip / Slip Angle (Alpha) ---
% =========================================================================
% Hub Y-Velocities A
v_hub_y_f_a = vy_a + (lf * r_a_rad);
v_hub_y_r_a = vy_a - (lr * r_a_rad);

% Slip Angles A (Radians to Degrees)
alpha_fl_a = ((delta_a_deg * pi/180) - atan2(v_hub_y_f_a, v_hub_fl_a)) * (180/pi);
alpha_fr_a = ((delta_a_deg * pi/180) - atan2(v_hub_y_f_a, v_hub_fr_a)) * (180/pi);
alpha_rl_a = (0 - atan2(v_hub_y_r_a, v_hub_rl_a)) * (180/pi);
alpha_rr_a = (0 - atan2(v_hub_y_r_a, v_hub_rr_a)) * (180/pi);

% Hub Y-Velocities B
v_hub_y_f_b = vy_b + (lf * r_b_rad);
v_hub_y_r_b = vy_b - (lr * r_b_rad);

% Slip Angles B (Radians to Degrees)
alpha_fl_b = ((delta_b_deg * pi/180) - atan2(v_hub_y_f_b, v_hub_fl_b)) * (180/pi);
alpha_fr_b = ((delta_b_deg * pi/180) - atan2(v_hub_y_f_b, v_hub_fr_b)) * (180/pi);
alpha_rl_b = (0 - atan2(v_hub_y_r_b, v_hub_rl_b)) * (180/pi);
alpha_rr_b = (0 - atan2(v_hub_y_r_b, v_hub_rr_b)) * (180/pi);

% =========================================================================
% --- CALCULATIONS: Tire G-Forces (Utilized Friction) ---
% =========================================================================
min_Fz = 1.0; % Prevent divide-by-zero during wheel lift

% Vehicle A G-Forces
G_lat_fl_a = Fy_fl_a ./ max(Fz_fl_a, min_Fz); G_long_fl_a = Fx_fl_a ./ max(Fz_fl_a, min_Fz);
G_lat_fr_a = Fy_fr_a ./ max(Fz_fr_a, min_Fz); G_long_fr_a = Fx_fr_a ./ max(Fz_fr_a, min_Fz);
G_lat_rl_a = Fy_rl_a ./ max(Fz_rl_a, min_Fz); G_long_rl_a = Fx_rl_a ./ max(Fz_rl_a, min_Fz);
G_lat_rr_a = Fy_rr_a ./ max(Fz_rr_a, min_Fz); G_long_rr_a = Fx_rr_a ./ max(Fz_rr_a, min_Fz);

% Vehicle B G-Forces
G_lat_fl_b = Fy_fl_b ./ max(Fz_fl_b, min_Fz); G_long_fl_b = Fx_fl_b ./ max(Fz_fl_b, min_Fz);
G_lat_fr_b = Fy_fr_b ./ max(Fz_fr_b, min_Fz); G_long_fr_b = Fx_fr_b ./ max(Fz_fr_b, min_Fz);
G_lat_rl_b = Fy_rl_b ./ max(Fz_rl_b, min_Fz); G_long_rl_b = Fx_rl_b ./ max(Fz_rl_b, min_Fz);
G_lat_rr_b = Fy_rr_b ./ max(Fz_rr_b, min_Fz); G_long_rr_b = Fx_rr_b ./ max(Fz_rr_b, min_Fz);

%% 5. PLOTTING AND VISUALIZATION
disp('-> Generating Telemetry Dashboards (Figures 1 to 10)...');

% =========================================================================
% FIG 1: Chassis Dynamics & Tracking
% =========================================================================
figure('Name', 'Fig 1: Chassis Dynamics & Tracking', 'Color', 'w', 'Position', [50, 50, 1000, 800]);
subplot(3,1,1);
plot(t_a, r_ref_deg, 'k--', 'LineWidth', 1.5); hold on; grid on;
plot(t_a, r_a_deg, 'b-', 'LineWidth', 1.5); plot(t_b, r_b_deg, 'r-', 'LineWidth', 1.5);
title('Yaw Rate Tracking: TV vs Open Diff'); xlabel('Time (s)'); ylabel('Yaw Rate (deg/s)');
legend('Target Yaw Rate (r_{ref})', 'Dual Motor TV (r_a)', 'Single Motor (r_b)', 'Location', 'best');

subplot(3,1,2);
plot(t_a, beta_a_deg, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, beta_b_deg, 'r-', 'LineWidth', 1.5);
title('Vehicle Sideslip Angle (\beta)'); xlabel('Time (s)'); ylabel('Sideslip (deg)');
legend('Dual Motor TV (\beta_a)', 'Single Motor (\beta_b)', 'Location', 'best');

subplot(3,1,3);
% Since Step Steer is open loop, we just plot the origin line and the paths turning out
plot(X_a, zeros(size(X_a)), 'k--', 'LineWidth', 1.0); hold on; grid on;
plot(X_a, Y_a, 'b-', 'LineWidth', 1.5); plot(X_b, Y_b, 'r-', 'LineWidth', 1.5);
title('XY Path Tracing (Step Steer Trajectory)'); xlabel('Global X (m)'); ylabel('Global Y (m)');
legend('Initial Heading (Y=0)', 'Dual Motor TV Path', 'Single Motor Path', 'Location', 'best'); axis equal; 

% =========================================================================
% FIG 2: Motor Torque Allocation & Brake Vectoring
% =========================================================================
figure('Name', 'Fig 2: Motor Torque Allocation', 'Color', 'w', 'Position', [100, 100, 800, 600]);

subplot(2,1,1);
plot(t_a, Trr_m_a, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_a, Trl_m_a, 'g-', 'LineWidth', 1.5);
title('Vehicle A (Dual Motor) - Independent Torque Allocation'); xlabel('Time (s)'); ylabel('Motor Torque (Nm)');
legend('Rear Right Motor (T_{rr})', 'Rear Left Motor (T_{rl})', 'Location', 'best');

subplot(2,1,2);
% --- Left Y-Axis: Motor Torque ---
yyaxis left;
plot(t_b, T_total_b, 'r-', 'LineWidth', 1.5); hold on; grid on;
ylabel('Combined Motor Torque (Nm)');

% --- Right Y-Axis: Friction Brake Torque ---
yyaxis right;
plot(t_b, Trr_fric_b, 'b--', 'LineWidth', 1.2); hold on;
plot(t_b, Trl_fric_b, 'g--', 'LineWidth', 1.2);
ylabel('Friction Brake Torque (Nm)');

title('Vehicle B (Single Motor) - Total Axle Torque & Brake Vectoring'); 
xlabel('Time (s)'); 
legend('Total Motor Torque', 'Rear Right Brake (T_{rr\_fric})', 'Rear Left Brake (T_{rl\_fric})', 'Location', 'best');

% =========================================================================
% FIG 3: Steering Inputs & Lateral Velocity
% =========================================================================
figure('Name', 'Fig 3: Steering Inputs & Lateral Velocity', 'Color', 'w', 'Position', [150, 150, 800, 600]);
subplot(2,1,1);
plot(t_a, delta_a_deg, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, delta_b_deg, 'r-', 'LineWidth', 1.5);
title('Actual Steering Angle at the Wheels (\delta)'); xlabel('Time (s)'); ylabel('Steering Angle (deg)');
legend('Dual Motor TV (\delta_a)', 'Single Motor (\delta_b)', 'Location', 'best');

subplot(2,1,2);
plot(t_a, vy_a, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, vy_b, 'r-', 'LineWidth', 1.5);
title('Lateral Velocity (V_y)'); xlabel('Time (s)'); ylabel('Lateral Velocity (m/s)');
legend('Dual Motor TV (V_{ya})', 'Single Motor (V_{yb})', 'Location', 'best');

% =========================================================================
% FIG 4: Longitudinal Tire Forces (Fx)
% =========================================================================
figure('Name', 'Fig 4: Longitudinal Tire Forces (Fx)', 'Color', 'w', 'Position', [200, 200, 1000, 700]);
subplot(2,2,1); plot(t_a, Fx_fl_a, 'b-'); hold on; grid on; plot(t_b, Fx_fl_b, 'r-'); title('Front Left (FL)'); ylabel('Fx (N)');
subplot(2,2,2); plot(t_a, Fx_fr_a, 'b-'); hold on; grid on; plot(t_b, Fx_fr_b, 'r-'); title('Front Right (FR)'); 
subplot(2,2,3); plot(t_a, Fx_rl_a, 'b-'); hold on; grid on; plot(t_b, Fx_rl_b, 'r-'); title('Rear Left (RL)'); xlabel('Time (s)'); ylabel('Fx (N)');
subplot(2,2,4); plot(t_a, Fx_rr_a, 'b-'); hold on; grid on; plot(t_b, Fx_rr_b, 'r-'); title('Rear Right (RR)'); xlabel('Time (s)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best');

% =========================================================================
% FIG 5: Lateral Tire Forces (Fy)
% =========================================================================
figure('Name', 'Fig 5: Lateral Tire Forces (Fy)', 'Color', 'w', 'Position', [250, 250, 1000, 700]);
subplot(2,2,1); plot(t_a, Fy_fl_a, 'b-'); hold on; grid on; plot(t_b, Fy_fl_b, 'r-'); title('Front Left (FL)'); ylabel('Fy (N)');
subplot(2,2,2); plot(t_a, Fy_fr_a, 'b-'); hold on; grid on; plot(t_b, Fy_fr_b, 'r-'); title('Front Right (FR)'); 
subplot(2,2,3); plot(t_a, Fy_rl_a, 'b-'); hold on; grid on; plot(t_b, Fy_rl_b, 'r-'); title('Rear Left (RL)'); xlabel('Time (s)'); ylabel('Fy (N)');
subplot(2,2,4); plot(t_a, Fy_rr_a, 'b-'); hold on; grid on; plot(t_b, Fy_rr_b, 'r-'); title('Rear Right (RR)'); xlabel('Time (s)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best');

% =========================================================================
% FIG 6: Time-Domain Grip Utilization (F_res vs \mu * Fz)
% =========================================================================
figure('Name', 'Fig 6: Grip Utilization (F_res vs \mu * Fz)', 'Color', 'w', 'Position', [300, 300, 1000, 700]);
subplot(2,2,1); 
plot(t_a, Fz_lim_fl_a, 'b--', 'LineWidth', 1); hold on; grid on;
plot(t_a, F_res_fl_a, 'b-', 'LineWidth', 1.5);
plot(t_b, Fz_lim_fl_b, 'r--', 'LineWidth', 1);
plot(t_b, F_res_fl_b, 'r-', 'LineWidth', 1.5);
title('Front Left (FL) Grip Utilization'); ylabel('Force (N)');

subplot(2,2,2); 
plot(t_a, Fz_lim_fr_a, 'b--', 'LineWidth', 1); hold on; grid on;
plot(t_a, F_res_fr_a, 'b-', 'LineWidth', 1.5);
plot(t_b, Fz_lim_fr_b, 'r--', 'LineWidth', 1);
plot(t_b, F_res_fr_b, 'r-', 'LineWidth', 1.5);
title('Front Right (FR) Grip Utilization'); 

subplot(2,2,3); 
plot(t_a, Fz_lim_rl_a, 'b--', 'LineWidth', 1); hold on; grid on;
plot(t_a, F_res_rl_a, 'b-', 'LineWidth', 1.5);
plot(t_b, Fz_lim_rl_b, 'r--', 'LineWidth', 1);
plot(t_b, F_res_rl_b, 'r-', 'LineWidth', 1.5);
title('Rear Left (RL) Grip Utilization'); xlabel('Time (s)'); ylabel('Force (N)');

subplot(2,2,4); 
plot(t_a, Fz_lim_rr_a, 'b--', 'LineWidth', 1); hold on; grid on;
plot(t_a, F_res_rr_a, 'b-', 'LineWidth', 1.5);
plot(t_b, Fz_lim_rr_b, 'r--', 'LineWidth', 1);
plot(t_b, F_res_rr_b, 'r-', 'LineWidth', 1.5);
title('Rear Right (RR) Grip Utilization'); xlabel('Time (s)');
legend('TV Limit (\mu*Fz)', 'TV Actual (F_{res})', 'Single Motor Limit (\mu*Fz)', 'Single Motor Actual (F_{res})', 'Location', 'best');

% =========================================================================
% FIG 7: Tire Longitudinal Slip (\kappa)
% =========================================================================
figure('Name', 'Fig 7: Tire Longitudinal Slip (\kappa)', 'Color', 'w', 'Position', [350, 350, 1000, 700]);
subplot(2,2,1); plot(t_a, kappa_fl_a, 'b-'); hold on; grid on; plot(t_b, kappa_fl_b, 'r-'); title('Front Left (FL) Slip'); ylabel('Slip (\kappa)');
subplot(2,2,2); plot(t_a, kappa_fr_a, 'b-'); hold on; grid on; plot(t_b, kappa_fr_b, 'r-'); title('Front Right (FR) Slip'); 
subplot(2,2,3); plot(t_a, kappa_rl_a, 'b-'); hold on; grid on; plot(t_b, kappa_rl_b, 'r-'); title('Rear Left (RL) Slip'); xlabel('Time (s)'); ylabel('Slip (\kappa)');
subplot(2,2,4); plot(t_a, kappa_rr_a, 'b-'); hold on; grid on; plot(t_b, kappa_rr_b, 'r-'); title('Rear Right (RR) Slip'); xlabel('Time (s)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best');

% =========================================================================
% FIG 8: Tire Lateral Slip Angles (\alpha)
% =========================================================================
figure('Name', 'Fig 8: Tire Lateral Slip Angles (\alpha)', 'Color', 'w', 'Position', [400, 400, 1000, 700]);
subplot(2,2,1); plot(t_a, alpha_fl_a, 'b-'); hold on; grid on; plot(t_b, alpha_fl_b, 'r-'); title('Front Left (FL) \alpha'); ylabel('Slip Angle (deg)');
subplot(2,2,2); plot(t_a, alpha_fr_a, 'b-'); hold on; grid on; plot(t_b, alpha_fr_b, 'r-'); title('Front Right (FR) \alpha'); 
subplot(2,2,3); plot(t_a, alpha_rl_a, 'b-'); hold on; grid on; plot(t_b, alpha_rl_b, 'r-'); title('Rear Left (RL) \alpha'); xlabel('Time (s)'); ylabel('Slip Angle (deg)');
subplot(2,2,4); plot(t_a, alpha_rr_a, 'b-'); hold on; grid on; plot(t_b, alpha_rr_b, 'r-'); title('Rear Right (RR) \alpha'); xlabel('Time (s)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best');

% =========================================================================
% FIG 9: The Friction Circle (G-G Diagram)
% =========================================================================
figure('Name', 'Fig 9: Tire Friction Circles (G-G Diagram)', 'Color', 'w', 'Position', [450, 450, 1000, 800]);
% Generate data for the outer limit circle (mu_max)
theta = linspace(0, 2*pi, 100);
circle_x = mu_max * cos(theta);
circle_y = mu_max * sin(theta);

% Helper logic to plot crosshairs and bounds cleanly
for i = 1:4
    subplot(2,2,i);
    plot(circle_x, circle_y, 'k-', 'LineWidth', 2); hold on; grid on; % Limit Circle
    
    % Draw crosshairs
    plot([-2, 2], [0, 0], 'k:', 'LineWidth', 1);
    plot([0, 0], [-2, 2], 'k:', 'LineWidth', 1);
    
    xlabel('Lateral Gs (G_y)');
    ylabel('Longitudinal Gs (G_x)');
    axis equal; xlim([-1.8 1.8]); ylim([-1.8 1.8]);
end

% Plot specific data into each subplot
subplot(2,2,1); title('Front Left (FL) G-G');
plot(G_lat_fl_b, G_long_fl_b, 'r.', 'MarkerSize', 2); plot(G_lat_fl_a, G_long_fl_a, 'b.', 'MarkerSize', 2);
subplot(2,2,2); title('Front Right (FR) G-G');
plot(G_lat_fr_b, G_long_fr_b, 'r.', 'MarkerSize', 2); plot(G_lat_fr_a, G_long_fr_a, 'b.', 'MarkerSize', 2);
legend('Grip Limit (\mu_{max})', '', '', 'Single Motor', 'Dual Motor TV', 'Location', 'bestoutside');

subplot(2,2,3); title('Rear Left (RL) G-G');
plot(G_lat_rl_b, G_long_rl_b, 'r.', 'MarkerSize', 2); plot(G_lat_rl_a, G_long_rl_a, 'b.', 'MarkerSize', 2);
subplot(2,2,4); title('Rear Right (RR) G-G');
plot(G_lat_rr_b, G_long_rr_b, 'r.', 'MarkerSize', 2); plot(G_lat_rr_a, G_long_rr_a, 'b.', 'MarkerSize', 2);

% =========================================================================
% FIG 10: STEP STEER TV CONTROL INTENT (TRANSIENT SPIKE)
% =========================================================================
Delta_T_a = Trr_m_a - Trl_m_a; 
figure('Name', 'Fig 10: Torque Vectoring Intent Analysis', 'Color', 'w', 'Position', [250, 250, 1000, 800]);

subplot(3,1,1);
yyaxis left; plot(t_a, delta_a_deg, 'k--', 'LineWidth', 1.5); ylabel('Steering Input (deg)');
yyaxis right; plot(t_a, r_a_deg, 'b-', 'LineWidth', 1.5); ylabel('Yaw Rate (deg/s)');
title('Step Steer: Instantaneous Driver Request vs Response'); 
xlim([10 20]); grid on; % Zoomed in on the step event

subplot(3,1,2);
plot(t_a, Trr_m_a, 'b-', 'LineWidth', 1); hold on; grid on; plot(t_a, Trl_m_a, 'g-', 'LineWidth', 1);
ylabel('Motor Torque (Nm)'); legend('Rear Right', 'Rear Left', 'Location', 'best');
title('Transient Actuator Shock Response');
xlim([10 20]);

subplot(3,1,3);
yyaxis left; plot(t_a, delta_a_deg, 'k--', 'LineWidth', 1.5); ylabel('Steering Input (deg)');
yyaxis right; plot(t_a, Delta_T_a, 'm-', 'LineWidth', 1.5); ylabel('\Delta Torque (Trr - Trl) [Nm]');
title('TV Intent: Initial Agility Spike vs Steady-State Hold');
legend('Steering Input', '\Delta Torque (Applied Yaw Moment)', 'Location', 'best');
xlim([10 20]); grid on; xlabel('Time (s)');

disp('====================================================');
disp('-> All Post-Processing Dashboards Rendered Successfully.');