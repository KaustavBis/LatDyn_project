% =========================================================================
% MASTER SIMULATION SCRIPT: MU-SPLIT DISTURBANCE (MUS)
% Models: MUS_1a (Dual Motor TV) vs MUS_1b (Single Motor)
% =========================================================================
clear; clc; close all;

%% 1. INITIALIZATION & SIMULATION
disp('====================================================');
disp('   EXTREME ASYMMETRIC MU-SPLIT DISTURBANCE          ');
disp('====================================================');

MUS; % Run the startup/planner script to ensure workspace variables are fresh

disp('-> Running Dual Motor TV Simulation (MUS_1a)...');
out_a = sim('MUS_1a', 'ReturnWorkspaceOutputs', 'on');
save('temp_out_a.mat', 'out_a'); 

disp('-> Running Single Motor Baseline (MUS_1b)...');
out_b = sim('MUS_1b', 'ReturnWorkspaceOutputs', 'on');

load('temp_out_a.mat'); delete('temp_out_a.mat'); 

%% 2. DATA EXTRACTION
disp('-> Extracting Telemetry Data...');

t_a = out_a.tout; t_b = out_b.tout;

% Inputs & Environment
delta_a_deg = squeeze(out_a.delta_a.Data) * (180/pi);
mu_L = interp1(mu_profile_left(:,1), mu_profile_left(:,2), t_a);
mu_R = interp1(mu_profile_right(:,1), mu_profile_right(:,2), t_a);

% Core Velocities
vx_a = squeeze(out_a.vx_curr_a.Data); vy_a = squeeze(out_a.vy_curr_a.Data);
vx_b = squeeze(out_b.vx_curr_b.Data); vy_b = squeeze(out_b.vy_curr_b.Data);

% Chassis Dynamics (Deriving Beta Mathematically)
r_a_rad = squeeze(out_a.r_curr_a.Data); 
r_b_rad = squeeze(out_b.r_curr_b.Data);
r_a_deg = r_a_rad * (180/pi);
r_b_deg = r_b_rad * (180/pi);
beta_a_deg = atan2(vy_a, vx_a) * (180/pi); 
beta_b_deg = atan2(vy_b, vx_b) * (180/pi); 

% Actuators - Vehicle A
Trr_m_a = squeeze(out_a.Trr_m_a.Data); Trl_m_a = squeeze(out_a.Trl_m_a.Data);
Trr_f_a = squeeze(out_a.Trr_f_a.Data); Trl_f_a = squeeze(out_a.Trl_f_a.Data);
Delta_T_a = Trr_m_a - Trl_m_a; 

% Actuators - Vehicle B
Trr_m_b = squeeze(out_b.Trr_m_b.Data); Trl_m_b = squeeze(out_b.Trl_m_b.Data);
T_mot_total_b = Trr_m_b + Trl_m_b;
Trr_f_b = squeeze(out_b.Trr_f_b.Data); Trl_f_b = squeeze(out_b.Trl_f_b.Data);

% =========================================================================
% NEW: WHEEL SPEED & SLIP EXTRACTION
% =========================================================================
% Extract packed wheel speed vectors [w_fl, w_fr, w_rl, w_rr]
w_vec_a_data = squeeze(out_a.w_vec_a.Data);
w_vec_b_data = squeeze(out_b.w_vec_b.Data);

% Handle array orientation safely depending on Simulink version
if size(w_vec_a_data, 1) == 4
    w_rl_a = w_vec_a_data(3,:)'; w_rr_a = w_vec_a_data(4,:)';
else
    w_rl_a = w_vec_a_data(:,3); w_rr_a = w_vec_a_data(:,4);
end

if size(w_vec_b_data, 1) == 4
    w_rl_b = w_vec_b_data(3,:)'; w_rr_b = w_vec_b_data(4,:)';
else
    w_rl_b = w_vec_b_data(:,3); w_rr_b = w_vec_b_data(:,4);
end

% Convert to RPM
RPM_rl_a = w_rl_a * (30/pi); RPM_rr_a = w_rr_a * (30/pi);
RPM_rl_b = w_rl_b * (30/pi); RPM_rr_b = w_rr_b * (30/pi);

% Slip Math Parameters
Rw = 0.34; tw = 1.65; lr = 1.3; 

% Vehicle A Slip Calculations
v_hub_x_rl_a = vx_a - (tw/2)*r_a_rad;
v_hub_x_rr_a = vx_a + (tw/2)*r_a_rad;
v_hub_y_rear_a = vy_a - lr*r_a_rad;

kappa_rl_a = (w_rl_a .* Rw - v_hub_x_rl_a) ./ max(abs(v_hub_x_rl_a), 1.0);
kappa_rr_a = (w_rr_a .* Rw - v_hub_x_rr_a) ./ max(abs(v_hub_x_rr_a), 1.0);
alpha_rl_a = atan2(v_hub_y_rear_a, v_hub_x_rl_a) * (180/pi);
alpha_rr_a = atan2(v_hub_y_rear_a, v_hub_x_rr_a) * (180/pi);

% Vehicle B Slip Calculations
v_hub_x_rl_b = vx_b - (tw/2)*r_b_rad;
v_hub_x_rr_b = vx_b + (tw/2)*r_b_rad;
v_hub_y_rear_b = vy_b - lr*r_b_rad;

kappa_rl_b = (w_rl_b .* Rw - v_hub_x_rl_b) ./ max(abs(v_hub_x_rl_b), 1.0);
kappa_rr_b = (w_rr_b .* Rw - v_hub_x_rr_b) ./ max(abs(v_hub_x_rr_b), 1.0);
alpha_rl_b = atan2(v_hub_y_rear_b, v_hub_x_rl_b) * (180/pi);
alpha_rr_b = atan2(v_hub_y_rear_b, v_hub_x_rr_b) * (180/pi);


%% 3. RENDERING DASHBOARDS
disp('-> Rendering Dashboards...');

% Dynamically link markers to the planner variables
if exist('t_drop', 'var'); ev1 = t_drop; else; ev1 = 5.0; end
if exist('t_recover', 'var'); ev2 = t_recover; else; ev2 = 15.0; end
view_window = [0 max(ev2 + 5, 20)]; % Auto-scale x-axis to fit events

% FIG 1: The Disturbance
figure('Name', 'Fig 1: Environmental Disturbance', 'Color', 'w', 'Position', [50, 50, 900, 500]);
subplot(2,1,1);
plot(t_a, mu_L, 'b-', 'LineWidth', 2); hold on; plot(t_a, mu_R, 'r-', 'LineWidth', 2);
xline(ev1, 'k--', '\mu Drop', 'LabelVerticalAlignment', 'bottom'); 
xline(ev2, 'k--', '\mu Restore', 'LabelVerticalAlignment', 'bottom');
title('Tire Friction Profile (The Disturbance)'); ylabel('\mu'); legend('Left Tires', 'Right Tires');
xlim(view_window); ylim([0 1.7]); grid on;
subplot(2,1,2);
plot(t_a, delta_a_deg, 'k-', 'LineWidth', 2);
title('Driver Steering Input'); xlabel('Time (s)'); ylabel('Steer Angle (deg)');
xlim(view_window); ylim([-2 2]); grid on;

% FIG 2: Vehicle Response
figure('Name', 'Fig 2: Disturbance Rejection', 'Color', 'w', 'Position', [100, 100, 900, 600]);
subplot(2,1,1);
plot(t_a, r_a_deg, 'b-', 'LineWidth', 1.5); hold on; plot(t_b, r_b_deg, 'r-', 'LineWidth', 1.5);
xline(ev1, 'k--', 'Ice Patch Hit'); xline(ev2, 'k--', 'Tarmac Recovery');
title('Yaw Rate Response'); ylabel('Yaw Rate (deg/s)');
legend('Vehicle A (Dual Motor TV)', 'Vehicle B (Single Motor)', 'Location', 'best');
xlim(view_window); grid on;
subplot(2,1,2);
plot(t_a, beta_a_deg, 'b-', 'LineWidth', 1.5); hold on; plot(t_b, beta_b_deg, 'r-', 'LineWidth', 1.5);
xline(ev1, 'k--', 'Ice Patch Hit'); xline(ev2, 'k--', 'Tarmac Recovery');
title('Vehicle Sideslip Angle \beta'); xlabel('Time (s)'); ylabel('Sideslip (deg)');
xlim(view_window); grid on;

% FIG 3: TV Intent (The Catch)
figure('Name', 'Fig 3: TV Control Intent', 'Color', 'w', 'Position', [150, 150, 900, 400]);
plot(t_a, Delta_T_a, 'm-', 'LineWidth', 2); hold on;
xline(ev1, 'k--', 'Ice Patch Hit'); xline(ev2, 'k--', 'Tarmac Recovery');
title('Torque Vectoring Intent (Right-Left Torque Delta)');
xlabel('Time (s)'); ylabel('\Delta Torque (Nm)');
legend('\DeltaT applied to catch yaw error', 'Location', 'best');
xlim(view_window); grid on;

% FIG 4: MOTOR TORQUE ALLOCATION (PROPULSION VECTORING)
figure('Name', 'Motor Torque Allocation Comparison', 'Color', 'w', 'Position', [300, 300, 1000, 600]);
subplot(2,1,1);
plot(t_a, Trr_m_a, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_a, Trl_m_a, 'g-', 'LineWidth', 1.5);
xline(ev1, 'k--'); xline(ev2, 'k--');
title('Vehicle A (Dual Motor TV): Independent Motor Torques');
ylabel('Motor Torque (Nm)');
legend('Rear Right Motor', 'Rear Left Motor', 'Location', 'best');
xlim(view_window);
subplot(2,1,2);
plot(t_b, T_mot_total_b, 'r-', 'LineWidth', 1.5); grid on; hold on;
xline(ev1, 'k--'); xline(ev2, 'k--');
title('Vehicle B (Single Motor): Total Applied Axle Torque');
xlabel('Time (s)'); ylabel('Motor Torque (Nm)');
legend('Total Rear Axle Motor Torque', 'Location', 'best');
xlim(view_window);

% FIG 5: FRICTION BRAKE ALLOCATION (BRAKE VECTORING)
figure('Name', 'Friction Brake Vectoring Comparison', 'Color', 'w', 'Position', [350, 350, 1000, 600]);
subplot(2,1,1);
plot(t_a, Trr_f_a, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_a, Trl_f_a, 'g-', 'LineWidth', 1.5);
xline(ev1, 'k--'); xline(ev2, 'k--');
title('Vehicle A (Dual Motor TV): Applied Friction Brakes');
ylabel('Brake Torque (Nm)');
legend('Rear Right Brake', 'Rear Left Brake', 'Location', 'best');
xlim(view_window);
subplot(2,1,2);
plot(t_b, Trr_f_b, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, Trl_f_b, 'g-', 'LineWidth', 1.5);
xline(ev1, 'k--'); xline(ev2, 'k--');
title('Vehicle B (Single Motor ESC): Asymmetric Brake Vectoring');
xlabel('Time (s)'); ylabel('Brake Torque (Nm)');
legend('Rear Right Brake', 'Rear Left Brake', 'Location', 'best');
xlim(view_window);

% FIG 6: REAR WHEEL RPM COMPARISON
figure('Name', 'Fig 6: Rear Wheel RPM', 'Color', 'w', 'Position', [400, 400, 1000, 600]);
subplot(2,1,1);
plot(t_a, RPM_rl_a, 'g-', 'LineWidth', 1.5); hold on; grid on;
plot(t_a, RPM_rr_a, 'b-', 'LineWidth', 1.5);
xline(ev1, 'k--', 'Ice Hit', 'LabelVerticalAlignment', 'bottom'); 
xline(ev2, 'k--', 'Tarmac Recovery', 'LabelVerticalAlignment', 'bottom');
title('Vehicle A (Dual Motor TV): Rear Wheel RPM');
ylabel('Speed (RPM)'); legend('Rear Left', 'Rear Right', 'Location', 'best');
xlim(view_window);
subplot(2,1,2);
plot(t_b, RPM_rl_b, 'g-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, RPM_rr_b, 'b-', 'LineWidth', 1.5);
xline(ev1, 'k--', 'Ice Hit', 'LabelVerticalAlignment', 'bottom'); 
xline(ev2, 'k--', 'Tarmac Recovery', 'LabelVerticalAlignment', 'bottom');
title('Vehicle B (Single Motor ESC): Rear Wheel RPM');
xlabel('Time (s)'); ylabel('Speed (RPM)'); legend('Rear Left', 'Rear Right', 'Location', 'best');
xlim(view_window);

% FIG 7: LONGITUDINAL TIRE SLIP (KAPPA)
figure('Name', 'Fig 7: Longitudinal Slip', 'Color', 'w', 'Position', [450, 450, 1000, 600]);
subplot(2,1,1);
plot(t_a, kappa_rl_a, 'g-', 'LineWidth', 1.5); hold on; grid on;
plot(t_a, kappa_rr_a, 'b-', 'LineWidth', 1.5);
xline(ev1, 'k--', '\mu Drop', 'LabelVerticalAlignment', 'bottom'); 
xline(ev2, 'k--', '\mu Restore', 'LabelVerticalAlignment', 'bottom');
title('Vehicle A (Dual Motor TV): Longitudinal Slip Ratio (\kappa)');
ylabel('Slip Ratio (\kappa)'); legend('Rear Left', 'Rear Right', 'Location', 'best');
xlim(view_window); 
ylim('auto'); % <-- Auto-scales to capture massive slip spikes
subplot(2,1,2);
plot(t_b, kappa_rl_b, 'g-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, kappa_rr_b, 'b-', 'LineWidth', 1.5);
xline(ev1, 'k--', '\mu Drop', 'LabelVerticalAlignment', 'bottom'); 
xline(ev2, 'k--', '\mu Restore', 'LabelVerticalAlignment', 'bottom');
title('Vehicle B (Single Motor ESC): Longitudinal Slip Ratio (\kappa)');
xlabel('Time (s)'); ylabel('Slip Ratio (\kappa)'); legend('Rear Left', 'Rear Right', 'Location', 'best');
xlim(view_window); 
ylim('auto'); % <-- Auto-scales to capture massive slip spikes

disp('-> Extreme Mu-Split Testing Complete.');
%% =========================================================================
% CAR A PERFORMANCE DASHBOARD: MU-SPLIT TELEMETRY
% Focus: Four-Wheel Speeds, Slips, and Actuator Intent
% =========================================================================

% =========================================================================
% CAR A PERFORMANCE DASHBOARD: BRS MU-SPLIT TELEMETRY
% Focus: Rear-Wheel Speeds, Slips, and Actuator Intent
% =========================================================================

% 1. Extract Rear Wheel Speeds (w_vec indices 3 and 4)
if size(w_vec_a_data, 1) == 4
    w_rl_a = w_vec_a_data(3,:)'; w_rr_a = w_vec_a_data(4,:)';
else
    w_rl_a = w_vec_a_data(:,3); w_rr_a = w_vec_a_data(:,4);
end
RPM_rl_a = w_rl_a * (30/pi); RPM_rr_a = w_rr_a * (30/pi);

% 2. Calculate Rear Wheel Slips
v_hub_x_rl_a = vx_a - (tw/2)*r_a_rad;
v_hub_x_rr_a = vx_a + (tw/2)*r_a_rad;
kappa_rl_a = (w_rl_a .* Rw - v_hub_x_rl_a) ./ max(abs(v_hub_x_rl_a), 1.0);
kappa_rr_a = (w_rr_a .* Rw - v_hub_x_rr_a) ./ max(abs(v_hub_x_rr_a), 1.0);

% 3. Rendering Dashboard
figure('Name', 'Car A: BRS Performance Dashboard', 'Color', 'w', 'Position', [100, 50, 1100, 900]);

% --- Subplot 1: Rear Wheel Speeds ---
subplot(3,1,1);
plot(t_a, RPM_rl_a, 'g-', 'LineWidth', 2); hold on; grid on;
plot(t_a, RPM_rr_a, 'b-', 'LineWidth', 2);
xline(ev1, 'k--', 'Ice Hit', 'LabelVerticalAlignment', 'bottom'); 
xline(ev2, 'k--', 'Tarmac Recovery', 'LabelVerticalAlignment', 'bottom');
ylabel('Speed (RPM)'); title('Wheel RPMs (BRS Braking)');
legend('RL (Driven/Braked)', 'RR (Driven/Braked)', 'Location', 'northeast');
xlim(view_window);

% --- Subplot 2: Rear Wheel Slips ---
subplot(3,1,2);
plot(t_a, kappa_rl_a, 'g-', 'LineWidth', 2); hold on; grid on;
plot(t_a, kappa_rr_a, 'b-', 'LineWidth', 2);
yline(-0.05, 'r--', 'BRS Target (-0.05)', 'LineWidth', 1.5); 
xline(ev1, 'k--'); xline(ev2, 'k--');
ylabel('Slip Ratio (\kappa)'); title('Longitudinal Tire Slip Regulation (BRS)');
legend('RL', 'RR', 'Location', 'southeast');
xlim(view_window); 
ylim([-0.25 0.05]); % Scaled for negative slip (braking)

% --- Subplot 3: Motor Torques & TV Intent ---
subplot(3,1,3);
yyaxis left
plot(t_a, Trl_m_a, 'g-', 'LineWidth', 2); hold on; grid on;
plot(t_a, Trr_m_a, 'b-', 'LineWidth', 2);
ylabel('Motor Torque (Nm)');
set(gca, 'YColor', [0 0.5 0]); 

yyaxis right
plot(t_a, Delta_T_a, 'm-', 'LineWidth', 1.5, 'LineStyle', ':');
ylabel('TV Intent \DeltaT (Nm)');
set(gca, 'YColor', 'm'); 

xline(ev1, 'k--', 'Ice Hit', 'LabelVerticalAlignment', 'bottom'); 
xline(ev2, 'k--', 'Tarmac Recovery', 'LabelVerticalAlignment', 'bottom');
title('Actuator Allocation & Torque Vectoring Intent');
legend('RL Regen Torque', 'RR Regen Torque', 'TV Intent (\DeltaT)', 'Location', 'southwest');
xlim(view_window); xlabel('Time (s)');