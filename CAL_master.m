% =========================================================================
% MASTER SIMULATION & TELEMETRY SCRIPT: ISO 4138 SKIDPAD (CAL) A/B TESTING
% Models: CAL_3a (Dual Motor / TV) vs CAL_3b (Single Motor / Open Diff)
% =========================================================================
clear; clc; close all;

%% 1. USER INPUT & INITIALIZATION
disp('====================================================');
disp('   ISO 4138 CONSTANT RADIUS (CAL) - A/B TEST SUITE  ');
disp('====================================================');

disp('-> Loading Vehicle Parameters & Velocity Sweep Profile...');
% init_skidpad; 

%% 2. EXECUTE SIMULATIONS (With Workspace Wipe Protection)
disp('-> Running Dual Motor TV Simulation (CAL_3a)...');
out_a = sim('CAL_3a', 'ReturnWorkspaceOutputs', 'on');
save('temp_out_a.mat', 'out_a'); 
disp('   [CAL_3a Complete & Saved]');

disp('-> Running Single Motor Simulation (CAL_3b)...');
out_b = sim('CAL_3b', 'ReturnWorkspaceOutputs', 'on');
disp('   [CAL_3b Complete]');

load('temp_out_a.mat');
delete('temp_out_a.mat'); 

%% 3. DATA EXTRACTION: CHASSIS DYNAMICS
disp('-> Extracting Chassis Telemetry Data...');

t_a = out_a.tout;
t_b = out_b.tout;

% ----------------- VEHICLE A -----------------
X_a = squeeze(out_a.X_curr_a.Data);
Y_a = squeeze(out_a.Y_curr_a.Data);
vx_a = squeeze(out_a.vx_curr_a.Data);
vy_a = squeeze(out_a.vy_curr_a.Data);
r_a_rad = squeeze(out_a.r_curr_a.Data);
r_a_deg = r_a_rad * (180/pi);
delta_a_deg = squeeze(out_a.delta_a.Data) * (180/pi);
Trr_m_a = squeeze(out_a.Trr_m_a.Data);
Trl_m_a = squeeze(out_a.Trl_m_a.Data);
beta_a_deg = atan2(vy_a, vx_a) * (180/pi);
Ay_a_G = (vx_a .* r_a_rad) / veh_params.g;

% ----------------- VEHICLE B ---------------
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
Ay_b_G = (vx_b .* r_b_rad) / veh_params.g;
%% 4. DATA EXTRACTION: TIRE KINEMATICS & DYNAMICS
disp('-> Extracting Tire Dynamics & Calculating Kinematics...');

Rw = veh_params.Rw;      
tw = veh_params.tw;      
mu_max = veh_params.mu_max;   
lf = veh_params.lf;       
lr = veh_params.lr;       

Fx_rr_a = squeeze(out_a.Fx_rr_a.Data); Fx_rl_a = squeeze(out_a.Fx_rl_a.Data);
Fx_fr_a = squeeze(out_a.Fx_fr_a.Data); Fx_fl_a = squeeze(out_a.Fx_fl_a.Data);
Fy_rr_a = squeeze(out_a.Fy_rr_a.Data); Fy_rl_a = squeeze(out_a.Fy_rl_a.Data);
Fy_fr_a = squeeze(out_a.Fy_fr_a.Data); Fy_fl_a = squeeze(out_a.Fy_fl_a.Data);
Fz_rr_a = squeeze(out_a.Fz_rr_a.Data); Fz_rl_a = squeeze(out_a.Fz_rl_a.Data);
Fz_fr_a = squeeze(out_a.Fz_fr_a.Data); Fz_fl_a = squeeze(out_a.Fz_fl_a.Data);
w_rr_a = squeeze(out_a.w_rr_a.Data); w_rl_a = squeeze(out_a.w_rl_a.Data);
w_fr_a = squeeze(out_a.w_fr_a.Data); w_fl_a = squeeze(out_a.w_fl_a.Data);

Fx_rr_b = squeeze(out_b.Fx_rr_b.Data); Fx_rl_b = squeeze(out_b.Fx_rl_b.Data);
Fx_fr_b = squeeze(out_b.Fx_fr_b.Data); Fx_fl_b = squeeze(out_b.Fx_fl_b.Data);
Fy_rr_b = squeeze(out_b.Fy_rr_b.Data); Fy_rl_b = squeeze(out_b.Fy_rl_b.Data);
Fy_fr_b = squeeze(out_b.Fy_fr_b.Data); Fy_fl_b = squeeze(out_b.Fy_fl_b.Data);
Fz_rr_b = squeeze(out_b.Fz_rr_b.Data); Fz_rl_b = squeeze(out_b.Fz_rl_b.Data);
Fz_fr_b = squeeze(out_b.Fz_fr_b.Data); Fz_fl_b = squeeze(out_b.Fz_fl_b.Data);
w_rr_b = squeeze(out_b.w_rr_b.Data); w_rl_b = squeeze(out_b.w_rl_b.Data);
w_fr_b = squeeze(out_b.w_fr_b.Data); w_fl_b = squeeze(out_b.w_fl_b.Data);

Fz_lim_rr_a = mu_max * Fz_rr_a; Fz_lim_rl_a = mu_max * Fz_rl_a;
Fz_lim_fr_a = mu_max * Fz_fr_a; Fz_lim_fl_a = mu_max * Fz_fl_a;
Fz_lim_rr_b = mu_max * Fz_rr_b; Fz_lim_rl_b = mu_max * Fz_rl_b;
Fz_lim_fr_b = mu_max * Fz_fr_b; Fz_lim_fl_b = mu_max * Fz_fl_b;

F_res_fl_a = sqrt(Fx_fl_a.^2 + Fy_fl_a.^2); F_res_fr_a = sqrt(Fx_fr_a.^2 + Fy_fr_a.^2);
F_res_rl_a = sqrt(Fx_rl_a.^2 + Fy_rl_a.^2); F_res_rr_a = sqrt(Fx_rr_a.^2 + Fy_rr_a.^2);
F_res_fl_b = sqrt(Fx_fl_b.^2 + Fy_fl_b.^2); F_res_fr_b = sqrt(Fx_fr_b.^2 + Fy_fr_b.^2);
F_res_rl_b = sqrt(Fx_rl_b.^2 + Fy_rl_b.^2); F_res_rr_b = sqrt(Fx_rr_b.^2 + Fy_rr_b.^2);

v_hub_rr_a = max(vx_a + (tw/2)*r_a_rad, 1.0); v_hub_rl_a = max(vx_a - (tw/2)*r_a_rad, 1.0);
v_hub_fr_a = max(vx_a + (tw/2)*r_a_rad, 1.0); v_hub_fl_a = max(vx_a - (tw/2)*r_a_rad, 1.0);
kappa_rr_a = (w_rr_a * Rw - v_hub_rr_a) ./ v_hub_rr_a; kappa_rl_a = (w_rl_a * Rw - v_hub_rl_a) ./ v_hub_rl_a;
kappa_fr_a = (w_fr_a * Rw - v_hub_fr_a) ./ v_hub_fr_a; kappa_fl_a = (w_fl_a * Rw - v_hub_fl_a) ./ v_hub_fl_a;

v_hub_rr_b = max(vx_b + (tw/2)*r_b_rad, 1.0); v_hub_rl_b = max(vx_b - (tw/2)*r_b_rad, 1.0);
v_hub_fr_b = max(vx_b + (tw/2)*r_b_rad, 1.0); v_hub_fl_b = max(vx_b - (tw/2)*r_b_rad, 1.0);
kappa_rr_b = (w_rr_b * Rw - v_hub_rr_b) ./ v_hub_rr_b; kappa_rl_b = (w_rl_b * Rw - v_hub_rl_b) ./ v_hub_rl_b;
kappa_fr_b = (w_fr_b * Rw - v_hub_fr_b) ./ v_hub_fr_b; kappa_fl_b = (w_fl_b * Rw - v_hub_fl_b) ./ v_hub_fl_b;

v_hub_y_f_a = vy_a + (lf * r_a_rad); v_hub_y_r_a = vy_a - (lr * r_a_rad);
alpha_fl_a = ((delta_a_deg * pi/180) - atan2(v_hub_y_f_a, v_hub_fl_a)) * (180/pi);
alpha_fr_a = ((delta_a_deg * pi/180) - atan2(v_hub_y_f_a, v_hub_fr_a)) * (180/pi);
alpha_rl_a = (0 - atan2(v_hub_y_r_a, v_hub_rl_a)) * (180/pi);
alpha_rr_a = (0 - atan2(v_hub_y_r_a, v_hub_rr_a)) * (180/pi);

v_hub_y_f_b = vy_b + (lf * r_b_rad); v_hub_y_r_b = vy_b - (lr * r_b_rad);
alpha_fl_b = ((delta_b_deg * pi/180) - atan2(v_hub_y_f_b, v_hub_fl_b)) * (180/pi);
alpha_fr_b = ((delta_b_deg * pi/180) - atan2(v_hub_y_f_b, v_hub_fr_b)) * (180/pi);
alpha_rl_b = (0 - atan2(v_hub_y_r_b, v_hub_rl_b)) * (180/pi);
alpha_rr_b = (0 - atan2(v_hub_y_r_b, v_hub_rr_b)) * (180/pi);

min_Fz = 1.0; 
G_lat_fl_a = Fy_fl_a ./ max(Fz_fl_a, min_Fz); G_long_fl_a = Fx_fl_a ./ max(Fz_fl_a, min_Fz);
G_lat_fr_a = Fy_fr_a ./ max(Fz_fr_a, min_Fz); G_long_fr_a = Fx_fr_a ./ max(Fz_fr_a, min_Fz);
G_lat_rl_a = Fy_rl_a ./ max(Fz_rl_a, min_Fz); G_long_rl_a = Fx_rl_a ./ max(Fz_rl_a, min_Fz);
G_lat_rr_a = Fy_rr_a ./ max(Fz_rr_a, min_Fz); G_long_rr_a = Fx_rr_a ./ max(Fz_rr_a, min_Fz);

G_lat_fl_b = Fy_fl_b ./ max(Fz_fl_b, min_Fz); G_long_fl_b = Fx_fl_b ./ max(Fz_fl_b, min_Fz);
G_lat_fr_b = Fy_fr_b ./ max(Fz_fr_b, min_Fz); G_long_fr_b = Fx_fr_b ./ max(Fz_fr_b, min_Fz);
G_lat_rl_b = Fy_rl_b ./ max(Fz_rl_b, min_Fz); G_long_rl_b = Fx_rl_b ./ max(Fz_rl_b, min_Fz);
G_lat_rr_b = Fy_rr_b ./ max(Fz_rr_b, min_Fz); G_long_rr_b = Fx_rr_b ./ max(Fz_rr_b, min_Fz);

%% 5. PLOTTING AND VISUALIZATION
disp('-> Generating Telemetry Dashboards (Figures 1 to 11)...');

% =========================================================================
% FIG 1 to FIG 9: Chassis & Dynamics Tracking
% =========================================================================
% =========================================================================
% FIG 1 to FIG 9: Chassis & Dynamics Tracking
% =========================================================================
% =========================================================================
% FIG 1: ISO 4138 Path Tracking (Hardcoded Entry)
% =========================================================================
subplot(2,2,1);

% 1. Hardcoded Geometry
L_straight = 50; % Entry straight distance (m)
R_target = 50;   % Skidpad radius (m)

% 2. Construct Target Path (Straight + Circle)
% Circle starts from -pi/2 to tangent smoothly with the X-axis
theta_circ = linspace(-pi/2, 1.5*pi, 100); 

X_target = [0, L_straight, L_straight + R_target * cos(theta_circ)];
Y_target = [0, 0, R_target + R_target * sin(theta_circ)];

% 3. Plotting
plot(X_target, Y_target, 'k--', 'LineWidth', 1.5); hold on; grid on;
plot(X_a, Y_a, 'b-', 'LineWidth', 1.5); 
plot(X_b, Y_b, 'r-', 'LineWidth', 1.5);

title('Path Tracking (50m Target)'); 
xlabel('X (m)'); ylabel('Y (m)');
legend('Target Path', 'Dual Motor TV', 'Single Motor', 'Location', 'best'); 
axis equal;
subplot(2,2,2);
plot(Ay_b_G, delta_b_deg, 'r-', 'LineWidth', 2); hold on; grid on; plot(Ay_a_G, delta_a_deg, 'b-', 'LineWidth', 2);
L = veh_params.L; R = 50; ackermann_deg = (L/R) * (180/pi);
plot([0, max(Ay_a_G)], [ackermann_deg, ackermann_deg], 'k--', 'LineWidth', 1.5);
title('Vehicle Handling Diagram'); xlabel('Lateral Acceleration (G)'); ylabel('Steering Angle (deg)');
legend('Single Motor', 'Dual Motor TV', 'Ackermann Baseline', 'Location', 'northwest'); xlim([0 1.2]);

subplot(2,2,[3,4]);
plot(t_a, beta_a_deg, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, beta_b_deg, 'r-', 'LineWidth', 1.5);
title('Vehicle Sideslip Angle (\beta)'); xlabel('Time (s)'); ylabel('Sideslip (deg)');
legend('Dual Motor TV (\beta_a)', 'Single Motor (\beta_b)', 'Location', 'best');

figure('Name', 'Fig 2: Motor Torque Allocation', 'Color', 'w', 'Position', [100, 100, 800, 600]);

subplot(2,1,1); 
plot(t_a, Trr_m_a, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_a, Trl_m_a, 'g-', 'LineWidth', 1.5);
title('Vehicle A (Dual Motor) Torque Allocation'); xlabel('Time (s)'); ylabel('Torque (Nm)');
legend('Rear Right', 'Rear Left', 'Location', 'best');

subplot(2,1,2); 
% --- Left Y-Axis: Motor Torque ---
yyaxis left;
plot(t_b, T_total_b, 'r-', 'LineWidth', 1.5); hold on; grid on;
ylabel('Combined Torque (Nm)');

% --- Right Y-Axis: Friction Brake Torque ---
yyaxis right;
plot(t_b, Trr_fric_b, 'b--', 'LineWidth', 1.2); hold on;
plot(t_b, Trl_fric_b, 'g--', 'LineWidth', 1.2);
ylabel('Friction Brake Torque (Nm)');

title('Vehicle B (Single Motor) Total Axle Torque & Brake Vectoring'); 
xlabel('Time (s)'); 
legend('Total Output', 'Rear Right Brake (T_{rr\_fric})', 'Rear Left Brake (T_{rl\_fric})', 'Location', 'best');

figure('Name', 'Fig 3: Steering & Lateral Velocity', 'Color', 'w', 'Position', [150, 150, 800, 600]);
subplot(2,1,1); plot(t_a, delta_a_deg, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, delta_b_deg, 'r-', 'LineWidth', 1.5);
title('Actual Steering Angle'); xlabel('Time (s)'); ylabel('Angle (deg)'); legend('Dual Motor', 'Single Motor', 'Location', 'best');
subplot(2,1,2); plot(t_a, vy_a, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, vy_b, 'r-', 'LineWidth', 1.5);
title('Lateral Velocity (V_y)'); xlabel('Time (s)'); ylabel('Velocity (m/s)'); legend('Dual Motor', 'Single Motor', 'Location', 'best');

figure('Name', 'Fig 4: Longitudinal Forces (Fx)', 'Color', 'w', 'Position', [200, 200, 1000, 700]);
subplot(2,2,1); plot(t_a, Fx_fl_a, 'b-'); hold on; grid on; plot(t_b, Fx_fl_b, 'r-'); title('FL Fx');
subplot(2,2,2); plot(t_a, Fx_fr_a, 'b-'); hold on; grid on; plot(t_b, Fx_fr_b, 'r-'); title('FR Fx'); 
subplot(2,2,3); plot(t_a, Fx_rl_a, 'b-'); hold on; grid on; plot(t_b, Fx_rl_b, 'r-'); title('RL Fx'); 
subplot(2,2,4); plot(t_a, Fx_rr_a, 'b-'); hold on; grid on; plot(t_b, Fx_rr_b, 'r-'); title('RR Fx'); legend('Dual', 'Single');

figure('Name', 'Fig 5: Lateral Forces (Fy)', 'Color', 'w', 'Position', [250, 250, 1000, 700]);
subplot(2,2,1); plot(t_a, Fy_fl_a, 'b-'); hold on; grid on; plot(t_b, Fy_fl_b, 'r-'); title('FL Fy');
subplot(2,2,2); plot(t_a, Fy_fr_a, 'b-'); hold on; grid on; plot(t_b, Fy_fr_b, 'r-'); title('FR Fy'); 
subplot(2,2,3); plot(t_a, Fy_rl_a, 'b-'); hold on; grid on; plot(t_b, Fy_rl_b, 'r-'); title('RL Fy'); 
subplot(2,2,4); plot(t_a, Fy_rr_a, 'b-'); hold on; grid on; plot(t_b, Fy_rr_b, 'r-'); title('RR Fy'); legend('Dual', 'Single');

figure('Name', 'Fig 6: Grip Utilization (F_res vs Limit)', 'Color', 'w', 'Position', [300, 300, 1000, 700]);
subplot(2,2,1); plot(t_a, Fz_lim_fl_a, 'b--'); hold on; grid on; plot(t_a, F_res_fl_a, 'b-'); plot(t_b, Fz_lim_fl_b, 'r--'); plot(t_b, F_res_fl_b, 'r-'); title('FL Grip');
subplot(2,2,2); plot(t_a, Fz_lim_fr_a, 'b--'); hold on; grid on; plot(t_a, F_res_fr_a, 'b-'); plot(t_b, Fz_lim_fr_b, 'r--'); plot(t_b, F_res_fr_b, 'r-'); title('FR Grip'); 
subplot(2,2,3); plot(t_a, Fz_lim_rl_a, 'b--'); hold on; grid on; plot(t_a, F_res_rl_a, 'b-'); plot(t_b, Fz_lim_rl_b, 'r--'); plot(t_b, F_res_rl_b, 'r-'); title('RL Grip');
subplot(2,2,4); plot(t_a, Fz_lim_rr_a, 'b--'); hold on; grid on; plot(t_a, F_res_rr_a, 'b-'); plot(t_b, Fz_lim_rr_b, 'r--'); plot(t_b, F_res_rr_b, 'r-'); title('RR Grip');

figure('Name', 'Fig 7: Longitudinal Slip (\kappa)', 'Color', 'w', 'Position', [350, 350, 1000, 700]);
subplot(2,2,1); plot(t_a, kappa_fl_a, 'b-'); hold on; grid on; plot(t_b, kappa_fl_b, 'r-'); title('FL \kappa');
subplot(2,2,2); plot(t_a, kappa_fr_a, 'b-'); hold on; grid on; plot(t_b, kappa_fr_b, 'r-'); title('FR \kappa'); 
subplot(2,2,3); plot(t_a, kappa_rl_a, 'b-'); hold on; grid on; plot(t_b, kappa_rl_b, 'r-'); title('RL \kappa');
subplot(2,2,4); plot(t_a, kappa_rr_a, 'b-'); hold on; grid on; plot(t_b, kappa_rr_b, 'r-'); title('RR \kappa');

figure('Name', 'Fig 8: Lateral Slip Angles (\alpha)', 'Color', 'w', 'Position', [400, 400, 1000, 700]);
subplot(2,2,1); plot(t_a, alpha_fl_a, 'b-'); hold on; grid on; plot(t_b, alpha_fl_b, 'r-'); title('FL \alpha');
subplot(2,2,2); plot(t_a, alpha_fr_a, 'b-'); hold on; grid on; plot(t_b, alpha_fr_b, 'r-'); title('FR \alpha'); 
subplot(2,2,3); plot(t_a, alpha_rl_a, 'b-'); hold on; grid on; plot(t_b, alpha_rl_b, 'r-'); title('RL \alpha');
subplot(2,2,4); plot(t_a, alpha_rr_a, 'b-'); hold on; grid on; plot(t_b, alpha_rr_b, 'r-'); title('RR \alpha');

figure('Name', 'Fig 9: Friction Circles (G-G)', 'Color', 'w', 'Position', [450, 450, 1000, 800]);
theta = linspace(0, 2*pi, 100); circle_x = mu_max * cos(theta); circle_y = mu_max * sin(theta);
for i = 1:4
    subplot(2,2,i); plot(circle_x, circle_y, 'k-', 'LineWidth', 2); hold on; grid on; 
    plot([-2, 2], [0, 0], 'k:', 'LineWidth', 1); plot([0, 0], [-2, 2], 'k:', 'LineWidth', 1); axis equal; xlim([-1.8 1.8]); ylim([-1.8 1.8]);
end
subplot(2,2,1); title('FL G-G'); plot(G_lat_fl_b, G_long_fl_b, 'r.', 'MarkerSize', 2); plot(G_lat_fl_a, G_long_fl_a, 'b.', 'MarkerSize', 2);
subplot(2,2,2); title('FR G-G'); plot(G_lat_fr_b, G_long_fr_b, 'r.', 'MarkerSize', 2); plot(G_lat_fr_a, G_long_fr_a, 'b.', 'MarkerSize', 2);
subplot(2,2,3); title('RL G-G'); plot(G_lat_rl_b, G_long_rl_b, 'r.', 'MarkerSize', 2); plot(G_lat_rl_a, G_long_rl_a, 'b.', 'MarkerSize', 2);
subplot(2,2,4); title('RR G-G'); plot(G_lat_rr_b, G_long_rr_b, 'r.', 'MarkerSize', 2); plot(G_lat_rr_a, G_long_rr_a, 'b.', 'MarkerSize', 2);

% =========================================================================
% FIG 10 & 11: ELECTRICAL TELEMETRY EXTRACTION & PLOTTING
% =========================================================================
try
    SOC_a      = squeeze(out_a.SOC_a.Data);
    T_batt_a   = squeeze(out_a.T_batt_a.Data);
    I_batt_a   = squeeze(out_a.I_batt_a.Data);
    Pwr_Elec_a = squeeze(out_a.Pwr_Elec_a.Data);
    
    SOC_b      = squeeze(out_b.SOC_b.Data);
    T_batt_b   = squeeze(out_b.T_batt_b.Data);
    I_batt_b   = squeeze(out_b.I_batt_b.Data);
    Pwr_Elec_b = squeeze(out_b.Pwr_Elec_b.Data);
catch
    disp('Warning: Could not extract electrical data. Check workspace signal names.');
end

% ---------------------------------------------------------
% FIGURE 10: Battery States (SOC, Current, Temperature)
% ---------------------------------------------------------
figure('Name', 'Fig 10: Battery States', 'Color', 'w', 'Position', [500, 500, 800, 800]);

subplot(3,1,1);
plot(t_a, SOC_a, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, SOC_b, 'r-', 'LineWidth', 1.5);
title('Battery State of Charge (SOC)'); ylabel('SOC (%)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best');

subplot(3,1,2);
plot(t_a, I_batt_a, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, I_batt_b, 'r-', 'LineWidth', 1.5);
title('Battery Current'); ylabel('Current (A)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best');

subplot(3,1,3);
plot(t_a, T_batt_a, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, T_batt_b, 'r-', 'LineWidth', 1.5);
title('Battery Temperature'); xlabel('Time (s)'); ylabel('Temperature (°C)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best');

% ---------------------------------------------------------
% FIGURE 11: Electrical Power Analysis
% ---------------------------------------------------------
figure('Name', 'Fig 11: Electrical Power Analysis', 'Color', 'w', 'Position', [550, 550, 800, 600]);

subplot(2,1,1);
plot(t_a, I_batt_a, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, I_batt_b, 'r-', 'LineWidth', 1.5);
title('Battery Current Draw'); ylabel('Current (A)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best');

subplot(2,1,2);
% Convert Watts to Kilowatts for cleaner Y-axis scaling
plot(t_a, Pwr_Elec_a / 1000, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, Pwr_Elec_b / 1000, 'r-', 'LineWidth', 1.5);
title('Gross Electrical Power Consumption'); xlabel('Time (s)'); ylabel('Power (kW)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best');

disp('-> All Post-Processing Dashboards Rendered Successfully.');