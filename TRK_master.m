% =========================================================================
% MASTER SIMULATION SCRIPT: FULL TRACK LAP (TRK) - COMPREHENSIVE VALIDATION
% Models: TRK_1a (Dual Motor TV) vs TRK_1b (Single Motor)
% Location: Autodromo Nazionale Monza
% =========================================================================
clear; clc; close all;

%% 1. INITIALIZATION & SIMULATION
disp('====================================================');
disp('   MONZA LAP 1: DYNAMICS, ENERGY & THERMAL SUITE    ');
disp('====================================================');
TRK; % Load LQR, Track CSV, and Vehicle Parameters

disp('-> Running Dual Motor TV Simulation (TRK_1a)...');
out_a = sim('TRK_1a', 'ReturnWorkspaceOutputs', 'on');
save('temp_out_a.mat', 'out_a'); 

disp('-> Running Single Motor Baseline (TRK_1b)...');
out_b = sim('TRK_1b', 'ReturnWorkspaceOutputs', 'on');
load('temp_out_a.mat'); delete('temp_out_a.mat'); 

%% 2. DATA EXTRACTION & CORE KINEMATICS
disp('-> Extracting Telemetry & Computing Performance Metrics...');
t_a = out_a.tout; t_b = out_b.tout;
g = 9.81;

% Path Distance (Integrated s-parameter)
dist_a = out_a.s_a.Data(end); 
dist_b = out_b.s_b.Data(end);
s_a = squeeze(out_a.s_a.Data);
s_b = squeeze(out_b.s_b.Data);

% Core Kinematics & Lateral Load
vx_a = squeeze(out_a.vx_curr_a.Data); vx_b = squeeze(out_b.vx_curr_b.Data); 
vy_a = squeeze(out_a.vy_curr_a.Data); vy_b = squeeze(out_b.vy_curr_b.Data);
r_a  = squeeze(out_a.r_curr_a.Data);  r_b  = squeeze(out_b.r_curr_b.Data); % Yaw Rate
X_a = squeeze(out_a.X_curr_a.Data); Y_a = squeeze(out_a.Y_curr_a.Data);
X_b = squeeze(out_b.X_curr_b.Data); Y_b = squeeze(out_b.Y_curr_b.Data);

% Calculate Lateral Acceleration (G)
Ay_a_G = abs(vx_a .* r_a) / g;
Ay_b_G = abs(vx_b .* r_b) / g;

% Sideslip Angle Calculation (deg)
beta_a = atan2(vy_a, max(vx_a, 0.1)) * (180/pi); 
beta_b = atan2(vy_b, max(vx_b, 0.1)) * (180/pi);

% =========================================================================
% --- DISTANCE NORMALIZATION & DYNAMIC WINDOWING ---
% =========================================================================
% 1. Find the exact distance where the shorter run ended for fair comparison
shared_dist = min(dist_a, dist_b);

% 2. Create logical masks for the shared distance
idx_dist_a = s_a <= shared_dist;
idx_dist_b = s_b <= shared_dist;

% 3. Filter Parameters
v_thresh_mps = 45 / 3.6; % Speed > 45 km/h
ay_thresh    = 0.1;      % Lat-accel > 0.1 G

% 4. Combine Filters (Speed + Lat-G + Shared Distance)
idx_valid_a = (vx_a > v_thresh_mps) & (Ay_a_G > ay_thresh) & idx_dist_a;
idx_valid_b = (vx_b > v_thresh_mps) & (Ay_b_G > ay_thresh) & idx_dist_b;

% Average Speed Mask (Speed + Shared Distance only)
idx_v_a = (vx_a > v_thresh_mps) & idx_dist_a;
idx_v_b = (vx_b > v_thresh_mps) & idx_dist_b;

% =========================================================================
% --- STATISTICAL PERFORMANCE CALCULATORS ---
% =========================================================================
% Mask beta to prevent trapz from integrating straight-line time gaps
beta_masked_a = abs(beta_a) .* idx_valid_a;
beta_masked_b = abs(beta_b) .* idx_valid_b;

% 1. Cornering IAS (Integral of Absolute Sideslip)
IAS_a = trapz(t_a, beta_masked_a);
IAS_b = trapz(t_b, beta_masked_b);

% 2. Cornering RMS Sideslip (beta_RMS)
beta_RMS_a = rms(beta_a(idx_valid_a));
beta_RMS_b = rms(beta_b(idx_valid_b));

% 3. Peak Lateral Departure (Peak vy)
peak_vy_a = max(abs(vy_a(idx_valid_a)));
peak_vy_b = max(abs(vy_b(idx_valid_b)));

% =========================================================================
% ELECTRICAL, THERMAL & ENERGY LOGGING
% =========================================================================
% Vehicle A (Dual Motor)
P_elec_a = squeeze(out_a.P_elec_a.Data) / 1000; % kW
I_batt_a = squeeze(out_a.I_batt_a.Data);
T_batt_a = squeeze(out_a.T_batt_a.Data); T_mot_a = squeeze(out_a.T_motor_a.Data);
SOC_a    = squeeze(out_a.SOC_a.Data) * 100;
E_cons_a  = cumtrapz(t_a, max(0, P_elec_a)) / 3600;    
E_regen_a = cumtrapz(t_a, abs(min(0, P_elec_a))) / 3600;

% Vehicle B (Single Motor)
P_elec_b = squeeze(out_b.P_elec_b.Data) / 1000; % kW
I_batt_b = squeeze(out_b.I_batt_b.Data);
T_batt_b = squeeze(out_b.T_batt_b.Data); T_mot_b = squeeze(out_b.T_motor_b.Data);
SOC_b    = squeeze(out_b.SOC_b.Data) * 100;
E_cons_b  = cumtrapz(t_b, max(0, P_elec_b)) / 3600;    
E_regen_b = cumtrapz(t_b, abs(min(0, P_elec_b))) / 3600;

% Friction Waste Calculation (Vehicle B)
T_fric_b = abs(squeeze(out_b.Trr_fric_b.Data)) + abs(squeeze(out_b.Trl_fric_b.Data));
w_rear_b = (squeeze(out_b.w_rr_b.Data) + squeeze(out_b.w_rl_b.Data)) / 2;
P_fric_b = (T_fric_b .* w_rear_b) / 1000; 
E_fric_waste_b = cumtrapz(t_b, P_fric_b) / 3600;

%% 3. COMMAND WINDOW MISSION SUMMARY
fprintf('\n====================================================\n');
fprintf('   FINAL MISSION SUMMARY: MONZA LAP 1\n');
fprintf('   (Normalized to Shared Distance: %.2f m)\n', shared_dist);
fprintf('   (Filtering: Speed > 45 km/h | Lateral Load > 0.1G)\n');
fprintf('====================================================\n');
fprintf('METRIC                  | CAR A (Dual-Motor TV) | CAR B (Single-Motor BV)\n');
fprintf('----------------------------------------------------\n');
fprintf('Distance Covered (m)    | %12.2f | %12.2f\n', dist_a, dist_b);
fprintf('Avg. Speed (Active)     | %12.2f | %12.2f\n', mean(vx_a(idx_v_a))*3.6, mean(vx_b(idx_v_b))*3.6);
fprintf('Total Motoring (kWh)    | %12.4f | %12.4f\n', E_cons_a(end), E_cons_b(end));
fprintf('Total Regen (kWh)       | %12.4f | %12.4f\n', E_regen_a(end), E_regen_b(end));
fprintf('Friction Waste (kWh)    | %12.4f | %12.4f\n', 0.0000, E_fric_waste_b(end));
fprintf('Max Battery Temp (degC) | %12.1f | %12.1f\n', max(T_batt_a), max(T_batt_b));
fprintf('Max Motor Temp (degC)   | %12.1f | %12.1f\n', max(T_mot_a), max(T_mot_b));
fprintf('Minimum SOC (%%)         | %12.2f | %12.2f\n', min(SOC_a), min(SOC_b));
fprintf('Cornering IAS (deg-s)   | %12.2f | %12.2f\n', IAS_a, IAS_b);
fprintf('Cornering RMS beta (deg)| %12.3f | %12.3f\n', beta_RMS_a, beta_RMS_b);
fprintf('Peak Lat. Vel (m/s)     | %12.3f | %12.3f\n', peak_vy_a, peak_vy_b);
fprintf('====================================================\n');

%% 4. RENDERING DASHBOARDS
% FIG 1: Track Trajectory
figure('Name', 'Fig 1: Monza Trajectory', 'Color', 'w', 'Position', [50, 50, 800, 600]);
plot(X_track, Y_track, 'k--', 'LineWidth', 2); hold on; grid on;
plot(X_a, Y_a, 'b-', 'LineWidth', 1.5);
plot(X_b, Y_b, 'r-', 'LineWidth', 1.5);
title('Track Path Tracing'); legend('Ref', 'Veh A', 'Veh B'); axis equal;

% FIG 2: System Health Dashboard (Superimposed)
figure('Name', 'Fig 2: System Parameters', 'Color', 'w', 'Position', [100, 100, 1200, 800]);
subplot(4,1,1); plot(t_a, I_batt_a, 'b'); hold on; plot(t_b, I_batt_b, 'r'); ylabel('Current (A)'); title('Battery Current'); grid on;
subplot(4,1,2); plot(t_a, SOC_a, 'b', 'LineWidth', 2.5); hold on; plot(t_b, SOC_b, 'r', 'LineWidth', 2.5); ylabel('SOC (%)'); title('Battery SOC'); grid on;
subplot(4,1,3); plot(t_a, T_batt_a, 'b', 'LineWidth', 2.5); hold on; plot(t_b, T_batt_b, 'r', 'LineWidth', 2.5); ylabel('T\_batt (\circC)'); title('Battery Temperature'); grid on;
subplot(4,1,4); plot(t_a, T_mot_a, 'b', 'LineWidth', 2.5); hold on; plot(t_b, T_mot_b, 'r', 'LineWidth', 2.5); ylabel('T\_mot (\circC)'); xlabel('Time (s)'); title('Motor Temperature'); grid on;
legend('Car A (Dual-Motor TV)', 'Car B (Single-Motor BV)', 'Orientation', 'horizontal', 'Location', 'southoutside');

% FIG 3: Lateral Kinematics
figure('Name', 'Fig 3: Lateral Instability Analysis', 'Color', 'w', 'Position', [150, 150, 1000, 600]);
subplot(2,1,1); plot(t_a, vy_a, 'b'); hold on; plot(t_b, vy_b, 'r'); ylabel('v_y (m/s)'); title('Lateral Velocity (v_y)'); grid on;
subplot(2,1,2); plot(t_a, beta_a, 'b'); hold on; plot(t_b, beta_b, 'r'); ylabel('\beta (deg)'); xlabel('Time (s)'); title('Vehicle Sideslip Angle'); grid on;

% FIG 4: Cumulative Energy Profiling
figure('Name', 'Fig 4: Energy Consumption', 'Color', 'w', 'Position', [200, 200, 1000, 400]);
plot(t_a, E_cons_a, 'b-', 'LineWidth', 2.5); hold on; plot(t_a, E_regen_a, 'b--','LineWidth', 2.5);
plot(t_b, E_cons_b, 'r-', 'LineWidth', 2.5); plot(t_b, E_regen_b, 'r--', 'LineWidth', 2.5); plot(t_b, E_fric_waste_b, 'k:', 'LineWidth', 2.5);
title('Energy Consumption (kWh)'); ylabel('Energy'); xlabel('Time (s)'); grid on;
legend('A Consumed', 'A Recovered', 'B Consumed', 'B Recovered', 'B Fric Waste');

%% 5. EXPORT
disp('-> Exporting Dashboards...');
if ispc; desk_path = fullfile(getenv('USERPROFILE'), 'Desktop'); else; desk_path = fullfile(getenv('HOME'), 'Desktop'); end
out_dir = fullfile(desk_path, 'Lat_Dyn', 'TRK'); if ~exist(out_dir, 'dir'); mkdir(out_dir); end
figs = findall(0, 'Type', 'figure');
for i = 1:length(figs); saveas(figs(i), fullfile(out_dir, [regexprep(figs(i).Name, '[^a-zA-Z0-9]', '_'), '.jpg']), 'jpeg'); end
disp('   [Execution Complete]');