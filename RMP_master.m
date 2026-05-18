% =========================================================================
% MASTER SIMULATION & TELEMETRY SCRIPT: SLOW RAMP STEER (RMP)
% Models: RMP_1a (Dual Motor / TV) vs RMP_1b (Single Motor / Brake Vec)
% Target: Steady-State Gain (Agility) Extraction via Linear Regression
% =========================================================================
clear; clc; close all;

%% 1. INITIALIZATION & SIMULATION
disp('====================================================');
disp('   SLOW RAMP STEER (RMP) - STEADY STATE GAIN        ');
disp('====================================================');

% init_rmp; 
disp('-> Running Dual Motor TV Simulation (RMP_1a)...');
out_a = sim('RMP_1a', 'ReturnWorkspaceOutputs', 'on');
save('temp_out_a.mat', 'out_a'); 

disp('-> Running Single Motor Simulation (RMP_1b)...');
out_b = sim('RMP_1b', 'ReturnWorkspaceOutputs', 'on');
load('temp_out_a.mat'); delete('temp_out_a.mat'); 

%% 2. DATA EXTRACTION: TIME DOMAIN
disp('-> Extracting Telemetry Data...');
t_a = out_a.tout; t_b = out_b.tout;

delta_a_raw = squeeze(out_a.delta_a.Data) * (180/pi);
delta_b_raw = squeeze(out_b.delta_b.Data) * (180/pi);
r_a_raw = squeeze(out_a.r_curr_a.Data) * (180/pi);
r_b_raw = squeeze(out_b.r_curr_b.Data) * (180/pi);
r_ref_deg = squeeze(out_a.r_ref_a.Data) * (180/pi); 

vx_a = squeeze(out_a.vx_curr_a.Data); 
vx_b = squeeze(out_b.vx_curr_b.Data);
Ay_a_raw = (vx_a .* squeeze(out_a.r_curr_a.Data)) / veh_params.g;
Ay_b_raw = (vx_b .* squeeze(out_b.r_curr_b.Data)) / veh_params.g;

Trr_m_a = squeeze(out_a.Trr_m_a.Data); 
Trl_m_a = squeeze(out_a.Trl_m_a.Data);

%% 3. ISOLATE LINEAR REGION & CALCULATE GAIN
disp('-> Analyzing Quasi-Steady-State Gradient...');

% Isolate the linear tire region (e.g., 15.5s to 18.0s)
% This guarantees we measure the chassis before the tires hit friction saturation (limit understeer).
idx_lin_a = (t_a >= 15.5) & (t_a <= 18.0);
idx_lin_b = (t_b >= 15.5) & (t_b <= 18.0);

delta_lin_a = delta_a_raw(idx_lin_a); r_lin_a = r_a_raw(idx_lin_a);
delta_lin_b = delta_b_raw(idx_lin_b); r_lin_b = r_b_raw(idx_lin_b);

% Use 1st-degree polynomial fit (linear regression) to find the slope (Steady State Gain)
p_a = polyfit(delta_lin_a, r_lin_a, 1); gain_a = p_a(1);
p_b = polyfit(delta_lin_b, r_lin_b, 1); gain_b = p_b(1);

% Generate ideal fit lines for plotting
fit_yaw_a = polyval(p_a, delta_a_raw(t_a >= 15.0));
fit_yaw_b = polyval(p_b, delta_b_raw(t_b >= 15.0));

% --- Explicitly Output to Workspace & Terminal ---
assignin('base', 'RMP_SS_Gain_DualMotor', gain_a);
assignin('base', 'RMP_SS_Gain_SingleMotor', gain_b);

disp('====================================================');
disp('   EXTRACTED STEADY-STATE GAINS');
disp('====================================================');
fprintf('-> Dual Motor TV:   %.3f (1/s)\n', gain_a);
fprintf('-> Single Motor:    %.3f (1/s)\n', gain_b);

%% 4. PLOTTING
disp('====================================================');
disp('-> Rendering Dashboards...');

% =========================================================================
% FIG 1: FULL TIME DOMAIN VERIFICATION
% =========================================================================
figure('Name', 'Fig 1: Ramp Steer Time Domain', 'Color', 'w', 'Position', [50, 50, 1000, 800]);

subplot(3,1,1);
plot(t_a, vx_a * 3.6, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, vx_b * 3.6, 'r-', 'LineWidth', 1.5);
title('Longitudinal Velocity (Settling Phase Verification)'); ylabel('Speed (km/h)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best'); xlim([0 t_a(end)]);

subplot(3,1,2);
yyaxis left; plot(t_a, delta_a_raw, 'k-', 'LineWidth', 1.5); ylabel('Steering Input (deg)');
yyaxis right; plot(t_a, r_ref_deg, 'k--', 'LineWidth', 1.5); hold on;
plot(t_a, r_a_raw, 'b-', 'LineWidth', 1.5); plot(t_b, r_b_raw, 'r-', 'LineWidth', 1.5);
ylabel('Yaw Rate (deg/s)'); title('Quasi-Steady-State Ramp Response');
legend('Steering Input', 'Demand (Ref)', 'Dual Motor TV', 'Single Motor', 'Location', 'northwest');
xlim([14 t_a(end)]); grid on;

subplot(3,1,3);
plot(t_a, Ay_a_raw, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, Ay_b_raw, 'r-', 'LineWidth', 1.5);
title('Lateral Acceleration (Tire Saturation Check)'); xlabel('Time (s)'); ylabel('Lat Accel (G)');
yline(0.4, 'k--', 'Linear Region Limit', 'LabelHorizontalAlignment', 'left');
xlim([14 t_a(end)]); 

% =========================================================================
% FIG 2: YAW RATE VS STEERING ANGLE (THE GAIN EXTRACTION PLOT)
% =========================================================================
figure('Name', 'Fig 2: Steady-State Gain Extraction', 'Color', 'w', 'Position', [100, 100, 900, 600]);

% Plot the active ramp region only
idx_plot = (t_a >= 15.0);
delta_plot = delta_a_raw(idx_plot);

plot(delta_plot, r_a_raw(idx_plot), 'b-', 'LineWidth', 2); hold on; grid on;
plot(delta_plot, r_b_raw(idx_plot), 'r-', 'LineWidth', 2);

% Overlay the linear regression fit lines
plot(delta_plot, fit_yaw_a, 'b--', 'LineWidth', 1.5);
plot(delta_plot, fit_yaw_b, 'r--', 'LineWidth', 1.5);

title('Steady-State Yaw Rate vs Steering Angle (Gradient = Gain)');
xlabel('Steering Angle (deg)'); ylabel('Yaw Rate (deg/s)');
legend('Dual Motor TV (Raw)', 'Single Motor (Raw)', ...
       sprintf('Dual Fit (Gain: %.3f)', gain_a), ...
       sprintf('Single Fit (Gain: %.3f)', gain_b), 'Location', 'northwest');
xlim([0 max(delta_plot)]); ylim([0 max(fit_yaw_a)*1.2]);

% =========================================================================
% FIG 3: TV CONTROL INTENT (STEADY-STATE TRACKING)
% =========================================================================
Delta_T_a = Trr_m_a - Trl_m_a; 
figure('Name', 'Fig 3: Torque Vectoring Intent Analysis', 'Color', 'w', 'Position', [150, 150, 1000, 600]);

subplot(2,1,1);
plot(t_a, Trr_m_a, 'b-', 'LineWidth', 1); hold on; grid on;
plot(t_a, Trl_m_a, 'g-', 'LineWidth', 1);
ylabel('Motor Torque (Nm)'); title('Steady-State Torque Allocation (No Transient Kicks)');
legend('Rear Right', 'Rear Left', 'Location', 'northwest'); xlim([14 t_a(end)]);

subplot(2,1,2);
yyaxis left; plot(t_a, delta_a_raw, 'k--', 'LineWidth', 1.5); ylabel('Steering Input (deg)');
yyaxis right; plot(t_a, Delta_T_a, 'm-', 'LineWidth', 1.5); ylabel('\Delta Torque [Nm]');
title('TV Intent: Continuous Tracking Error Minimization');
legend('Steering Input', '\Delta Torque (Applied Yaw Moment)', 'Location', 'northwest');
xlim([14 t_a(end)]); grid on; xlabel('Time (s)');

%% 5. MIMURO PLOT DATA EXTRACTION (INJECT INTO MAT FILE)
disp('====================================================');
disp('-> Extracting Mimuro Steady-State Gain Parameters...');

mat_filename = 'Mimuro_Data.mat';
if isfile(mat_filename); load(mat_filename, 'Mimuro'); else; Mimuro = struct(); end

% Injecting into the 'STR' namespace to maintain compatibility with existing plotters
Mimuro.Dual_Motor.STR.Gain = gain_a;
Mimuro.Single_Motor.STR.Gain = gain_b;

save(mat_filename, 'Mimuro');
disp(['-> Gain parameters saved successfully to ', mat_filename]);
disp('====================================================');
disp('-> Ramp Steer (RMP) Execution Complete.');

%% 5. MIMURO PLOT DATA EXTRACTION (INJECT INTO MAT FILE)
disp('====================================================');
disp('-> Extracting Mimuro Steady-State Gain Parameters...');

mat_filename = 'Mimuro_Data.mat';
if isfile(mat_filename); load(mat_filename, 'Mimuro'); else; Mimuro = struct(); end

% Injecting into the 'STR' namespace to maintain compatibility with existing plotters
Mimuro.Dual_Motor.STR.Gain = gain_a;
Mimuro.Single_Motor.STR.Gain = gain_b;

save(mat_filename, 'Mimuro');
disp(['-> Gain parameters saved successfully to ', mat_filename]);
disp('====================================================');
disp('-> Ramp Steer (RMP) Execution Complete.');