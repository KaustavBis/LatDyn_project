% =========================================================================
% MASTER SIMULATION & TELEMETRY SCRIPT: TRANSIENT SLALOM TESTING
% Models: SLA_1a (Dual Motor / TV) vs SLA_1b (Single Motor / Brake Vec)
% =========================================================================
clear; clc; close all;

%% 1. USER INPUT & INITIALIZATION
disp('====================================================');
disp('   TRANSIENT AGILITY (SLALOM) - A/B TEST SUITE      ');
disp('====================================================');

% Call initialization script (Uncomment if not run manually)
% init_slalom; 

%% 2. EXECUTE SIMULATIONS (With Workspace Wipe Protection)
disp('-> Running Dual Motor TV Simulation (SLA_1a)...');
out_a = sim('SLA_1a', 'ReturnWorkspaceOutputs', 'on');
save('temp_out_a.mat', 'out_a'); 
disp('   [SLA_1a Complete & Saved]');

disp('-> Running Single Motor Simulation (SLA_1b)...');
out_b = sim('SLA_1b', 'ReturnWorkspaceOutputs', 'on');
disp('   [SLA_1b Complete]');

load('temp_out_a.mat');
delete('temp_out_a.mat'); 

%% 3. DATA EXTRACTION: CHASSIS DYNAMICS & TRANSIENTS
disp('-> Extracting Chassis Telemetry Data...');

t_a = out_a.tout;
t_b = out_b.tout;

dt_a = mean(diff(t_a));
dt_b = mean(diff(t_b));

% ----------------- VEHICLE A (Dual Motor) -----------------
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

% Transient specific: Yaw Acceleration (rad/s^2)
r_dot_a = gradient(r_a_rad, dt_a);
Ay_a_G = (vx_a .* r_a_rad) / veh_params.g;

% ----------------- VEHICLE B (Single Motor) ---------------
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

% Friction Brake Data (Added)
Trr_fric_b = squeeze(out_b.Trr_fric_b.Data);
Trl_fric_b = squeeze(out_b.Trl_fric_b.Data);

beta_b_deg = atan2(vy_b, vx_b) * (180/pi);

% Transient specific: Yaw Acceleration (rad/s^2) & Lateral Gs (Restored)
r_dot_b = gradient(r_b_rad, dt_b);
Ay_b_G = (vx_b .* r_b_rad) / veh_params.g;

%% 4. DATA EXTRACTION: TIRE DYNAMICS
disp('-> Extracting Tire Dynamics...');

Fz_fl_a = squeeze(out_a.Fz_fl_a.Data); Fz_fr_a = squeeze(out_a.Fz_fr_a.Data);
Fz_rl_a = squeeze(out_a.Fz_rl_a.Data); Fz_rr_a = squeeze(out_a.Fz_rr_a.Data);
Fz_fl_b = squeeze(out_b.Fz_fl_b.Data); Fz_fr_b = squeeze(out_b.Fz_fr_b.Data);
Fz_rl_b = squeeze(out_b.Fz_rl_b.Data); Fz_rr_b = squeeze(out_b.Fz_rr_b.Data);

%% 5. PLOTTING AND VISUALIZATION
disp('-> Generating Transient Telemetry Dashboards...');

% =========================================================================
% FIG 1: OPEN-LOOP DYNAMIC RESPONSE (AGILITY & STABILITY)
% =========================================================================
figure('Name', 'Fig 1: Open-Loop Dynamic Metrics', 'Color', 'w', 'Position', [50, 50, 1200, 800]);

% 1. Steering Input vs Yaw Rate (Phase & Gain)
subplot(2,2,1);
yyaxis left;
plot(t_a, delta_a_deg, 'k--', 'LineWidth', 1.5); 
ylabel('Steering Input (deg)');
yyaxis right;
plot(t_a, r_a_deg, 'b-', 'LineWidth', 1.5); hold on;
plot(t_b, r_b_deg, 'r-', 'LineWidth', 1.5);
ylabel('Yaw Rate (deg/s)');
title('Yaw Response vs Steering Command');
legend('Steering (\delta)', 'Dual Motor (r_a)', 'Single Motor (r_b)', 'Location', 'best');
xlim([min(t_a) max(t_a)]); grid on;

% 2. Speed Bleed (Efficiency)
subplot(2,2,2);
plot(t_a, vx_a * 3.6, 'b-', 'LineWidth', 2); hold on; grid on;
plot(t_b, vx_b * 3.6, 'r-', 'LineWidth', 2);
title('Kinetic Energy Retention (Speed Bleed)');
xlabel('Time (s)'); ylabel('Velocity (km/h)');
legend('Dual Motor (No Brakes)', 'Single Motor (Brake Vec)', 'Location', 'best');

% 3. Sideslip Angle (Stability)
subplot(2,2,3);
plot(t_a, beta_a_deg, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, beta_b_deg, 'r-', 'LineWidth', 1.5);
title('Vehicle Sideslip Angle (\beta)'); 
xlabel('Time (s)'); ylabel('Sideslip (deg)');
legend('Dual Motor', 'Single Motor', 'Location', 'best');

% 4. Lateral Acceleration (G-Force)
subplot(2,2,4);
plot(t_a, Ay_a_G, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, Ay_b_G, 'r-', 'LineWidth', 1.5);
title('Lateral Acceleration Demand vs Fulfillment'); 
xlabel('Time (s)'); ylabel('Lateral Gs');
legend('Dual Motor', 'Single Motor', 'Location', 'best');

% =========================================================================
% FIG 2: AGILITY METRICS (Phase Lag Overlay)
% =========================================================================
% Normalize Steering and Yaw Rate to visualize the time delay (Phase Lag)
norm_steer_a = delta_a_deg / max(abs(delta_a_deg));
norm_yaw_a   = r_a_deg / max(abs(r_a_deg));

norm_steer_b = delta_b_deg / max(abs(delta_b_deg));
norm_yaw_b   = r_b_deg / max(abs(r_b_deg));

figure('Name', 'Fig 2: Agility & Phase Lag Analysis', 'Color', 'w', 'Position', [100, 100, 1000, 800]);

subplot(2,1,1);
plot(t_a, norm_steer_a, 'k--', 'LineWidth', 1); hold on; grid on;
plot(t_a, norm_yaw_a, 'b-', 'LineWidth', 1.5);
title('Car A (Dual Motor) - Steering vs Yaw Response Phase Lag');
ylabel('Normalized Amplitude');
legend('Steering Input', 'Yaw Response', 'Location', 'best'); xlim([0 max(t_a)]);

subplot(2,1,2);
plot(t_b, norm_steer_b, 'k--', 'LineWidth', 1); hold on; grid on;
plot(t_b, norm_yaw_b, 'r-', 'LineWidth', 1.5);
title('Car B (Single Motor) - Steering vs Yaw Response Phase Lag');
xlabel('Time (s)'); ylabel('Normalized Amplitude');
legend('Steering Input', 'Yaw Response', 'Location', 'best'); xlim([0 max(t_a)]);

% =========================================================================
% FIG 3: KINETIC ENERGY RETENTION (Velocity Bleed)
% =========================================================================
figure('Name', 'Fig 3: Longitudinal Velocity Bleed', 'Color', 'w', 'Position', [150, 150, 800, 400]);
plot(t_a, vx_a * 3.6, 'b-', 'LineWidth', 2); hold on; grid on;
plot(t_b, vx_b * 3.6, 'r-', 'LineWidth', 2);
title('Kinetic Energy Retention through Slalom (Brake Drag vs Motor Vectoring)');
xlabel('Time (s)'); ylabel('Velocity (km/h)');
legend('Dual Motor TV (No Friction Braking)', 'Single Motor (Brake Vectoring)', 'Location', 'best');

% =========================================================================
% FIG 4: TRANSIENT YAW ACCELERATION (\dot{r})
% =========================================================================
figure('Name', 'Fig 4: Transient Yaw Acceleration', 'Color', 'w', 'Position', [200, 200, 800, 400]);
plot(t_a, r_dot_a, 'b-', 'LineWidth', 1.5); hold on; grid on;
plot(t_b, r_dot_b, 'r-', 'LineWidth', 1.5);
title('Yaw Acceleration (\dot{r}) - "Snap" and Responsiveness');
xlabel('Time (s)'); ylabel('Yaw Accel (rad/s^2)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best');

% =========================================================================
% FIG 5: MOTOR TORQUE ALLOCATION
% =========================================================================
figure('Name', 'Fig 5: Motor Torque Allocation', 'Color', 'w', 'Position', [250, 250, 800, 600]);

subplot(2,1,1);
plot(t_a, Trr_m_a, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_a, Trl_m_a, 'g-', 'LineWidth', 1.5);
title('Vehicle A (Dual Motor) - High Frequency Transient Torque'); 
ylabel('Motor Torque (Nm)'); legend('Rear Right', 'Rear Left', 'Location', 'best');

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

title('Vehicle B (Single Motor) - Total Axle Torque & Brake Vectoring'); 
xlabel('Time (s)'); 
legend('Total Motor Torque', 'Rear Right Brake (T_{rr\_fric})', 'Rear Left Brake (T_{rl\_fric})', 'Location', 'best');
% =========================================================================
% FIG 6 & 7: ELECTRICAL TELEMETRY EXTRACTION & PLOTTING
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

figure('Name', 'Fig 6: Battery States', 'Color', 'w', 'Position', [300, 300, 800, 800]);
subplot(3,1,1);
plot(t_a, SOC_a, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, SOC_b, 'r-', 'LineWidth', 1.5);
title('Battery State of Charge (SOC)'); ylabel('SOC (%)'); legend('Dual Motor', 'Single Motor', 'Location', 'best');
subplot(3,1,2);
plot(t_a, I_batt_a, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, I_batt_b, 'r-', 'LineWidth', 1.5);
title('Transient Battery Current'); ylabel('Current (A)');
subplot(3,1,3);
plot(t_a, T_batt_a, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, T_batt_b, 'r-', 'LineWidth', 1.5);
title('Battery Temperature'); xlabel('Time (s)'); ylabel('Temperature (°C)');

figure('Name', 'Fig 7: Electrical Power Analysis', 'Color', 'w', 'Position', [350, 350, 800, 600]);
subplot(2,1,1);
plot(t_a, I_batt_a, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, I_batt_b, 'r-', 'LineWidth', 1.5);
title('Battery Current Draw'); ylabel('Current (A)'); legend('Dual Motor TV', 'Single Motor', 'Location', 'best');
subplot(2,1,2);
plot(t_a, Pwr_Elec_a / 1000, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, Pwr_Elec_b / 1000, 'r-', 'LineWidth', 1.5);
title('Gross Electrical Power Consumption'); xlabel('Time (s)'); ylabel('Power (kW)');

% =========================================================================
% FIG X: SLALOM TV CONTROL INTENT (PHASE ALIGNMENT)
% =========================================================================
% Assuming variables: t_a, delta_a_deg, r_a_deg, Trr_m_a, Trl_m_a 
Delta_T_a = Trr_m_a - Trl_m_a; 

figure('Name', 'SLA: Torque Vectoring Intent Analysis', 'Color', 'w', 'Position', [250, 250, 1000, 800]);

subplot(3,1,1);
yyaxis left; plot(t_a, delta_a_deg, 'k--', 'LineWidth', 1.5); ylabel('Steering Input (deg)');
yyaxis right; plot(t_a, r_a_deg, 'b-', 'LineWidth', 1.5); ylabel('Yaw Rate (deg/s)');
title('Slalom: Driver Request vs Chassis Response'); grid on;

subplot(3,1,2);
plot(t_a, Trr_m_a, 'b-', 'LineWidth', 1); hold on; grid on; plot(t_a, Trl_m_a, 'g-', 'LineWidth', 1);
ylabel('Motor Torque (Nm)'); legend('Rear Right', 'Rear Left', 'Location', 'best');
title('Slalom Actuator Pacing');

subplot(3,1,3);
yyaxis left; plot(t_a, delta_a_deg, 'k--', 'LineWidth', 1.5); ylabel('Steering Input (deg)');
yyaxis right; plot(t_a, Delta_T_a, 'm-', 'LineWidth', 1.5); ylabel('\Delta Torque (Trr - Trl) [Nm]');
title('TV Intent: Continuous Agility Enhancement (Phase Check)');
legend('Steering Input', '\Delta Torque (Applied Yaw Moment)', 'Location', 'best');
grid on; xlabel('Time (s)');
disp('-> All Slalom Dashboards Rendered Successfully.');