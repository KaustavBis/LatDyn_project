% =========================================================================
% MASTER SIMULATION & TELEMETRY SCRIPT: STEERING RELEASE (RLS)
% Models: RLS_1a (Dual Motor / TV) vs RLS_1b (Single Motor / Brake Vec)
% Target: Damping Ratio (\zeta) via Transient Overshoot
% =========================================================================
clear; clc; close all;

%% 1. INITIALIZATION & SIMULATION
disp('====================================================');
disp('   STEERING RELEASE (RLS) - DAMPING RATIO           ');
disp('====================================================');

% init_rls; 
disp('-> Running Dual Motor TV Simulation (RLS_1a)...');
out_a = sim('RLS_1a', 'ReturnWorkspaceOutputs', 'on');
save('temp_out_a.mat', 'out_a'); 

disp('-> Running Single Motor Simulation (RLS_1b)...');
out_b = sim('RLS_1b', 'ReturnWorkspaceOutputs', 'on');
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

Trr_m_a = squeeze(out_a.Trr_m_a.Data); 
Trl_m_a = squeeze(out_a.Trl_m_a.Data);

%% 3. ISOLATE RING-DOWN & CALCULATE DAMPING RATIO
disp('-> Analyzing Transient Ring-Down & Overshoot...');

% 1. Get the Steady-State Yaw Rate just before release (t = 14.5 to 14.9s)
r_ss_a = mean(r_a_raw((t_a >= 14.5) & (t_a <= 14.9)));
r_ss_b = mean(r_b_raw((t_b >= 14.5) & (t_b <= 14.9)));

% 2. Isolate the decay region AFTER the steering wheel hits 0 (t > 15.1s)
idx_decay_a = (t_a >= 15.1);
idx_decay_b = (t_b >= 15.1);
r_decay_a = r_a_raw(idx_decay_a);
r_decay_b = r_b_raw(idx_decay_b);

% 3. Find the maximum overshoot (crossing the zero line)
% Since the initial turn was positive, overshoot is the most negative value
overshoot_val_a = min(r_decay_a);
overshoot_val_b = min(r_decay_b);

% 4. Base-MATLAB Overshoot Damping Math
% If the value didn't cross zero (or barely crossed), it is critically/over damped.
calc_damping = @(os_val, r_ss) ...
    (os_val >= -0.1) * 1.0 + ... % Cap at 1.0 if there is effectively no overshoot
    (os_val < -0.1) * (-log(abs(os_val)/r_ss) / sqrt(pi^2 + (log(abs(os_val)/r_ss))^2));

damp_a = calc_damping(overshoot_val_a, r_ss_a);
damp_b = calc_damping(overshoot_val_b, r_ss_b);

% --- Explicitly Output to Workspace & Terminal ---
assignin('base', 'RLS_Damping_DualMotor', damp_a);
assignin('base', 'RLS_Damping_SingleMotor', damp_b);

disp('====================================================');
disp('   EXTRACTED DAMPING RATIOS (\zeta)');
disp('====================================================');
fprintf('-> Dual Motor TV:   %.3f\n', damp_a);
fprintf('-> Single Motor:    %.3f\n', damp_b);

%% 4. PLOTTING
disp('====================================================');
disp('-> Rendering Dashboards...');

% =========================================================================
% FIG 1: FULL TIME DOMAIN VERIFICATION
% =========================================================================
figure('Name', 'Fig 1: Steering Release Time Domain', 'Color', 'w', 'Position', [50, 50, 1000, 800]);

subplot(2,1,1);
plot(t_a, vx_a * 3.6, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, vx_b * 3.6, 'r-', 'LineWidth', 1.5);
title('Longitudinal Velocity (10s Settling Phase Verification)'); ylabel('Speed (km/h)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best'); 
xlim([0 t_a(end)]); % Fit to full simulation time

subplot(2,1,2);
yyaxis left; 
plot(t_a, delta_a_raw, 'k-', 'LineWidth', 1.5); 
ylabel('Steering Input (deg)');
ylim([-0.5 2.0]);

yyaxis right; 
plot(t_a, r_ref_deg, 'k--', 'LineWidth', 1.5); hold on;
plot(t_a, r_a_raw, 'b-', 'LineWidth', 1.5); plot(t_b, r_b_raw, 'r-', 'LineWidth', 1.5);
ylabel('Yaw Rate (deg/s)'); title('Transient Snap-Release Recovery');
legend('Steering Input', 'Demand (Ref)', 'Dual Motor TV', 'Single Motor', 'Location', 'northwest');
xlim([0 t_a(end)]); grid on; % Fit to full simulation time

% =========================================================================
% FIG 2: RING-DOWN OVERSHOOT EXTRACTION
% =========================================================================
figure('Name', 'Fig 2: Damping Ratio Extraction', 'Color', 'w', 'Position', [100, 100, 900, 600]);

% Plot the active decay region only
plot(t_a(idx_decay_a), r_decay_a, 'b-', 'LineWidth', 2); hold on; grid on;
plot(t_b(idx_decay_b), r_decay_b, 'r-', 'LineWidth', 2);
yline(0, 'k-', 'LineWidth', 1);

% Annotate Overshoots
plot(t_a(r_a_raw == overshoot_val_a), overshoot_val_a, 'bv', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
plot(t_b(r_b_raw == overshoot_val_b), overshoot_val_b, 'rv', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

title(sprintf('Ring-Down Decay Analysis\nTV Damping: %.3f | Single Motor Damping: %.3f', damp_a, damp_b));
xlabel('Time (s)'); ylabel('Yaw Rate (deg/s)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'southeast');
xlim([24 t_a(end)]); % Shows exactly from release point to end of simulation

% =========================================================================
% FIG 3: TV CONTROL INTENT (STABILIZATION PHASE)
% =========================================================================
Delta_T_a = Trr_m_a - Trl_m_a; 
figure('Name', 'Fig 3: Torque Vectoring Intent Analysis', 'Color', 'w', 'Position', [150, 150, 1000, 600]);

subplot(2,1,1);
plot(t_a, Trr_m_a, 'b-', 'LineWidth', 1); hold on; grid on;
plot(t_a, Trl_m_a, 'g-', 'LineWidth', 1);
ylabel('Motor Torque (Nm)'); title('Actuator Response to Snap Release');
legend('Rear Right', 'Rear Left', 'Location', 'northwest'); 
xlim([0 t_a(end)]); % Fit to full simulation time

subplot(2,1,2);
yyaxis left; plot(t_a, delta_a_raw, 'k--', 'LineWidth', 1.5); ylabel('Steering Input (deg)'); ylim([-0.5 2.0]);
yyaxis right; plot(t_a, Delta_T_a, 'm-', 'LineWidth', 1.5); ylabel('\Delta Torque [Nm]');
title('TV Intent: Pure Damping Injection (Anti-Spin)');
legend('Steering Input', '\Delta Torque (Applied Yaw Moment)', 'Location', 'northwest');
xlim([0 t_a(end)]); grid on; xlabel('Time (s)'); % Fit to full simulation time

%% 5. MIMURO PLOT DATA EXTRACTION (INJECT INTO MAT FILE)
disp('====================================================');
disp('-> Extracting Mimuro Damping Parameters...');

mat_filename = 'Mimuro_Data.mat';
if isfile(mat_filename); load(mat_filename, 'Mimuro'); else; Mimuro = struct(); end

% Injecting into the 'STR' namespace to complete the Rhombus dataset!
Mimuro.Dual_Motor.STR.Damping_Ratio = damp_a;
Mimuro.Single_Motor.STR.Damping_Ratio = damp_b;

save(mat_filename, 'Mimuro');
disp(['-> Damping parameters saved successfully to ', mat_filename]);
disp('====================================================');
disp('-> Steering Release (RLS) Execution Complete.');

%% 5. MIMURO PLOT DATA EXTRACTION (INJECT INTO MAT FILE)
disp('====================================================');
disp('-> Extracting Mimuro Damping Parameters...');

mat_filename = 'Mimuro_Data.mat';
if isfile(mat_filename); load(mat_filename, 'Mimuro'); else; Mimuro = struct(); end

% Injecting into the 'STR' namespace to complete the Rhombus dataset!
Mimuro.Dual_Motor.STR.Damping_Ratio = damp_a;
Mimuro.Single_Motor.STR.Damping_Ratio = damp_b;

save(mat_filename, 'Mimuro');
disp(['-> Damping parameters saved successfully to ', mat_filename]);
disp('====================================================');
disp('-> Steering Release (RLS) Execution Complete.');