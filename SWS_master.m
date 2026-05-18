% =========================================================================
% MASTER SIMULATION & TELEMETRY SCRIPT: SWEPT SINE (SWS) BODE ANALYSIS
% Models: SWS_1a (Dual Motor / TV) vs SWS_1b (Single Motor / Brake Vec)
% Architecture: Raw FFT + Light Smoothing (Base-MATLAB)
% =========================================================================
clear; clc; close all;

%% 1. INITIALIZATION & SIMULATION
disp('====================================================');
disp('   ISO 7401 SWEPT SINE (SWS) - FREQUENCY RESPONSE   ');
disp('====================================================');

% Run setup script (Uncomment if needed)
% init_sws; 

disp('-> Running Dual Motor TV Simulation (SWS_1a)...');
out_a = sim('SWS_1a', 'ReturnWorkspaceOutputs', 'on');
save('temp_out_a.mat', 'out_a'); 

disp('-> Running Single Motor Simulation (SWS_1b)...');
out_b = sim('SWS_1b', 'ReturnWorkspaceOutputs', 'on');
load('temp_out_a.mat'); delete('temp_out_a.mat'); 

%% 2. DATA EXTRACTION: TIME DOMAIN
disp('-> Extracting Telemetry Data...');
t_a = out_a.tout; t_b = out_b.tout;
dt = mean(diff(t_a)); Fs = 1/dt; 

% Raw Inputs/Outputs (for Time Domain Plotting)
delta_a_raw = squeeze(out_a.delta_a.Data) * (180/pi);
delta_b_raw = squeeze(out_b.delta_b.Data) * (180/pi);
r_a_raw = squeeze(out_a.r_curr_a.Data) * (180/pi);
r_b_raw = squeeze(out_b.r_curr_b.Data) * (180/pi);
vx_a = squeeze(out_a.vx_curr_a.Data); vx_b = squeeze(out_b.vx_curr_b.Data);

Ay_a_raw = (vx_a .* squeeze(out_a.r_curr_a.Data)) / veh_params.g;
Ay_b_raw = (vx_b .* squeeze(out_b.r_curr_b.Data)) / veh_params.g;

Trr_m_a = squeeze(out_a.Trr_m_a.Data); Trl_m_a = squeeze(out_a.Trl_m_a.Data);
Trr_m_b = squeeze(out_b.Trr_m_b.Data); Trl_m_b = squeeze(out_b.Trl_m_b.Data);
T_total_b = Trr_m_b + Trl_m_b;

%% 3. ISOLATE ACTIVE CHIRP FOR FFT
% Strip off the 15-second straight-line settling phase so zeroes don't corrupt the math
disp('-> Isolating Active Maneuver Window for FFT...');
t_start = 15.0; t_chirp = 20.0;
idx_a = (t_a >= t_start) & (t_a <= t_start + t_chirp);
idx_b = (t_b >= t_start) & (t_b <= t_start + t_chirp);

delta_a_fft = delta_a_raw(idx_a); r_a_fft = r_a_raw(idx_a); Ay_a_fft = Ay_a_raw(idx_a);
delta_b_fft = delta_b_raw(idx_b); r_b_fft = r_b_raw(idx_b); Ay_b_fft = Ay_b_raw(idx_b);

%% 4. CORRECTED TRANSFER FUNCTION (RAW FFT + LIGHT POST-SMOOTHING)
disp('-> Calculating Transfer Functions...');

L_a = length(delta_a_fft); L_b = length(delta_b_fft);
f_a = Fs * (0:(L_a/2))/L_a; w_rad_a = 2 * pi * f_a;
f_b = Fs * (0:(L_b/2))/L_b; w_rad_b = 2 * pi * f_b;

% 1. Compute Base Fast Fourier Transforms (FFT)
X_a = fft(delta_a_fft); Y_yaw_a = fft(r_a_fft); Y_ay_a = fft(Ay_a_fft);
X_b = fft(delta_b_fft); Y_yaw_b = fft(r_b_fft); Y_ay_b = fft(Ay_b_fft);

% 2. Extract Single-Sided Spectra
X_a = X_a(1:floor(L_a/2)+1); 
Y_yaw_a = Y_yaw_a(1:floor(L_a/2)+1); Y_ay_a = Y_ay_a(1:floor(L_a/2)+1);
X_b = X_b(1:floor(L_b/2)+1); 
Y_yaw_b = Y_yaw_b(1:floor(L_b/2)+1); Y_ay_b = Y_ay_b(1:floor(L_b/2)+1);

% 3. Raw Transfer Function (H = Output / Input)
H_yaw_a = Y_yaw_a ./ X_a; H_ay_a = Y_ay_a ./ X_a;
H_yaw_b = Y_yaw_b ./ X_b; H_ay_b = Y_ay_b ./ X_b;

% 4. Raw Gain and Phase (Bounded to +/- 180 degrees)
Gain_raw_yaw_a = 20 * log10(abs(H_yaw_a)); Phase_raw_yaw_a = angle(H_yaw_a) * (180/pi);
Gain_raw_yaw_b = 20 * log10(abs(H_yaw_b)); Phase_raw_yaw_b = angle(H_yaw_b) * (180/pi);
Gain_raw_ay_a = 20 * log10(abs(H_ay_a)); Phase_raw_ay_a = angle(H_ay_a) * (180/pi);
Gain_raw_ay_b = 20 * log10(abs(H_ay_b)); Phase_raw_ay_b = angle(H_ay_b) * (180/pi);

% 5. Apply LIGHT smoothing (5 points = ~0.25 Hz window) 
% This removes jagged edges without destroying the vehicle dynamics peaks
light_win = 5; 
Gain_yaw_a = movmean(Gain_raw_yaw_a, light_win); Phase_yaw_a = movmean(Phase_raw_yaw_a, light_win);
Gain_yaw_b = movmean(Gain_raw_yaw_b, light_win); Phase_yaw_b = movmean(Phase_raw_yaw_b, light_win);
Gain_ay_a = movmean(Gain_raw_ay_a, light_win); Phase_ay_a = movmean(Phase_raw_ay_a, light_win);
Gain_ay_b = movmean(Gain_raw_ay_b, light_win); Phase_ay_b = movmean(Phase_raw_ay_b, light_win);

%% 5. PLOTTING
disp('-> Rendering Dashboards...');

% =========================================================================
% FIG 1: TIME DOMAIN VERIFICATION
% =========================================================================
figure('Name', 'Fig 1: Time Domain Responses', 'Color', 'w', 'Position', [50, 50, 1000, 800]);
subplot(3,1,1);
plot(t_a, vx_a * 3.6, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, vx_b * 3.6, 'r-', 'LineWidth', 1.5);
title('Longitudinal Velocity (Settling Phase Verification)'); ylabel('Speed (km/h)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best'); xlim([0 sim_params.t_end]);

subplot(3,1,2);
yyaxis left; plot(t_a, delta_a_raw, 'k-', 'LineWidth', 1); ylabel('Steering Input (deg)');
yyaxis right; plot(t_a, r_a_raw, 'b-', 'LineWidth', 1.5); hold on; plot(t_b, r_b_raw, 'r-', 'LineWidth', 1.5);
ylabel('Yaw Rate (deg/s)'); title('Swept Sine Time-Domain Response');
xlim([0 sim_params.t_end]); grid on;

subplot(3,1,3);
plot(t_a, Ay_a_raw, 'b-', 'LineWidth', 1.5); hold on; plot(t_b, Ay_b_raw, 'r-', 'LineWidth', 1.5);
title('Lateral Acceleration Demand'); xlabel('Time (s)'); ylabel('Lat Accel (G)');
xlim([0 sim_params.t_end]); grid on;

% =========================================================================
% FIG 2: BODE PLOT - YAW RATE
% =========================================================================
figure('Name', 'Fig 2: Yaw Rate Bode Plot', 'Color', 'w', 'Position', [100, 100, 900, 800]);
subplot(2,1,1);
semilogx(w_rad_a, Gain_yaw_a, 'b-', 'LineWidth', 2); hold on; grid on;
semilogx(w_rad_b, Gain_yaw_b, 'r-', 'LineWidth', 2);
xline(1, 'k--', 'w = 1 rad/s Ref', 'LabelOrientation', 'horizontal', 'LabelHorizontalAlignment', 'left');
title('Yaw Rate Frequency Response G(s) = r / \delta'); ylabel('Magnitude (dB)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'southwest'); xlim([0.5 20]);

subplot(2,1,2);
semilogx(w_rad_a, Phase_yaw_a, 'b-', 'LineWidth', 2); hold on; grid on;
semilogx(w_rad_b, Phase_yaw_b, 'r-', 'LineWidth', 2);
xline(1, 'k--', 'w = 1 rad/s Ref', 'LabelOrientation', 'horizontal', 'LabelHorizontalAlignment', 'left');
xlabel('Frequency \omega (rad/s)'); ylabel('Phase (degrees)'); xlim([0.5 20]);

% =========================================================================
% FIG 3: BODE PLOT - LATERAL ACCELERATION
% =========================================================================
figure('Name', 'Fig 3: Lateral Accel Bode Plot', 'Color', 'w', 'Position', [150, 150, 900, 800]);
subplot(2,1,1);
semilogx(w_rad_a, Gain_ay_a, 'b-', 'LineWidth', 2); hold on; grid on;
semilogx(w_rad_b, Gain_ay_b, 'r-', 'LineWidth', 2);
xline(1, 'k--', 'w = 1 rad/s Ref', 'LabelOrientation', 'horizontal', 'LabelHorizontalAlignment', 'left');
title('Lateral Acceleration Frequency Response G(s) = Ay / \delta'); ylabel('Magnitude (dB)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'southwest'); xlim([0.5 20]);

subplot(2,1,2);
semilogx(w_rad_a, Phase_ay_a, 'b-', 'LineWidth', 2); hold on; grid on;
semilogx(w_rad_b, Phase_ay_b, 'r-', 'LineWidth', 2);
xline(1, 'k--', 'w = 1 rad/s Ref', 'LabelOrientation', 'horizontal', 'LabelHorizontalAlignment', 'left');
xlabel('Frequency \omega (rad/s)'); ylabel('Phase (degrees)'); xlim([0.5 20]);

% =========================================================================
% FIG 4: TRANSIENT CONTROL EFFORT
% =========================================================================
figure('Name', 'Fig 4: Control Effort Allocation', 'Color', 'w', 'Position', [200, 200, 1000, 600]);
subplot(2,1,1);
plot(t_a, Trr_m_a, 'b-', 'LineWidth', 1); hold on; grid on; plot(t_a, Trl_m_a, 'g-', 'LineWidth', 1);
title('Dual Motor TV - Independent Torque Allocation'); ylabel('Motor Torque (Nm)');
legend('Rear Right', 'Rear Left', 'Location', 'best'); xlim([10 35]); 

subplot(2,1,2);
plot(t_b, T_total_b, 'r-', 'LineWidth', 1.5); hold on; grid on;
title('Single Motor - Total Axle Torque'); xlabel('Time (s)'); ylabel('Combined Torque (Nm)');
xlim([10 35]);

% =========================================================================
% FIG 5: TV CONTROL INTENT (AGILITY VS STABILITY)
% =========================================================================
Delta_T_a = Trr_m_a - Trl_m_a; 
figure('Name', 'Fig 5: Torque Vectoring Intent Analysis', 'Color', 'w', 'Position', [250, 250, 1000, 800]);

subplot(3,1,1);
yyaxis left; 
plot(t_a, delta_a_raw, 'k--', 'LineWidth', 1.5); 
ylabel('Steering Input (deg)');
yyaxis right; 
plot(t_a, r_a_raw, 'b-', 'LineWidth', 1.5); hold on;
plot(t_b, r_b_raw, 'r-', 'LineWidth', 1.5); 
ylabel('Actual Yaw Rate (deg/s)');
title('Driver Request vs Chassis Response'); 
legend('Steering Input', 'Dual Motor TV', 'Single Motor', 'Location', 'southwest');
xlim([15 30]); grid on;

subplot(3,1,2);
plot(t_a, Trr_m_a, 'b-', 'LineWidth', 1); hold on; grid on;
plot(t_a, Trl_m_a, 'g-', 'LineWidth', 1);
ylabel('Motor Torque (Nm)'); legend('Rear Right', 'Rear Left', 'Location', 'northeast');
title('Individual Actuator Outputs'); xlim([15 30]);

subplot(3,1,3);
yyaxis left; plot(t_a, delta_a_raw, 'k--', 'LineWidth', 1.5); ylabel('Steering Input (deg)');
yyaxis right; plot(t_a, Delta_T_a, 'm-', 'LineWidth', 1.5); ylabel('\Delta Torque (Trr - Trl) [Nm]');
title('TV Intent: In-Phase = Agility | Out-of-Phase = Stability');
legend('Steering Input', '\Delta Torque (Applied Yaw Moment)', 'Location', 'southwest');
xlim([15 30]); grid on; xlabel('Time (s)');

disp('====================================================');
disp('-> Swept Sine Suite Execution Complete.');