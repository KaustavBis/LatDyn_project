% =========================================================================
% MASTER SIMULATION & TELEMETRY SCRIPT: ONE HERTZ DWELL (OHZ)
% Models: OHZ_1a (Dual Motor / TV) vs OHZ_1b (Single Motor / Brake Vec)
% Target: Pure Time-Domain Phase Lag Extraction (Base-MATLAB)
% =========================================================================
clear; clc; close all;

%% 1. INITIALIZATION & SIMULATION
disp('====================================================');
disp('   ONE HERTZ DWELL (OHZ) - PHASE LAG EXTRACTION     ');
disp('====================================================');

% init_ohz; 
disp('-> Running Dual Motor TV Simulation (OHZ_1a)...');
out_a = sim('OHZ_1a', 'ReturnWorkspaceOutputs', 'on');
save('temp_out_a.mat', 'out_a'); 

disp('-> Running Single Motor Simulation (OHZ_1b)...');
out_b = sim('OHZ_1b', 'ReturnWorkspaceOutputs', 'on');
load('temp_out_a.mat'); delete('temp_out_a.mat'); 

%% 2. DATA EXTRACTION: TIME DOMAIN
disp('-> Extracting Telemetry Data...');
t_a = out_a.tout; t_b = out_b.tout;
dt = mean(diff(t_a));  

delta_a_raw = squeeze(out_a.delta_a.Data) * (180/pi);
delta_b_raw = squeeze(out_b.delta_b.Data) * (180/pi);
r_a_raw = squeeze(out_a.r_curr_a.Data) * (180/pi);
r_b_raw = squeeze(out_b.r_curr_b.Data) * (180/pi);
vx_a = squeeze(out_a.vx_curr_a.Data); vx_b = squeeze(out_b.vx_curr_b.Data);

Trr_m_a = squeeze(out_a.Trr_m_a.Data); Trl_m_a = squeeze(out_a.Trl_m_a.Data);
Trr_m_b = squeeze(out_b.Trr_m_b.Data); Trl_m_b = squeeze(out_b.Trl_m_b.Data);
T_total_b = Trr_m_b + Trl_m_b;

%% 3. TIME-DOMAIN PHASE LAG CALCULATION (CUSTOM PEAK DETECTION)
disp('-> Calculating Phase Lag via Direct Peak Time-Shifts...');

% Isolate final 5 seconds to bypass startup transients
idx_active = (t_a >= 20.0) & (t_a <= 25.0);
t_active = t_a(idx_active);

% Normalize and smooth slightly to prevent micro-noise from triggering false peaks
d_a_norm = movmean(delta_a_raw(idx_active) - mean(delta_a_raw(idx_active)), 10);
yaw_a_norm = movmean(r_a_raw(idx_active) - mean(r_a_raw(idx_active)), 10);
yaw_b_norm = movmean(r_b_raw(idx_active) - mean(r_b_raw(idx_active)), 10);

% --- Custom Base-MATLAB Peak Finder Function ---
% Finds points where the slope changes from positive to negative
get_peaks = @(x) find((x(2:end-1) > x(1:end-2)) & (x(2:end-1) > x(3:end))) + 1;

raw_locs_d = get_peaks(d_a_norm);
raw_locs_ya = get_peaks(yaw_a_norm);
raw_locs_yb = get_peaks(yaw_b_norm);

% Filter peaks to ensure they are spaced out (approx 0.8s for 1Hz signal)
min_dist = floor(0.8 / dt);
filter_peaks = @(locs) locs([true; diff(locs) > min_dist]);

locs_d = filter_peaks(raw_locs_d);
locs_ya = filter_peaks(raw_locs_ya);
locs_yb = filter_peaks(raw_locs_yb);

% --- Calculate Delay for Dual Motor (Vehicle A) ---
delay_a_list = [];
for i = 1:length(locs_d)
    t_steer_peak = t_active(locs_d(i));
    future_yaw_peaks = t_active(locs_ya(t_active(locs_ya) > t_steer_peak));
    if ~isempty(future_yaw_peaks)
        delay_a_list(end+1) = future_yaw_peaks(1) - t_steer_peak;
    end
end

% --- Calculate Delay for Single Motor (Vehicle B) ---
delay_b_list = [];
for i = 1:length(locs_d)
    t_steer_peak = t_active(locs_d(i));
    future_yaw_peaks = t_active(locs_yb(t_active(locs_yb) > t_steer_peak));
    if ~isempty(future_yaw_peaks)
        delay_b_list(end+1) = future_yaw_peaks(1) - t_steer_peak;
    end
end

% Average delays and convert to Phase Lag (Time Delay * Freq * 360)
time_delay_a = mean(delay_a_list);
time_delay_b = mean(delay_b_list);

phase_lag_a_deg = time_delay_a * 1.0 * 360; 
phase_lag_b_deg = time_delay_b * 1.0 * 360; 

%% 4. PLOTTING & VISUALIZATION
disp('-> Rendering Dashboards...');

% =========================================================================
% FIG 1: FULL TIME DOMAIN VERIFICATION
% =========================================================================
figure('Name', 'Fig 1: 1Hz Dwell Time Domain', 'Color', 'w', 'Position', [50, 50, 1000, 800]);

subplot(2,1,1);
plot(t_a, vx_a * 3.6, 'b-', 'LineWidth', 1.5); hold on; grid on; plot(t_b, vx_b * 3.6, 'r-', 'LineWidth', 1.5);
title('Longitudinal Velocity (Settling Phase Verification)'); ylabel('Speed (km/h)');
legend('Dual Motor TV', 'Single Motor', 'Location', 'best'); xlim([0 t_a(end)]);

subplot(2,1,2);
yyaxis left; plot(t_a, delta_a_raw, 'k-', 'LineWidth', 1.5); ylabel('Steering Input (deg)');
yyaxis right; plot(t_a, r_a_raw, 'b-', 'LineWidth', 1.5); hold on; plot(t_b, r_b_raw, 'r-', 'LineWidth', 1.5);
ylabel('Yaw Rate (deg/s)'); title('1.0 Hz Discrete Sine Response');
xlim([0 t_a(end)]); grid on;

% =========================================================================
% FIG 2: ZOOMED PHASE LAG VISUALIZATION
% =========================================================================
figure('Name', 'Fig 2: Normalized Phase Lag Comparison', 'Color', 'w', 'Position', [100, 100, 900, 600]);

% Normalize amplitudes to 1.0 for visual phase comparison
d_plot = d_a_norm / max(abs(d_a_norm));
y_a_plot = yaw_a_norm / max(abs(yaw_a_norm));
y_b_plot = yaw_b_norm / max(abs(yaw_b_norm));

plot(t_active, d_plot, 'k--', 'LineWidth', 2); hold on; grid on;
plot(t_active, y_a_plot, 'b-', 'LineWidth', 1.5);
plot(t_active, y_b_plot, 'r-', 'LineWidth', 1.5);

title('Steady-State Phase Lag Tracking (Normalized Amplitudes)');
xlabel('Time (s)'); ylabel('Normalized Amplitude');
legend('Steering Input', sprintf('Dual Motor TV (Lag: %.1f\\circ)', phase_lag_a_deg), ...
                         sprintf('Single Motor (Lag: %.1f\\circ)', phase_lag_b_deg), 'Location', 'best');
xlim([20.0 23.0]); % Zoom in on exactly 3 cycles

% =========================================================================
% FIG 3: TV CONTROL INTENT
% =========================================================================
Delta_T_a = Trr_m_a - Trl_m_a; 
figure('Name', 'Fig 3: Torque Vectoring Intent Analysis', 'Color', 'w', 'Position', [150, 150, 1000, 600]);

subplot(2,1,1);
plot(t_a, Trr_m_a, 'b-', 'LineWidth', 1); hold on; grid on;
plot(t_a, Trl_m_a, 'g-', 'LineWidth', 1);
ylabel('Motor Torque (Nm)'); title('Individual Actuator Outputs (1 Hz Continuous)');
legend('Rear Right', 'Rear Left', 'Location', 'northeast'); xlim([15 25]);

subplot(2,1,2);
yyaxis left; plot(t_a, delta_a_raw, 'k--', 'LineWidth', 1.5); ylabel('Steering Input (deg)');
yyaxis right; plot(t_a, Delta_T_a, 'm-', 'LineWidth', 1.5); ylabel('\Delta Torque [Nm]');
title('TV Intent: Continuous Rhythmic Tracking');
legend('Steering Input', '\Delta Torque (Applied Yaw Moment)', 'Location', 'southwest');
xlim([15 25]); grid on; xlabel('Time (s)');

%% 5. MIMURO PLOT DATA EXTRACTION (INJECT INTO SWS STRUCT)
disp('====================================================');
disp('-> Extracting Mimuro Phase Lag Parameters...');

mat_filename = 'Mimuro_Data.mat';
if isfile(mat_filename); load(mat_filename, 'Mimuro'); else; Mimuro = struct(); end

Mimuro.Dual_Motor.SWS.Phase_1Hz = phase_lag_a_deg;
Mimuro.Single_Motor.SWS.Phase_1Hz = phase_lag_b_deg;

save(mat_filename, 'Mimuro');
disp(['-> Phase Lag parameters saved successfully to ', mat_filename]);
disp('====================================================');
disp('-> One Hertz Dwell (OHZ) Execution Complete.');