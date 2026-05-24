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


%% Mechanical Parameters Dashboard - Car A
% TURN-BY-TURN ANALYSIS for TRK_1a Monza Simulation
% Restructured Layout: Track Map, Speed Hist, Torque Hist on Left Column.
% Beta, Vy, Ay Time History plots on Right Column.
% Turns are highlighted in red on all right-column plots, and turn-exclusive
% RMS lines with BOLD labels are displayed. Legends removed for clarity.

% proj = 'C:\Users\kaust\OneDrive\Documents\GitHub\LatDyn_project';
% load(fullfile(proj, 'TRK_1a_results.mat'));   % loads out_a

% ── 1. Extract Signals ────────────────────────────────────────────────────
t  = out_a.tout;
vx = squeeze(out_a.vx_curr_a.Data);
vy = squeeze(out_a.vy_curr_a.Data);
r  = squeeze(out_a.r_curr_a.Data);
X  = squeeze(out_a.X_curr_a.Data);
Y  = squeeze(out_a.Y_curr_a.Data);
s  = squeeze(out_a.s_a.Data);

% UPDATE THESE NAMES TO MATCH YOUR SIMULINK TORQUE LOGGING
T_RL = squeeze(out_a.Trl_m_a.Data); % Rear Left Motor Torque (Nm)
T_RR = squeeze(out_a.Trr_m_a.Data); % Rear Right Motor Torque (Nm)

% NEW Signal: Raw Lateral Acceleration (vx * r) in m/s^2
% Multiplied by -1 to visually align the peaks with the sideslip/vy conventions
ay_raw = -(vx .* r);

g       = 9.81;
Ay_G    = abs(ay_raw) / g;                           % lateral-G (centripetal)
beta    = atan2(vy, max(vx, 0.1)) * (180/pi);        % sideslip [deg]

% ── 2. Turn Detection ─────────────────────────────────────────────────────
Ay_thresh       = 0.3;     % Magnitude filter
min_duration_s  = 0.4;     % Duration filter

in_turn = Ay_G > Ay_thresh;

edges       = diff([0; in_turn; 0]);
turn_starts = find(edges == 1);
turn_ends   = find(edges == -1) - 1;

durations   = t(turn_ends) - t(turn_starts);
valid       = durations >= min_duration_s;
turn_starts = turn_starts(valid);
turn_ends   = turn_ends(valid);
N_turns     = numel(turn_starts);

fprintf('===============================================================\n');
fprintf(' CAR A: TURN DETECTION (Ay > %.2f G, duration >= %.1f s)\n', Ay_thresh, min_duration_s);
fprintf(' Found %d turn instances across the simulation\n', N_turns);
fprintf('===============================================================\n\n');

% ── 3. Lap Length (Monza nominal) ────────────────────────────────────────
lap_length     = 10000;
n_laps_covered = s(end) / lap_length;
fprintf('Lap length (Monza nominal): %.0f m   |   Total laps covered: %.2f\n\n', ...
    lap_length, n_laps_covered);

% ── 4. Per-Turn Metrics Table ─────────────────────────────────────────────
hdr = sprintf('%4s  %6s  %5s  %8s  %7s %7s %7s  %6s  %8s  %8s  %5s  %7s  %7s', ...
    '#', 'Lap', 'TIdx', 'Time(s)', 'Vin', 'Vapex', 'Vout', 'PkAy', ...
    'RMSbeta', 'Pkbeta', 'Dur', 'AvgTRL', 'AvgTRR');
fprintf('%s\n', hdr);
fprintf('%s\n', repmat('-', 1, length(hdr)));

turn_table = zeros(N_turns, 15);
Xc = zeros(N_turns,1); Yc = zeros(N_turns,1);

for k = 1:N_turns
    idx = turn_starts(k):turn_ends(k);
    
    s_mid     = mean(s(idx));
    lap_num   = floor(s_mid / lap_length) + 1;
    s_in_lap  = mod(s_mid, lap_length);
    
    Xc(k)     = mean(X(idx));
    Yc(k)     = mean(Y(idx));
    
    v_in     = vx(idx(1))   * 3.6;
    v_apex   = min(vx(idx)) * 3.6;
    v_out    = vx(idx(end)) * 3.6;
    
    pk_ay    = max(Ay_G(idx));
    rms_b    = rms(beta(idx));
    pk_b     = max(abs(beta(idx)));
    dur      = t(idx(end)) - t(idx(1));
    
    mean_trl = mean(T_RL(idx));
    mean_trr = mean(T_RR(idx));
    
    turn_table(k,:) = [k, lap_num, s_in_lap, t(idx(1)), v_in, v_apex, ...
                       v_out, pk_ay, rms_b, pk_b, dur, Xc(k), Yc(k), mean_trl, mean_trr];
                   
    fprintf('%4d  %6d  %5.0fm  %8.1f  %7.1f %7.1f %7.1f  %6.2f  %8.3f  %8.3f  %5.2f  %7.1f  %7.1f\n', ...
        k, lap_num, s_in_lap, t(idx(1)), v_in, v_apex, v_out, ...
        pk_ay, rms_b, pk_b, dur, mean_trl, mean_trr);
end

% ── 5. Cluster Turns by Spatial (X,Y) Centroid ───────────────────────────
R_cluster   = 120;     
cluster_xy  = [];      
corner_id   = zeros(N_turns, 1);

for k = 1:N_turns
    if isempty(cluster_xy)
        cluster_xy(end+1, :) = [Xc(k), Yc(k)];
        corner_id(k) = 1;
    else
        d = sqrt((cluster_xy(:,1) - Xc(k)).^2 + (cluster_xy(:,2) - Yc(k)).^2);
        [d_min, c_idx] = min(d);
        if d_min < R_cluster
            corner_id(k) = c_idx;
            n_in = sum(corner_id == c_idx);
            cluster_xy(c_idx,:) = ((n_in-1)*cluster_xy(c_idx,:) + [Xc(k), Yc(k)]) / n_in;
        else
            cluster_xy(end+1, :) = [Xc(k), Yc(k)];
            corner_id(k) = size(cluster_xy, 1);
        end
    end
end
N_corners = size(cluster_xy, 1);

first_time = zeros(N_corners, 1);
for c = 1:N_corners
    first_time(c) = min(turn_table(corner_id == c, 4));
end
[~, order]   = sort(first_time);

new_id       = zeros(N_corners, 1);
new_id(order)= 1:N_corners;
corner_id    = new_id(corner_id);
cluster_xy   = cluster_xy(order, :);

% ── 6. Per-Corner Summary ────────────────────────────────────────────────
fprintf('\n===============================================================\n');
fprintf(' CAR A: PER-CORNER SUMMARY (clustered by X,Y centroid)\n');
fprintf(' %d unique corners across %d laps\n', N_corners, max(turn_table(:,2)));
fprintf('===============================================================\n');

% ── 7. Global Sideslip Metrics ───────────────────────────────────────────
all_turn_mask = false(size(t));
for k = 1:N_turns
    all_turn_mask(turn_starts(k):turn_ends(k)) = true;
end

% Calculate RMS values exclusively during turns (logic retained)
beta_RMS_turns    = rms(beta(all_turn_mask));
vy_RMS_turns      = rms(vy(all_turn_mask)); 
ay_RMS_turns      = rms(ay_raw(all_turn_mask)); 

% ── 8. Visualisation ─────────────────────────────────────────────────────
fig = figure('Name','Turn-by-Turn Analysis - Car A','Position',[60 60 1400 950], ...
             'Color', [0.85 0.85 0.85]); 

% --- LEFT COLUMN (Histograms & Track) ---

% (a) Track Map (Top Left)
subplot(3,2,1);
plot(X(~all_turn_mask), Y(~all_turn_mask), '.', 'Color', [.6 .6 .6], 'MarkerSize', 1); hold on;
plot(X(all_turn_mask),  Y(all_turn_mask),  '.', 'Color', [.8 .2 .2], 'MarkerSize', 2);
for c = 1:N_corners
    plot(cluster_xy(c,1), cluster_xy(c,2), 'ko', 'MarkerFaceColor','y', 'MarkerSize', 12); 
    text(cluster_xy(c,1), cluster_xy(c,2), sprintf('%d', c), ...
        'HorizontalAlignment','center', 'FontWeight','bold', 'FontSize', 9, 'Color', 'k');
end
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('Track Map — %d corners', N_corners));
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k'); 

% (c) Per-corner Apex Speed (Mid Left)
subplot(3,2,3);
apex_by_corner = zeros(N_corners,1);
err_by_corner  = zeros(N_corners,1);
for c = 1:N_corners
    apex_by_corner(c) = mean(turn_table(corner_id==c, 6));
    err_by_corner(c)  = std(turn_table(corner_id==c, 6));
end
bar(1:N_corners, apex_by_corner, 'FaceColor', [.2 .5 .8]); hold on;
errorbar(1:N_corners, apex_by_corner, err_by_corner, 'k', 'LineStyle','none', 'CapSize',5);
xlabel('Corner #'); ylabel('Speed (km/h)');
title('Apex Speed per Corner (mean ± std)');
grid on;
xlim([0, N_corners+1]); 
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k'); 

% (f) Per-corner Average Motor Torques (Bottom Left)
subplot(3,2,5); 
trl_by_corner = zeros(N_corners,1);
trr_by_corner = zeros(N_corners,1);
for c = 1:N_corners
    trl_by_corner(c) = mean(turn_table(corner_id==c, 14)); 
    trr_by_corner(c) = mean(turn_table(corner_id==c, 15));
end
b = bar(1:N_corners, [trl_by_corner, trr_by_corner], 'grouped');
b(1).FaceColor = [0.2 0.6 0.5]; 
b(2).FaceColor = [0.8 0.5 0.1]; 
xlabel('Corner #'); ylabel('Average Torque (Nm)');
title('Average Motor Torque per Corner');
legend('Rear Left (RL)', 'Rear Right (RR)', 'Location', 'best');
grid on;
xlim([0, N_corners+1]); 
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k'); 

% --- RIGHT COLUMN (Time Histories Column) ---

% (b) Sideslip time history (Top Right)
subplot(3,2,2);
plot(t, beta, 'b', 'LineWidth', 0.4); hold on;
beta_in_turn = beta;
beta_in_turn(~all_turn_mask) = NaN;
plot(t, beta_in_turn, 'r', 'LineWidth', 0.6);
yline( beta_RMS_turns, '--k'); 
hold on; 
text(t(end)*0.8, beta_RMS_turns*1.1, sprintf('RMS turns = %.2f°', beta_RMS_turns), ...
    'Color','k', 'FontWeight', 'bold', 'FontSize', 9); 
yline(-beta_RMS_turns, '--k');
xlabel('Time (s)'); ylabel('\beta (deg)');
title('Sideslip (Beta) — red = inside detected turn');
grid on;
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k'); 

% (d) Lateral Velocity (Vy) time history (Mid Right)
subplot(3,2,4);
plot(t, vy, 'b', 'LineWidth', 0.4); hold on;
vy_in_turn = vy;
vy_in_turn(~all_turn_mask) = NaN;
plot(t, vy_in_turn, 'r', 'LineWidth', 0.6);
yline( vy_RMS_turns, '--k');
hold on;
text(t(end)*0.8, vy_RMS_turns*1.1, sprintf('RMS turns = %.2f m/s', vy_RMS_turns), ...
    'Color','k', 'FontWeight', 'bold', 'FontSize', 9); 
yline(-vy_RMS_turns, '--k');
xlabel('Time (s)'); ylabel('vy (m/s)');
title('Lateral Velocity (Vy) — red = inside detected turn');
grid on;
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k'); 

% (e) Lateral Acceleration (Ay) time history (Bottom Right)
subplot(3,2,6);
plot(t, ay_raw, 'b', 'LineWidth', 0.4); hold on;
ay_in_turn = ay_raw;
ay_in_turn(~all_turn_mask) = NaN;
plot(t, ay_in_turn, 'r', 'LineWidth', 0.6);
yline( ay_RMS_turns, '--k');
hold on;
text(t(end)*0.8, ay_RMS_turns*1.1, sprintf('RMS turns = %.2f m/s^2', ay_RMS_turns), ...
    'Color','k', 'FontWeight', 'bold', 'FontSize', 9); 
yline(-ay_RMS_turns, '--k');
xlabel('Time (s)'); ylabel('ay (m/s^2)'); 
title('Lateral Acceleration (Ay) — red = inside detected turn');
grid on;
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k'); 

% Title and Save
st = sgtitle('TRK\_1a Monza — Turn-by-Turn Analysis (Dual Motor)', 'FontWeight','bold','FontSize',14);
st.Color = 'k';
out_png = 'C:\Users\kaust\OneDrive\Desktop\Claude\TRK_1a_turn_analysis.png';
saveas(fig, out_png);
fprintf('\nPlot saved: %s\n', out_png);

% ── 9. Save Table as CSV ─────────────────────────────────────────────────
T = array2table(turn_table, 'VariableNames', ...
    {'TurnNum','LapNum','s_in_lap_m','TimeStart_s','Vin_kph','Vapex_kph', ...
     'Vout_kph','PeakAy_G','RMS_beta_deg','Peak_beta_deg','Duration_s', ...
     'Xcent_m','Ycent_m', 'Avg_TRL_Nm', 'Avg_TRR_Nm'});
T.CornerID = corner_id;
csvfile = 'C:\Users\kaust\OneDrive\Desktop\Claude\TRK_1a_turns.csv';
writetable(T, csvfile);
fprintf('Turn table saved: %s\n', csvfile);


%% Mechanical Parameters Dashboard - Car B
% TURN-BY-TURN ANALYSIS for TRK_1b Monza Simulation
% Reuses corner identification from Car A via track distance (s).
% All calculations are strictly limited to the first 1200 seconds.

% load(fullfile(proj, 'TRK_1b_results.mat'));   % loads out_b

% ── 1. Extract Signals & Apply 1200s Time Limit ─────────────────────────
t_b_full = out_b.tout;
idx_1200 = find(t_b_full <= 1200, 1, 'last'); % Limit everything to 1200s

t_b  = t_b_full(1:idx_1200);
vx_b = squeeze(out_b.vx_curr_b.Data);    vx_b = vx_b(1:idx_1200);
vy_b = squeeze(out_b.vy_curr_b.Data);    vy_b = vy_b(1:idx_1200);
r_b  = squeeze(out_b.r_curr_b.Data);     r_b  = r_b(1:idx_1200);
X_b  = squeeze(out_b.X_curr_b.Data);     X_b  = X_b(1:idx_1200);
Y_b  = squeeze(out_b.Y_curr_b.Data);     Y_b  = Y_b(1:idx_1200);
s_b  = squeeze(out_b.s_b.Data);          s_b  = s_b(1:idx_1200);

T_RL_b = squeeze(out_b.Trl_fric_b.Data); T_RL_b = T_RL_b(1:idx_1200); % Brake Torque
T_RR_b = squeeze(out_b.Trr_fric_b.Data); T_RR_b = T_RR_b(1:idx_1200);

ay_raw_b = -(vx_b .* r_b);
Ay_G_b   = abs(ay_raw_b) / g;                           
beta_b   = atan2(vy_b, max(vx_b, 0.1)) * (180/pi);      

% ── 2. Distance-Based Turn Mapping (from Car A) ──────────────────────────
fprintf('\n===============================================================\n');
fprintf(' CAR B: MAPPING TURNS FROM CAR A (Distance-based, limited to 1200s)\n');
fprintf('===============================================================\n\n');

turn_table_b  = [];
corner_id_b   = [];
all_turn_mask_b = false(size(t_b));

for k = 1:N_turns
    % Get physical start/end track distances of the turn from Car A
    s_start = s(turn_starts(k));
    s_end   = s(turn_ends(k));
    
    % Find those exact physical track locations in Car B's simulation
    idx_b = find(s_b >= s_start & s_b <= s_end);
    
    % If Car B hasn't reached this turn within 1200s, ignore it
    if isempty(idx_b)
        continue; 
    end
    
    all_turn_mask_b(idx_b) = true; % Add to global turn mask
    
    s_mid     = mean(s_b(idx_b));
    lap_num   = floor(s_mid / lap_length) + 1;
    s_in_lap  = mod(s_mid, lap_length);
    
    Xc_b      = mean(X_b(idx_b));
    Yc_b      = mean(Y_b(idx_b));
    
    v_in      = vx_b(idx_b(1))   * 3.6;
    v_bpex    = min(vx_b(idx_b)) * 3.6;
    v_out     = vx_b(idx_b(end)) * 3.6;
    
    pk_by     = max(Ay_G_b(idx_b));
    rms_b_val = rms(beta_b(idx_b));
    pk_b_val  = max(abs(beta_b(idx_b)));
    dur       = t_b(idx_b(end)) - t_b(idx_b(1));
    
    mean_trl  = mean(T_RL_b(idx_b));
    mean_trr  = mean(T_RR_b(idx_b));
    
    % Store the metrics, keeping the original Car A turn # and corner ID
    turn_table_b(end+1,:) = [k, lap_num, s_in_lap, t_b(idx_b(1)), v_in, v_bpex, ...
                             v_out, pk_by, rms_b_val, pk_b_val, dur, Xc_b, Yc_b, mean_trl, mean_trr];
    corner_id_b(end+1, 1) = corner_id(k);
    
    fprintf('Mapped Turn %4d | Corner %2d | Time: %8.1f s | Vapex: %7.1f km/h\n', ...
        k, corner_id(k), t_b(idx_b(1)), v_bpex);
end

% ── 7. Global Sideslip Metrics (Car B) ───────────────────────────────────
beta_RMS_turns_b  = rms(beta_b(all_turn_mask_b));
vy_RMS_turns_b    = rms(vy_b(all_turn_mask_b)); 
ay_RMS_turns_b    = rms(ay_raw_b(all_turn_mask_b)); 

% ── 8. Visualisation (Car B) ─────────────────────────────────────────────
fig_b = figure('Name','Turn-by-Turn Analysis - Car B','Position',[80 80 1400 950], ...
             'Color', [0.85 0.85 0.85]); 

% --- LEFT COLUMN (Histograms & Track) ---

% (a) Track Map (Top Left) - Using Car A's cluster_xy to maintain visual consistency
subplot(3,2,1);
plot(X_b(~all_turn_mask_b), Y_b(~all_turn_mask_b), '.', 'Color', [.6 .6 .6], 'MarkerSize', 1); hold on;
plot(X_b(all_turn_mask_b),  Y_b(all_turn_mask_b),  '.', 'Color', [.8 .2 .2], 'MarkerSize', 2);
for c = 1:N_corners
    plot(cluster_xy(c,1), cluster_xy(c,2), 'ko', 'MarkerFaceColor','y', 'MarkerSize', 12); 
    text(cluster_xy(c,1), cluster_xy(c,2), sprintf('%d', c), ...
        'HorizontalAlignment','center', 'FontWeight','bold', 'FontSize', 9, 'Color', 'k');
end
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('Track Map — %d corners', N_corners));
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k'); 

% (c) Per-corner Apex Speed (Mid Left)
subplot(3,2,3);
apex_by_corner_b = zeros(N_corners,1);
err_by_corner_b  = zeros(N_corners,1);
for c = 1:N_corners
    mask = corner_id_b == c;
    if any(mask)
        apex_by_corner_b(c) = mean(turn_table_b(mask, 6));
        err_by_corner_b(c)  = std(turn_table_b(mask, 6));
    end
end
bar(1:N_corners, apex_by_corner_b, 'FaceColor', [.2 .5 .8]); hold on;
errorbar(1:N_corners, apex_by_corner_b, err_by_corner_b, 'k', 'LineStyle','none', 'CapSize',5);
xlabel('Corner #'); ylabel('Speed (km/h)');
title('Apex Speed per Corner (mean ± std)');
grid on;
xlim([0, N_corners+1]); 
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k'); 

% (f) Per-corner Average Brake Torques (Bottom Left)
subplot(3,2,5); 
trl_by_corner_b = zeros(N_corners,1);
trr_by_corner_b = zeros(N_corners,1);
for c = 1:N_corners
    mask = corner_id_b == c;
    if any(mask)
        trl_by_corner_b(c) = mean(turn_table_b(mask, 14)); 
        trr_by_corner_b(c) = mean(turn_table_b(mask, 15));
    end
end
b_b = bar(1:N_corners, [trl_by_corner_b, trr_by_corner_b], 'grouped');
b_b(1).FaceColor = [0.2 0.6 0.5]; 
b_b(2).FaceColor = [0.8 0.5 0.1]; 
xlabel('Corner #'); ylabel('Average Torque (Nm)');
title('Average Brake Torque per Corner');
legend('Rear Left (RL)', 'Rear Right (RR)', 'Location', 'best');
grid on;
xlim([0, N_corners+1]); 
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k'); 

% --- RIGHT COLUMN (Time Histories Column) ---

% (b) Sideslip time history (Top Right)
subplot(3,2,2);
plot(t_b, beta_b, 'b', 'LineWidth', 0.4); hold on;
beta_in_turn_b = beta_b;
beta_in_turn_b(~all_turn_mask_b) = NaN;
plot(t_b, beta_in_turn_b, 'r', 'LineWidth', 0.6);
yline( beta_RMS_turns_b, '--k'); 
hold on; 
text(t_b(end)*0.8, beta_RMS_turns_b*1.1, sprintf('RMS turns = %.2f°', beta_RMS_turns_b), ...
    'Color','k', 'FontWeight', 'bold', 'FontSize', 9); 
yline(-beta_RMS_turns_b, '--k');
xlabel('Time (s)'); ylabel('\beta (deg)');
title('Sideslip (Beta) — red = inside detected turn');
grid on;
xlim([0, 1200]);
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k'); 

% (d) Lateral Velocity (Vy) time history (Mid Right)
subplot(3,2,4);
plot(t_b, vy_b, 'b', 'LineWidth', 0.4); hold on;
vy_in_turn_b = vy_b;
vy_in_turn_b(~all_turn_mask_b) = NaN;
plot(t_b, vy_in_turn_b, 'r', 'LineWidth', 0.6);
yline( vy_RMS_turns_b, '--k');
hold on;
text(t_b(end)*0.8, vy_RMS_turns_b*1.1, sprintf('RMS turns = %.2f m/s', vy_RMS_turns_b), ...
    'Color','k', 'FontWeight', 'bold', 'FontSize', 9); 
yline(-vy_RMS_turns_b, '--k');
xlabel('Time (s)'); ylabel('vy (m/s)');
title('Lateral Velocity (Vy) — red = inside detected turn');
grid on;
xlim([0, 1200]);
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k'); 

% (e) Lateral Acceleration (Ay) time history (Bottom Right)
subplot(3,2,6);
plot(t_b, ay_raw_b, 'b', 'LineWidth', 0.4); hold on;
ay_in_turn_b = ay_raw_b;
ay_in_turn_b(~all_turn_mask_b) = NaN;
plot(t_b, ay_in_turn_b, 'r', 'LineWidth', 0.6);
yline( ay_RMS_turns_b, '--k');
hold on;
text(t_b(end)*0.8, ay_RMS_turns_b*1.1, sprintf('RMS turns = %.2f m/s^2', ay_RMS_turns_b), ...
    'Color','k', 'FontWeight', 'bold', 'FontSize', 9); 
yline(-ay_RMS_turns_b, '--k');
xlabel('Time (s)'); ylabel('ay (m/s^2)'); 
title('Lateral Acceleration (Ay) — red = inside detected turn');
grid on;
xlim([0, 1200]);
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k'); 

% Title and Save
st_b = sgtitle('TRK\_1b Monza — Turn-by-Turn Analysis (Single Motor)', 'FontWeight','bold','FontSize',14);
st_b.Color = 'k';
out_png_b = 'C:\Users\kaust\OneDrive\Desktop\Claude\TRK_1b_turn_analysis.png';
saveas(fig_b, out_png_b);
fprintf('\nPlot saved: %s\n', out_png_b);

% ── 9. Save Table as CSV ─────────────────────────────────────────────────
T_b = array2table(turn_table_b, 'VariableNames', ...
    {'TurnNum','LapNum','s_in_lap_m','TimeStart_s','Vin_kph','Vapex_kph', ...
     'Vout_kph','PeakAy_G','RMS_beta_deg','Peak_beta_deg','Duration_s', ...
     'Xcent_m','Ycent_m', 'Avg_TRL_Nm', 'Avg_TRR_Nm'});
T_b.CornerID = corner_id_b;
csvfile_b = 'C:\Users\kaust\OneDrive\Desktop\Claude\TRK_1b_turns.csv';
writetable(T_b, csvfile_b);
fprintf('Turn table saved: %s\n', csvfile_b);

%% 10. COMPARATIVE STUDY OF CAR A AND B PERFORMANCE (HISTOGRAMS)
fprintf('\n===============================================================\n');
fprintf(' COMPARATIVE STUDY: KINEMATICS & TORQUES (HISTOGRAMS)\n');
fprintf('===============================================================\n');

% ... [Keep your data prep and calculation loops exactly as they were] ...

% ── 3. FIGURE 1: KINEMATICS HISTOGRAMS ─────────────────────────────────
fig_comp1 = figure('Name','Car A vs Car B: Kinematics','Position',[50 50 1000 900], 'Color', 'w');

% (A) Lateral Acceleration
subplot(3,1,1);
b_ay = bar(1:N_corners, [ay_a_comp(:), ay_b_comp(:)], 'grouped');
b_ay(1).FaceColor = [0.2 0.5 0.8]; b_ay(2).FaceColor = [0.8 0.2 0.2];
title('Average Lateral Acceleration (ay) per Corner'); ylabel('ay (m/s^2)');
legend('Car A (Blue)', 'Car B (Red)', 'Location', 'northeastoutside');
grid(gca, 'on'); xlim(gca, [0.5, N_corners+0.5]); set(gca, 'XColor', 'k', 'YColor', 'k');
% Demarcation
yl = ylim; yl_new = [yl(1), yl(2) + (yl(2)-yl(1))*0.08]; ylim(yl_new);
for i=0.5:1:N_corners+0.5; xline(i, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5, 'HandleVisibility', 'off'); end
for c=1:N_corners; text(c, yl_new(2)-(yl_new(2)-yl_new(1))*0.02, sprintf('C%d', c), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'FontWeight', 'bold', 'FontSize', 11, 'Color', 'k'); end

% (B) Lateral Velocity
subplot(3,1,2);
b_vy = bar(1:N_corners, [vy_a_comp(:), vy_b_comp(:)], 'grouped');
b_vy(1).FaceColor = [0.2 0.5 0.8]; b_vy(2).FaceColor = [0.8 0.2 0.2];
title('Average Lateral Velocity (vy) per Corner'); ylabel('vy (m/s)');
grid(gca, 'on'); xlim(gca, [0.5, N_corners+0.5]); set(gca, 'XColor', 'k', 'YColor', 'k');
yl = ylim; yl_new = [yl(1), yl(2) + (yl(2)-yl(1))*0.08]; ylim(yl_new);
for i=0.5:1:N_corners+0.5; xline(i, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5, 'HandleVisibility', 'off'); end
for c=1:N_corners; text(c, yl_new(2)-(yl_new(2)-yl_new(1))*0.02, sprintf('C%d', c), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'FontWeight', 'bold', 'FontSize', 11, 'Color', 'k'); end

% (C) Sideslip
subplot(3,1,3);
b_beta = bar(1:N_corners, [beta_a_comp(:), beta_b_comp(:)], 'grouped');
b_beta(1).FaceColor = [0.2 0.5 0.8]; b_beta(2).FaceColor = [0.8 0.2 0.2];
title('Average Sideslip Angle (\beta) per Corner'); ylabel('\beta (deg)'); xlabel('Corner ID');
grid(gca, 'on'); xlim(gca, [0.5, N_corners+0.5]); set(gca, 'XColor', 'k', 'YColor', 'k');
yl = ylim; yl_new = [yl(1), yl(2) + (yl(2)-yl(1))*0.08]; ylim(yl_new);
for i=0.5:1:N_corners+0.5; xline(i, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5, 'HandleVisibility', 'off'); end
for c=1:N_corners; text(c, yl_new(2)-(yl_new(2)-yl_new(1))*0.02, sprintf('C%d', c), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'FontWeight', 'bold', 'FontSize', 11, 'Color', 'k'); end

% ── 4. FIGURE 2: SPEED & TORQUE ─────────────────────────────────────────
fig_comp2 = figure('Name','Comparative Study: Speed & Torques','Position',[600 50 1000 800], 'Color', 'w');

% (A) Apex Speed
subplot(2,1,1);
b_spd2 = bar(1:N_corners, [apex_a_comp(:), apex_b_comp(:)], 'grouped');
b_spd2(1).FaceColor = [0.2 0.5 0.8]; b_spd2(2).FaceColor = [0.8 0.2 0.2]; 
title('Turn-by-Turn Apex Speed Comparison'); ylabel('Speed (km/h)');
legend('Car A (Dual Motor)', 'Car B (Single Motor)', 'Location', 'northeastoutside');
grid(gca, 'on'); xlim(gca, [0.5, N_corners+0.5]); set(gca, 'XColor', 'k', 'YColor', 'k');
yl = ylim; yl_new = [yl(1), yl(2) + (yl(2)-yl(1))*0.08]; ylim(yl_new);
for i=0.5:1:N_corners+0.5; xline(i, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5, 'HandleVisibility', 'off'); end
for c=1:N_corners; text(c, yl_new(2)-(yl_new(2)-yl_new(1))*0.02, sprintf('C%d', c), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'FontWeight', 'bold', 'FontSize', 11, 'Color', 'k'); end

% (B) Torque
subplot(2,1,2);
tq_data_comp = [trl_a_comp(:)*10, trr_a_comp(:)*10, trl_b_comp(:), trr_b_comp(:), tm_b_comp(:)*10];
b_tq2 = bar(1:N_corners, tq_data_comp, 'grouped');
b_tq2(1).FaceColor = [0.2 0.6 0.5]; b_tq2(2).FaceColor = [0.8 0.5 0.1];
b_tq2(3).FaceColor = [0.8 0 0]; b_tq2(4).FaceColor = [1 0.4 0]; b_tq2(5).FaceColor = [0.4 0.4 0.4];
title('Turn-by-Turn Torque Comparison'); xlabel('Corner ID'); ylabel('Torque (Nm)');
legend('RL_A*10', 'RR_A*10', 'RL_B Brake', 'RR_B Brake', 'M_B*10', 'Location', 'northeastoutside');
grid(gca, 'on'); xlim(gca, [0.5, N_corners+0.5]); set(gca, 'XColor', 'k', 'YColor', 'k');
ylim([-2500, 4000]);
for i=0.5:1:N_corners+0.5; xline(i, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5, 'HandleVisibility', 'off'); end
for c=1:N_corners; text(c, 4000 - (6500*0.02), sprintf('C%d', c), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'FontWeight', 'bold', 'FontSize', 11, 'Color', 'k'); end

%% 11. COMPARATIVE STUDY: LAP 2 ONLY
fprintf('\n===============================================================\n');
fprintf(' COMPARATIVE STUDY: LAP 2 ONLY\n');
fprintf('===============================================================\n');

apex_a_l2 = zeros(N_corners,1); apex_b_l2 = zeros(N_corners,1);
tq_a_l2   = zeros(N_corners, 2); tq_b_l2   = zeros(N_corners, 3);
ay_a_l2   = zeros(N_corners,1); ay_b_l2   = zeros(N_corners,1);
beta_a_l2 = zeros(N_corners,1); beta_b_l2 = zeros(N_corners,1);

for c = 1:N_corners
    idx_a = find(corner_id == c & turn_table(:, 2) == 2);
    idx_b = find(corner_id_b == c & turn_table_b(:, 2) == 2);
    if any(idx_a)
        apex_a_l2(c) = mean(turn_table(idx_a, 6));
        tq_a_l2(c,:) = [mean(turn_table(idx_a, 14)), mean(turn_table(idx_a, 15))];
        ay_a_l2(c)   = mean(turn_table(idx_a, 8));
        beta_a_l2(c) = mean(turn_table(idx_a, 9));
    end
    if any(idx_b)
        apex_b_l2(c) = mean(turn_table_b(idx_b, 6));
        tq_b_l2(c,1:2) = [mean(turn_table_b(idx_b, 14)), mean(turn_table_b(idx_b, 15))];
        tq_b_l2(c,3)   = mean(abs(out_b.Trl_m_b.Data * 2));
        ay_b_l2(c)     = mean(turn_table_b(idx_b, 8));
        beta_b_l2(c)   = mean(turn_table_b(idx_b, 9));
    end
end

plot_comparison_dashboard(N_corners, cluster_xy, ay_a_l2, ay_b_l2, zeros(N_corners,1), zeros(N_corners,1), beta_a_l2, beta_b_l2, ...
    apex_a_l2, apex_b_l2, tq_a_l2, tq_b_l2, X_track, Y_track, X_a, Y_a, X_b, Y_b, 'Section 11: Lap 2');

% ── Local Plotting Function ──
function plot_comparison_dashboard(N, cluster, ay_a, ay_b, vy_a, vy_b, beta_a, beta_b, apex_a, apex_b, tq_a, tq_b, Xtr, Ytr, Xa, Ya, Xb, Yb, title_str)
% Kinematics Figure
figure('Name', [title_str, ' Kinematics'], 'Position', [50 50 1000 800], 'Color', 'w');
d = {ay_a, ay_b, 'Ay (g)'; vy_a, vy_b, 'Vy (m/s)'; beta_a, beta_b, 'Sideslip (deg)'};
for s = 1:3
    subplot(3,1,s); bar(1:N, [d{s,1}, d{s,2}], 'grouped'); title(['Averages: ', d{s,3}]);
    legend('Car A', 'Car B', 'Location','northeast'); grid on; xlim([0.5 N+0.5]); 
    for i=0.5:1:N+0.5; xline(i, ':', 'HandleVisibility','off'); end
    for c=1:N; text(c, max(ylim)*0.9, num2str(c), 'HorizontalAlignment','center', 'FontWeight','bold'); end
end
% Perf Figure (Narrow Map, Broad Histograms)
figure('Name', [title_str, ' Performance'], 'Position', [150 50 1400 700], 'Color', 'w');
subplot('Position', [0.08 0.12 0.25 0.75]); % Track Narrow
plot(-Ytr, Xtr, 'k--', 'LineWidth', 1.5, 'HandleVisibility','off'); hold on;
plot(-Ya, Xa, 'b', 'LineWidth', 1); plot(-Yb, Xb, 'r', 'LineWidth', 1);
for c=1:N; plot(-cluster(c,2), cluster(c,1), 'ko', 'MarkerFaceColor','y', 'MarkerSize', 6, 'HandleVisibility','off');
    text(-cluster(c,2), cluster(c,1), num2str(c), 'FontWeight','bold'); end
title('Track Map'); axis equal; grid on;

subplot('Position', [0.42 0.55 0.5 0.35]); % Speed Broad
b = bar(1:N, [apex_a, apex_b], 'grouped');
title('Apex Speed'); ylabel('km/h'); legend('Car A', 'Car B', 'Location','northeast');
grid on; xlim([0.5 N+0.5]);
for c=1:N; text(c, max(ylim)*0.9, num2str(c), 'HorizontalAlignment','center', 'FontWeight','bold'); end

subplot('Position', [0.42 0.12 0.5 0.35]); % Torque Broad
bar(1:N, [tq_a(:,1)*10, tq_a(:,2)*10, tq_b(:,1), tq_b(:,2), tq_b(:,3)*10], 'grouped');
title('Torque Comparison'); ylabel('Nm'); ylim([-2500, 4000]);
legend('RL_A*10', 'RR_A*10', 'RL_B Brake', 'RR_B Brake', 'M_B*10', 'Location','northeast');
grid on; xlim([0.5 N+0.5]);
for c=1:N; text(c, 3500, num2str(c), 'HorizontalAlignment','center', 'FontWeight','bold'); end
end
