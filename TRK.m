% =========================================================================
% TRACK SIMULATION STARTUP SCRIPT (TRK)
% Location: Autodromo Nazionale Monza
% Architecture: Closed-Loop Pure Pursuit with 12-Lap Tiling & Smooth Launch
% =========================================================================
clc; close all;
disp('--- Initializing Monza Track Parameters ---');
% -------------------------------------------------------------------------
% 1. VEHICLE PARAMETERS (1600kg High-Performance Base)
% -------------------------------------------------------------------------
veh_params.m = 1600; veh_params.Iz = 2600; veh_params.Iw = 1.2;
veh_params.lf = 1.3; veh_params.lr = 1.3; veh_params.L = 2.6; 
veh_params.tw = 1.65; veh_params.h_cg = 0.35; veh_params.Rw = 0.34;
veh_params.rho = 1.225; veh_params.CdA = 0.6; veh_params.g = 9.81;
veh_params.roll_bias_f = 0.55; veh_params.roll_bias_r = 0.45;
veh_params.brake_bias_f = 0.65;
% Base Tire Stiffness 
veh_params.ks_f = 345000; veh_params.ks_r = 410000;
veh_params.C_rr = 0.015; veh_params.C_tire = 1.3; veh_params.E_tire = -1.0;
% Powertrain & Thermal (3P 238S Configuration)
veh_params.G = 10; veh_params.T_motor_max_hw = 500;
veh_params.P_batt_max_kw = 500; veh_params.P_regen_max_kw = 250;
veh_params.V_pack = 3.6 * 238; 
veh_params.Cap_Ah = (80 * 1000) / veh_params.V_pack; 
veh_params.R_pack = 0.002 * 238 / 3;
veh_params.m_batt = 450;  veh_params.cp_batt = 900;  
veh_params.hA_cool_batt = 25; veh_params.T_coolant_batt = 25;
veh_params.m_motor = 80;  veh_params.cp_motor = 500; 
veh_params.hA_cool_motor = 350; veh_params.T_ambient = 30;

% -------------------------------------------------------------------------
% 2. LQR GAIN SCHEDULING (Relaxed for Stability)
% -------------------------------------------------------------------------
disp('-> Synthesizing LQR Feedback Gains...');
Q_lqr = diag([1/(2.0^2), 1/(0.01^2)]);
R_lqr = 1/(2500^2);
V_op = 5:5:85; % Scheduled up to 85 m/s (~306 km/h)
K_scheduled = zeros(length(V_op), 2); 
for i = 1:length(V_op)
    vx_t = V_op(i);
    A_lin = [-(veh_params.ks_f + veh_params.ks_r)/(veh_params.m*vx_t), ((veh_params.ks_r*veh_params.lr - veh_params.ks_f*veh_params.lf)/(veh_params.m*vx_t)) - vx_t;
             (veh_params.ks_r*veh_params.lr - veh_params.ks_f*veh_params.lf)/(veh_params.Iz*vx_t), -(veh_params.ks_f*veh_params.lf^2 + veh_params.ks_r*veh_params.lr^2)/(veh_params.Iz*vx_t)];
    
    B_lin = [veh_params.ks_f/veh_params.m, 0; 
             (veh_params.lf*veh_params.ks_f)/veh_params.Iz, 1/veh_params.Iz];
         
    try
        K_scheduled(i, :) = lqr(A_lin, B_lin(:,2), Q_lqr, R_lqr); 
    catch
        if i > 1, K_scheduled(i, :) = K_scheduled(i-1, :); end
    end
end

% -------------------------------------------------------------------------
% 3. TRACK TELEMETRY INGESTION & MULTI-LAP TILING
% -------------------------------------------------------------------------
disp('-> Loading Monza Telemetry Data...');
opts = detectImportOptions('Monza_Transformed_Lap1.csv');
track_data = readtable('Monza_Transformed_Lap1.csv', opts);
% Extract raw timeseries
t_raw = track_data.timestamp;
vx_raw = track_data.Vx_vehicle_frame;    
X_raw = track_data.Track_X;
Y_raw = track_data.Track_Y;

% Sanitize: Ensure strictly monotonic timestamps
[t_raw, unique_idx] = unique(t_raw);
vx_raw = vx_raw(unique_idx);
X_raw = X_raw(unique_idx);
Y_raw = Y_raw(unique_idx);

% Apply a Savitzky-Golay filter to remove microscopic GPS kinks 
window_size = 25; 
X_raw = smoothdata(X_raw, 'sgolay', window_size);
Y_raw = smoothdata(Y_raw, 'sgolay', window_size);
disp(['-> Applied Savitzky-Golay smoothing (Window: ', num2str(window_size), ') to track coordinates.']);

% =========================================================================
% NEW: MULTI-LAP TRACK TILING (12 Laps)
% =========================================================================
num_laps = 12; 

% Drop the last point of the lap to prevent a duplicate (dx=0, dy=0) stutter
X_lap = X_raw(1:end-1);
Y_lap = Y_raw(1:end-1);
vx_lap = vx_raw(1:end-1);

% Tile the arrays
X_tiled = repmat(X_lap, num_laps, 1);
Y_tiled = repmat(Y_lap, num_laps, 1);
vx_tiled = repmat(vx_lap, num_laps, 1);

% Overwrite the base arrays with the new endless track
X_raw = X_tiled;
Y_raw = Y_tiled;
vx_raw = vx_tiled;

% Calculate Cumulative Distance (s) across all 12 tiled laps
dX = diff(X_raw);
dY = diff(Y_raw);
ds = sqrt(dX.^2 + dY.^2);
s_raw = [0; cumsum(ds)]; 

% -------------------------------------------------------------------------
% 4. TOP SPEED MULTIPLIER & SPEED BOUNDARY BOXES
% -------------------------------------------------------------------------
target_top_speed_kmh = 260.0; 
original_top_speed_kmh = max(vx_raw) * 3.6; 
speed_multiplier = target_top_speed_kmh / original_top_speed_kmh;

% Scale the velocity array down globally
vx_scaled = vx_raw * speed_multiplier;
disp(['-> Scaled track velocity by factor of ', num2str(speed_multiplier, '%.3f'), ' (Target: ', num2str(target_top_speed_kmh), ' km/h)']);

% --- 14 Hairpin Speed Limiters (Bounding Boxes) ---
v_limit_ms = 90 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= -1400 && X_raw(i) <= -1300) && (Y_raw(i) >= 800 && Y_raw(i) <= 1200)
        if vx_scaled(i) > v_limit_ms; vx_scaled(i) = v_limit_ms; end
    end
end
v_limit_ms2 = 60 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= 500 && X_raw(i) <= 800) && (Y_raw(i) >= 50 && Y_raw(i) <= 300)
        if vx_scaled(i) > v_limit_ms2; vx_scaled(i) = v_limit_ms2; end
    end
end
v_limit_ms3 = 160 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= 600 && X_raw(i) <= 1000) && (Y_raw(i) >= 300 && Y_raw(i) <= 600)
        if vx_scaled(i) > v_limit_ms3; vx_scaled(i) = v_limit_ms3; end
    end
end
v_limit_ms4 = 160 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= -1200 && X_raw(i) <= -700) && (Y_raw(i) >= 50 && Y_raw(i) <= 400)
        if vx_scaled(i) > v_limit_ms4; vx_scaled(i) = v_limit_ms4; end
    end
end
v_limit_ms5 = 100 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= -1200 && X_raw(i) <= -1000) && (Y_raw(i) >= 1150 && Y_raw(i) <= 1300)
        if vx_scaled(i) > v_limit_ms5; vx_scaled(i) = v_limit_ms5; end
    end
end
v_limit_ms6 = 180 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= -600 && X_raw(i) <= -300) && (Y_raw(i) >= 300 && Y_raw(i) <= 600)
        if vx_scaled(i) > v_limit_ms6; vx_scaled(i) = v_limit_ms6; end
    end
end
v_limit_ms7 = 230 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= -100 && X_raw(i) <= 0) && (Y_raw(i) >= 15 && Y_raw(i) <= 30)
        if vx_scaled(i) > v_limit_ms7; vx_scaled(i) = v_limit_ms7; end
    end
end
v_limit_ms8 = 250 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= -220 && X_raw(i) <= -100) && (Y_raw(i) >= 25 && Y_raw(i) <= 40)
        if vx_scaled(i) > v_limit_ms8; vx_scaled(i) = v_limit_ms8; end
    end
end
v_limit_ms9 = 250 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= -340 && X_raw(i) <= -220) && (Y_raw(i) >= 38 && Y_raw(i) <= 50)
        if vx_scaled(i) > v_limit_ms9; vx_scaled(i) = v_limit_ms9; end
    end
end
v_limit_ms10 = 160 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= -480 && X_raw(i) <= -340) && (Y_raw(i) >= 50 && Y_raw(i) <= 80)
        if vx_scaled(i) > v_limit_ms10; vx_scaled(i) = v_limit_ms10; end
    end
end
v_limit_ms11 = 160 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= -800 && X_raw(i) <= -480) && (Y_raw(i) >= 70 && Y_raw(i) <= 600)
        if vx_scaled(i) > v_limit_ms11; vx_scaled(i) = v_limit_ms11; end
    end
end
v_limit_ms12 = 160 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= -650 && X_raw(i) <= -300) && (Y_raw(i) >= 650 && Y_raw(i) <= 750)
        if vx_scaled(i) > v_limit_ms12; vx_scaled(i) = v_limit_ms12; end
    end
end
v_limit_ms13 = 120 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= 800 && X_raw(i) <= 1100) && (Y_raw(i) >= 50 && Y_raw(i) <= 600)
        if vx_scaled(i) > v_limit_ms13; vx_scaled(i) = v_limit_ms13; end
    end
end
v_limit_ms14 = 150 / 3.6; 
for i = 1:length(X_raw)
    if (X_raw(i) >= 650 && X_raw(i) <= 850) && (Y_raw(i) >= -100 && Y_raw(i) <= 25)
        if vx_scaled(i) > v_limit_ms14; vx_scaled(i) = v_limit_ms14; end
    end
end
disp('-> Applied speed limits to 14 Hairpin Bounding Boxes (Across all Laps).');

% Build Spatial Velocity Target (V_track) with Kickstart Ramp
V_track = vx_scaled;
ramp_dist = 150.0; % Smoothly ramp speed over the first 150 meters
for i = 1:length(s_raw)
    if s_raw(i) < ramp_dist
        % KICKSTART: Demand 10 m/s target while the car is at 0.1 m/s
        v_start = 10.0; 
        v_end = vx_scaled(i);
        if v_end > v_start
            V_track(i) = v_start + (v_end - v_start) * (s_raw(i) / ramp_dist);
        else
            V_track(i) = v_end;
        end
    end
end

% -------------------------------------------------------------------------
% 5. SIMULATION ENVIRONMENT SETUP & EXPORT
% -------------------------------------------------------------------------
sim_params.dt = 0.001;

% Buffer the simulation end time to ensure the car finishes 10 laps
% 10.5 laps duration accounts for scaled-down speed profile.
sim_params.t_end = (t_raw(end) * 10.5) / speed_multiplier; 
t_sim = (0:sim_params.dt:sim_params.t_end)';

% Export variables specifically formatted for the Pure Pursuit Block
assignin('base', 'X_track', X_raw); 
assignin('base', 'Y_track', Y_raw);
assignin('base', 'V_track', V_track);

% Environmental Disturbance: High grip tarmac (Monza)
mu_high = 1.2; 
mu_left = mu_high * ones(size(t_sim));  
mu_right = mu_high * ones(size(t_sim)); 
mu_profile_left = [t_sim, mu_left];
mu_profile_right = [t_sim, mu_right];

% Initial Conditions
sim_params.initial_speed = 0.1; % Start effectively at rest
sim_params.initial_X = X_raw(1); 
sim_params.initial_Y = Y_raw(1); 

% --- Robust Initial Heading Calculation ---
idx_ahead = find(s_raw > 15.0, 1, 'first'); 
if isempty(idx_ahead); idx_ahead = 2; end 
sim_params.initial_psi = atan2(Y_raw(idx_ahead)-Y_raw(1), X_raw(idx_ahead)-X_raw(1));
disp('--- Setup Complete: Ready for TRK_master.m Execution. ---');