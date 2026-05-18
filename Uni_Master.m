% =========================================================================
% UNIFIED MASTER SCRIPT: LATERAL DYNAMICS VALIDATION SUITE
% Executes OHZ, STP, RMP, RLS, Mimuro Generator, STR, SWS, CAL, SLA, DLC, and MUS.
% =========================================================================
disp('====================================================');
disp('   LATERAL DYNAMICS UNIFIED VALIDATION SUITE        ');
disp('====================================================');

%% ========================================================================
% 1. 1 HZ SINE DWELL (OHZ) EXECUTION & EXPORT
% =========================================================================
disp('>>> INITIATING 1 HZ SINE DWELL (OHZ) TEST <<<');
run('OHZ_master.m');
disp('-> Exporting OHZ Dashboards to Desktop...');
if ispc; desk_path = fullfile(getenv('USERPROFILE'), 'Desktop'); else; desk_path = fullfile(getenv('HOME'), 'Desktop'); end
out_dir = fullfile(desk_path, 'Lat_Dyn', 'OHZ');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
figs = findall(0, 'Type', 'figure');
for i = 1:length(figs)
    fig_name = figs(i).Name;
    if isempty(fig_name); fig_name = sprintf('Figure_%d', figs(i).Number); end
    safe_name = regexprep(fig_name, '[^a-zA-Z0-9]', '_');
    safe_name = regexprep(safe_name, '_+', '_');
    saveas(figs(i), fullfile(out_dir, [safe_name, '.jpg']), 'jpeg');
end
close all;

%% ========================================================================
% 2. STEERING PULSE (STP) EXECUTION & EXPORT
% =========================================================================
disp('>>> INITIATING STEERING PULSE (STP) TEST <<<');
run('STP_master.m');
disp('-> Exporting STP Dashboards to Desktop...');
if ispc; desk_path = fullfile(getenv('USERPROFILE'), 'Desktop'); else; desk_path = fullfile(getenv('HOME'), 'Desktop'); end
out_dir = fullfile(desk_path, 'Lat_Dyn', 'STP');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
figs = findall(0, 'Type', 'figure');
for i = 1:length(figs)
    fig_name = figs(i).Name;
    if isempty(fig_name); fig_name = sprintf('Figure_%d', figs(i).Number); end
    safe_name = regexprep(fig_name, '[^a-zA-Z0-9]', '_');
    safe_name = regexprep(safe_name, '_+', '_');
    saveas(figs(i), fullfile(out_dir, [safe_name, '.jpg']), 'jpeg');
end
close all;

%% ========================================================================
% 3. SLOW RAMP STEER (RMP) EXECUTION & EXPORT
% =========================================================================
disp('>>> INITIATING SLOW RAMP STEER (RMP) TEST <<<');
run('RMP_master.m');
disp('-> Exporting RMP Dashboards to Desktop...');
if ispc; desk_path = fullfile(getenv('USERPROFILE'), 'Desktop'); else; desk_path = fullfile(getenv('HOME'), 'Desktop'); end
out_dir = fullfile(desk_path, 'Lat_Dyn', 'RMP');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
figs = findall(0, 'Type', 'figure');
for i = 1:length(figs)
    fig_name = figs(i).Name;
    if isempty(fig_name); fig_name = sprintf('Figure_%d', figs(i).Number); end
    safe_name = regexprep(fig_name, '[^a-zA-Z0-9]', '_');
    safe_name = regexprep(safe_name, '_+', '_');
    saveas(figs(i), fullfile(out_dir, [safe_name, '.jpg']), 'jpeg');
end
close all;

%% ========================================================================
% 4. STEERING RELEASE (RLS) EXECUTION & EXPORT
% =========================================================================
disp('>>> INITIATING STEERING RELEASE (RLS) TEST <<<');
run('RLS_master.m');
disp('-> Exporting RLS Dashboards to Desktop...');
if ispc; desk_path = fullfile(getenv('USERPROFILE'), 'Desktop'); else; desk_path = fullfile(getenv('HOME'), 'Desktop'); end
out_dir = fullfile(desk_path, 'Lat_Dyn', 'RLS');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
figs = findall(0, 'Type', 'figure');
for i = 1:length(figs)
    fig_name = figs(i).Name;
    if isempty(fig_name); fig_name = sprintf('Figure_%d', figs(i).Number); end
    safe_name = regexprep(fig_name, '[^a-zA-Z0-9]', '_');
    safe_name = regexprep(safe_name, '_+', '_');
    saveas(figs(i), fullfile(out_dir, [safe_name, '.jpg']), 'jpeg');
end
close all;

%% ========================================================================
% 5. AUTOMATED MIMURO PLOT GENERATOR & EXPORT (WITH IDEAL BASELINE)
% =========================================================================
disp('>>> GENERATING SUPERIMPOSED MIMURO DYNAMICS PLOT <<<');
mat_filename = 'Mimuro_Data.mat';
if isfile(mat_filename)
    load(mat_filename, 'Mimuro');
    
    % =====================================================================
    % --- IDEAL LINEAR BICYCLE MODEL MATH (DYNAMIC EXTRACTION) ---
    % =====================================================================
    disp('-> Extracting Analytical Parameters from Workspace...');
    
    try
        % Dynamically extract parameters from your initialization struct
        m_ideal  = veh_params.m;       
        Iz_ideal = veh_params.Iz;      
        lf_ideal = veh_params.lf;      
        lr_ideal = veh_params.lr;      
        
        Cf_ideal = veh_params.ks_f;  
        Cr_ideal = veh_params.ks_r;   
        
        % Extract the exact steady-state test speed from the RLS telemetry
        Vx_ideal = mean(vx_a); 
        
    catch ME
        warning('Could not extract parameters from veh_params. Please ensure the test initialization script loaded veh_params to the workspace.');
        rethrow(ME);
    end
    
    % State-Space Matrix A (System Dynamics)
    A11 = -(Cf_ideal + Cr_ideal) / (m_ideal * Vx_ideal);
    A12 = (Cr_ideal * lr_ideal - Cf_ideal * lf_ideal) / (m_ideal * Vx_ideal) - Vx_ideal;
    A21 = (Cr_ideal * lr_ideal - Cf_ideal * lf_ideal) / (Iz_ideal * Vx_ideal);
    A22 = -(Cf_ideal * lf_ideal^2 + Cr_ideal * lr_ideal^2) / (Iz_ideal * Vx_ideal);
    A_mat = [A11, A12; A21, A22];
    
    % State-Space Matrix B (Steering Input) & Matrix C (Yaw Rate Output)
    B_mat = [Cf_ideal / m_ideal; (Cf_ideal * lf_ideal) / Iz_ideal];
    C_mat = [0, 1];
    
    % 1. Analytical Steady-State Gain [DC Gain = -C * A^-1 * B]
    ideal_gain = -C_mat * (A_mat \ B_mat);
    
    % 2 & 3. Analytical Natural Freq & Damping (From Eigenvalues of A)
    wn_ideal = sqrt(det(A_mat));
    ideal_damp = -trace(A_mat) / (2 * wn_ideal);
    ideal_freq_hz = wn_ideal / (2 * pi);
    
    % 4. Analytical Phase Lag at 1 Hz [Evaluate H(jw) = C * (jwI - A)^-1 * B]
    w_1hz = 2 * pi * 1.0;
    I_mat = eye(2);
    H_1hz = C_mat * ((1i * w_1hz * I_mat - A_mat) \ B_mat);
    ideal_phase = abs(angle(H_1hz) * (180 / pi)); % Absolute phase lag
    
    % Inject Ideal Data into the struct for automatic plotting
    Mimuro.Ideal_Linear_Model.STR.Gain = ideal_gain;
    Mimuro.Ideal_Linear_Model.STR.Damping_Ratio = ideal_damp;
    Mimuro.Ideal_Linear_Model.SWS.Natural_Freq_Hz = ideal_freq_hz;
    Mimuro.Ideal_Linear_Model.SWS.Phase_1Hz = ideal_phase;

    % =====================================================================
    % --- PLOTTING LOGIC ---
    % =====================================================================
    fig_mimuro = figure('Name', 'Mimuro Vehicle Dynamics Comparison', 'Color', 'w', 'Position', [150, 150, 900, 850]);
    clf(fig_mimuro); 
    hold on; axis equal; axis off;
    axis([-1.4 1.4 -1.4 1.4]); 
    
    cars = fieldnames(Mimuro);
    c_colors = {'b', 'r', 'k', 'g', 'm'}; 
    
    max_gain = 0; max_freq = 0; max_phase = 0; max_damp = 0;
    for i = 1:length(cars)
        c = cars{i};
        max_gain = max(max_gain, Mimuro.(c).STR.Gain);
        max_freq = max(max_freq, Mimuro.(c).SWS.Natural_Freq_Hz);
        max_phase = max(max_phase, abs(Mimuro.(c).SWS.Phase_1Hz)); 
        max_damp = max(max_damp, Mimuro.(c).STR.Damping_Ratio);
    end
    
    max_gain = max_gain * 1.25; max_freq = max_freq * 1.25;
    max_phase = max_phase * 1.25; max_damp = max_damp * 1.25;
    
    plot([0 0], [-1 1], 'k-', 'LineWidth', 1.5); 
    plot([-1 1], [0 0], 'k-', 'LineWidth', 1.5); 
    
    plot([0 0.5 0 -0.5 0], [0.5 0 -0.5 0 0.5], 'k:', 'Color', [0.8 0.8 0.8]);
    plot([0 1 0 -1 0], [1 0 -1 0 1], 'k:', 'Color', [0.8 0.8 0.8]);
    
    text(0, 1.15, 'Steady-State Gain (1/s) [Agility]', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    text(0.85, -0.15, 'Natural Frequency (Hz) [Responsiveness]', 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
    text(0, -1.15, 'Phase Lag at 1 Hz (deg) [Directness]', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    text(-0.85, -0.15, 'Damping Ratio (\zeta) [Stability]', 'HorizontalAlignment', 'right', 'FontWeight', 'bold');
    
    h_legend = zeros(1, length(cars));
    for i = 1:length(cars)
        c = cars{i};
        
        g_norm = Mimuro.(c).STR.Gain / max_gain;
        f_norm = Mimuro.(c).SWS.Natural_Freq_Hz / max_freq;
        p_norm = abs(Mimuro.(c).SWS.Phase_1Hz) / max_phase; 
        d_norm = Mimuro.(c).STR.Damping_Ratio / max_damp;
        
        X_poly = [0, f_norm, 0, -d_norm, 0];
        Y_poly = [g_norm, 0, -p_norm, 0, g_norm];
        
        if contains(c, 'Ideal')
            line_style = [c_colors{i} '.--'];
            line_width = 2.0;
        else
            line_style = [c_colors{i} '.-'];
            line_width = 2.5;
        end
        
        h_legend(i) = plot(X_poly, Y_poly, line_style, 'LineWidth', line_width, 'MarkerSize', 25);
        
        text(0.05, g_norm + (i*0.04 - 0.04), sprintf('%.3f', Mimuro.(c).STR.Gain), 'Color', c_colors{i}, 'FontWeight', 'bold');
        text(f_norm + 0.05, 0.05 + (i*0.04 - 0.04), sprintf('%.2f Hz', Mimuro.(c).SWS.Natural_Freq_Hz), 'Color', c_colors{i}, 'FontWeight', 'bold');
        text(0.05, -p_norm - (i*0.04 - 0.04), sprintf('%.1f\\circ', abs(Mimuro.(c).SWS.Phase_1Hz)), 'Color', c_colors{i}, 'FontWeight', 'bold');
        text(-d_norm - 0.05, 0.05 + (i*0.04 - 0.04), sprintf('%.3f', Mimuro.(c).STR.Damping_Ratio), 'Color', c_colors{i}, 'HorizontalAlignment', 'right', 'FontWeight', 'bold');
    end
    
    clean_names = strrep(cars, '_', ' ');
    legend(h_legend, clean_names, 'Location', 'northeastoutside', 'FontSize', 11);
    
    title('Transient Response Superimposition (Mimuro Plot)', 'FontSize', 14, 'FontWeight', 'bold', 'Units', 'normalized', 'Position', [0.5, 1.08, 0]);
    
    if ispc; desk_path = fullfile(getenv('USERPROFILE'), 'Desktop'); else; desk_path = fullfile(getenv('HOME'), 'Desktop'); end
    out_dir = fullfile(desk_path, 'Lat_Dyn', 'MIM');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    
    exportgraphics(fig_mimuro, fullfile(out_dir, 'Mimuro_Comparison_Plot.jpg'), 'Resolution', 300, 'BackgroundColor', 'w');
    %close(fig_mimuro);
    disp('   [Mimuro Plot Export Complete]');
else
    disp('   [WARNING: Mimuro_Data.mat not found. Ensure parameter tests ran successfully.]');
end

%% ========================================================================
% 6. STEP STEER (STR) EXECUTION & EXPORT
% =========================================================================
disp('>>> INITIATING STEP STEER (STR) TEST <<<');
run('STR_master.m');
disp('-> Exporting STR Dashboards to Desktop...');
if ispc; desk_path = fullfile(getenv('USERPROFILE'), 'Desktop'); else; desk_path = fullfile(getenv('HOME'), 'Desktop'); end
out_dir = fullfile(desk_path, 'Lat_Dyn', 'STR');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
figs = findall(0, 'Type', 'figure');
for i = 1:length(figs)
    fig_name = figs(i).Name;
    if isempty(fig_name); fig_name = sprintf('Figure_%d', figs(i).Number); end
    safe_name = regexprep(fig_name, '[^a-zA-Z0-9]', '_');
    safe_name = regexprep(safe_name, '_+', '_');
    saveas(figs(i), fullfile(out_dir, [safe_name, '.jpg']), 'jpeg');
end
close all;

%% ========================================================================
% 7. SWEPT SINE (SWS) EXECUTION & EXPORT
% =========================================================================
disp('>>> INITIATING SWEPT SINE (SWS) TEST <<<');
run('SWS_master.m');
disp('-> Exporting SWS Dashboards to Desktop...');
if ispc; desk_path = fullfile(getenv('USERPROFILE'), 'Desktop'); else; desk_path = fullfile(getenv('HOME'), 'Desktop'); end
out_dir = fullfile(desk_path, 'Lat_Dyn', 'SWS');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
figs = findall(0, 'Type', 'figure');
for i = 1:length(figs)
    fig_name = figs(i).Name;
    if isempty(fig_name); fig_name = sprintf('Figure_%d', figs(i).Number); end
    safe_name = regexprep(fig_name, '[^a-zA-Z0-9]', '_');
    safe_name = regexprep(safe_name, '_+', '_');
    saveas(figs(i), fullfile(out_dir, [safe_name, '.jpg']), 'jpeg');
end
close all;

%% ========================================================================
% 8. CIRCULAR ACCELERATION (CAL) EXECUTION & EXPORT
% =========================================================================
disp('>>> INITIATING CIRCULAR ACCELERATION (CAL) TEST <<<');
run('CAL_master.m');
disp('-> Exporting CAL Dashboards to Desktop...');
if ispc; desk_path = fullfile(getenv('USERPROFILE'), 'Desktop'); else; desk_path = fullfile(getenv('HOME'), 'Desktop'); end
out_dir = fullfile(desk_path, 'Lat_Dyn', 'CAL');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
figs = findall(0, 'Type', 'figure');
for i = 1:length(figs)
    fig_name = figs(i).Name;
    if isempty(fig_name); fig_name = sprintf('Figure_%d', figs(i).Number); end
    safe_name = regexprep(fig_name, '[^a-zA-Z0-9]', '_');
    safe_name = regexprep(safe_name, '_+', '_');
    saveas(figs(i), fullfile(out_dir, [safe_name, '.jpg']), 'jpeg');
end
close all;

%% ========================================================================
% 9. SLALOM (SLA) EXECUTION & EXPORT
% =========================================================================
disp('>>> INITIATING TRANSIENT SLALOM (SLA) TEST <<<');
run('SLA_master.m');
disp('-> Exporting SLA Dashboards to Desktop...');
if ispc; desk_path = fullfile(getenv('USERPROFILE'), 'Desktop'); else; desk_path = fullfile(getenv('HOME'), 'Desktop'); end
out_dir = fullfile(desk_path, 'Lat_Dyn', 'SLA');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
figs = findall(0, 'Type', 'figure');
for i = 1:length(figs)
    fig_name = figs(i).Name;
    if isempty(fig_name); fig_name = sprintf('Figure_%d', figs(i).Number); end
    safe_name = regexprep(fig_name, '[^a-zA-Z0-9]', '_');
    safe_name = regexprep(safe_name, '_+', '_');
    saveas(figs(i), fullfile(out_dir, [safe_name, '.jpg']), 'jpeg');
end
close all;

%% ========================================================================
% 10. DUAL LANE CHANGE (DLC) EXECUTION & EXPORT
% =========================================================================
disp('>>> INITIATING DUAL LANE CHANGE (DLC) TEST <<<');
run('DLC_master.m');
disp('-> Exporting DLC Dashboards to Desktop...');
if ispc; desk_path = fullfile(getenv('USERPROFILE'), 'Desktop'); else; desk_path = fullfile(getenv('HOME'), 'Desktop'); end
out_dir = fullfile(desk_path, 'Lat_Dyn', 'DLC');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
figs = findall(0, 'Type', 'figure');
for i = 1:length(figs)
    fig_name = figs(i).Name;
    if isempty(fig_name); fig_name = sprintf('Figure_%d', figs(i).Number); end
    safe_name = regexprep(fig_name, '[^a-zA-Z0-9]', '_');
    safe_name = regexprep(safe_name, '_+', '_');
    saveas(figs(i), fullfile(out_dir, [safe_name, '.jpg']), 'jpeg');
end
close all;

%% ========================================================================
% 11. MU-SPLIT (MUS) EXECUTION & EXPORT
% =========================================================================
disp('>>> INITIATING MU-SPLIT (MUS) DISTURBANCE TEST <<<');
run('MUS_master.m');
disp('-> Exporting MUS Dashboards to Desktop...');
if ispc; desk_path = fullfile(getenv('USERPROFILE'), 'Desktop'); else; desk_path = fullfile(getenv('HOME'), 'Desktop'); end
out_dir = fullfile(desk_path, 'Lat_Dyn', 'MUS');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
figs = findall(0, 'Type', 'figure');
for i = 1:length(figs)
    fig_name = figs(i).Name;
    if isempty(fig_name)
        fig_name = sprintf('Figure_%d', figs(i).Number);
    end
    safe_name = regexprep(fig_name, '[^a-zA-Z0-9]', '_');
    safe_name = regexprep(safe_name, '_+', '_');
    
    save_path = fullfile(out_dir, [safe_name, '.jpg']);
    saveas(figs(i), save_path, 'jpeg');
end
close all;
disp('   [MUS Export Complete]');

%% ========================================================================
disp('====================================================');
disp('  ALL TESTS COMPLETE AND TELEMETRY EXPORTED SAFELY  ');
disp('====================================================');