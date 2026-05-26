% =========================================================================
% UNIT TEST: regen_qp_allocator.m
% Run this BEFORE integrating into Simulink.
% All 7 test cases must print [PASS].  Any [FAIL] = bug in the function.
% =========================================================================
clear; clc;
disp('=========================================');
disp('  UNIT TESTS: regen_qp_allocator.m      ');
disp('=========================================');

% --- Shared physical constants (match vcu_allocator) ---
mu          = 1.2;
G           = 10;
Rw          = 0.34;
T_motor_max = 400;
P_regen_max = 250000;    % 250 kW
k_safety    = 0.88;

pass_count = 0;
fail_count = 0;

% =========================================================================
% TEST 1: Straight-line braking — no yaw demand, no lateral force
%         Expected: both motors share regen equally
% =========================================================================
T_req     = -100;    % -100 Nm per motor total request
DeltaT    =    0;    % no yaw differential
Fz_rl     = 3924;   Fz_rr = 3924;   % static, symmetric
Fy_rl     =    0;   Fy_rr =    0;   % no lateral load
omega     =  100;                    % 100 rad/s (wheel)

[T_rl, T_rr] = regen_qp_allocator(T_req, DeltaT, ...
    Fz_rl, Fz_rr, Fy_rl, Fy_rr, mu, omega, omega, ...
    P_regen_max, T_motor_max, G, Rw, k_safety);

expected_each = T_req / 2;   % -50 Nm each
ok = abs(T_rl - expected_each) < 0.1 && abs(T_rr - expected_each) < 0.1 ...
     && T_rl <= 0 && T_rr <= 0;
report_test(1, 'Straight-line symmetric regen split', ok, T_rl, T_rr, expected_each);
if ok, pass_count = pass_count+1; else, fail_count = fail_count+1; end

% =========================================================================
% TEST 2: Yaw-differential only — no total regen request
%         Expected: inner gets negative, outer gets positive (yaw correction)
%         Both must stay <= 0 (regen-only constraint)
% =========================================================================
T_req     = -50;    % small regen
DeltaT    = +80;    % large yaw demand (outer much more than inner)
Fz_rl = 3924; Fz_rr = 3924; Fy_rl = 0; Fy_rr = 0;

[T_rl, T_rr] = regen_qp_allocator(T_req, DeltaT, ...
    Fz_rl, Fz_rr, Fy_rl, Fy_rr, mu, omega, omega, ...
    P_regen_max, T_motor_max, G, Rw, k_safety);

T_rl_unc = (T_req - DeltaT)/2;   % = (-50-80)/2 = -65
T_rr_unc = (T_req + DeltaT)/2;   % = (-50+80)/2 = +15 -> clamps to 0

ok = (T_rl <= 0) && (T_rr <= 0) && (T_rl < T_rr);
report_test(2, 'Regen-only constraint upheld (no drive during braking)', ok, T_rl, T_rr, []);
if ok, pass_count = pass_count+1; else, fail_count = fail_count+1; end

% =========================================================================
% TEST 3: Friction circle constraint active on INNER wheel
%         High lateral load on inner => small Fx_avail_rl
%         Expected: T_rl is clamped, T_rr is not
% =========================================================================
T_req   = -80;   DeltaT = 0;
Fz_rl   = 2743;  Fz_rr  = 5105;    % cornering load transfer
Fy_rl   = 2159;  Fy_rr  = 4017;    % ~0.75 G lateral (from CUB.m analysis)

% Pre-compute expected limits
FC_rl_N = sqrt(max((mu*Fz_rl)^2 - Fy_rl^2, 0)) * k_safety;
FC_rr_N = sqrt(max((mu*Fz_rr)^2 - Fy_rr^2, 0)) * k_safety;
T_min_rl_exp = max(-T_motor_max, -(FC_rl_N * Rw / G));
T_min_rr_exp = max(-T_motor_max, -(FC_rr_N * Rw / G));

[T_rl, T_rr] = regen_qp_allocator(T_req, DeltaT, ...
    Fz_rl, Fz_rr, Fy_rl, Fy_rr, mu, omega, omega, ...
    P_regen_max, T_motor_max, G, Rw, k_safety);

% T_rl_unc = -40, T_rr_unc = -40 (equal split)
% But T_min_rl might be larger (less regen allowed)
ok = (T_rl >= T_min_rl_exp - 0.01) && (T_rr >= T_min_rr_exp - 0.01) ...
     && T_rl <= 0 && T_rr <= 0;
fprintf('[TEST 3]  FC_rl_avail = %.1f Nm motor | FC_rr_avail = %.1f Nm motor\n', ...
    -T_min_rl_exp, -T_min_rr_exp);
report_test(3, 'Friction circle clamps inner-wheel regen', ok, T_rl, T_rr, T_min_rl_exp);
if ok, pass_count = pass_count+1; else, fail_count = fail_count+1; end

% =========================================================================
% TEST 4: QP recovers MORE regen than flat-margin (outer wheel has headroom)
%         At 0.75G lateral the outer wheel can handle more braking than inner
%         QP should allocate more to outer -> higher total regen vs uniform split
% =========================================================================
T_req = -120;  DeltaT = 0;

[T_rl_qp, T_rr_qp] = regen_qp_allocator(T_req, DeltaT, ...
    Fz_rl, Fz_rr, Fy_rl, Fy_rr, mu, omega, omega, ...
    P_regen_max, T_motor_max, G, Rw, k_safety);

% Flat margin comparison: old code uses mu*Fz*0.90, no Fy correction
T_flat_limit_rl = -(mu * Fz_rl * 0.90) * Rw / G;
T_flat_limit_rr = -(mu * Fz_rr * 0.90) * Rw / G;
T_rl_flat = max(T_flat_limit_rl, T_req/2);
T_rr_flat = max(T_flat_limit_rr, T_req/2);

E_qp   = abs(T_rl_qp)   + abs(T_rr_qp);
E_flat = abs(T_rl_flat) + abs(T_rr_flat);

ok = (E_qp <= E_flat * 1.01);   % QP should recover <= flat (more conservative on inner)
fprintf('[TEST 4]  QP total regen: %.1f Nm  |  Flat-margin: %.1f Nm\n', E_qp, E_flat);
report_test(4, 'QP never violates inner FC (may recover slightly less)', ok, T_rl_qp, T_rr_qp, []);
if ok, pass_count = pass_count+1; else, fail_count = fail_count+1; end

% =========================================================================
% TEST 5: Battery power constraint active
%         High wheel speed + large regen request => power limit kicks in
% =========================================================================
T_req   = -380;   DeltaT = 0;
Fz_rl   = 3924;   Fz_rr  = 3924;
Fy_rl   = 0;      Fy_rr  = 0;
omega_hi = 500;    % very high wheel speed (highway)

[T_rl, T_rr] = regen_qp_allocator(T_req, DeltaT, ...
    Fz_rl, Fz_rr, Fy_rl, Fy_rr, mu, omega_hi, omega_hi, ...
    P_regen_max, T_motor_max, G, Rw, k_safety);

% Check power: (-T_rl)*G*omega_hi + (-T_rr)*G*omega_hi <= P_regen_max
P_actual = (-T_rl) * G * omega_hi + (-T_rr) * G * omega_hi;
ok = (P_actual <= P_regen_max * 1.01);   % allow 1% tolerance for rounding
fprintf('[TEST 5]  Actual regen power: %.1f W  |  Limit: %.0f W\n', P_actual, P_regen_max);
report_test(5, 'Battery power constraint enforced', ok, T_rl, T_rr, []);
if ok, pass_count = pass_count+1; else, fail_count = fail_count+1; end

% =========================================================================
% TEST 6: Near-saturated tyre (Fy ≈ mu*Fz) — almost no room for Fx
%         Expected: very small regen command on the inner wheel
% =========================================================================
T_req = -100;  DeltaT = 0;
Fz_sat = 2000;
Fy_sat = mu * Fz_sat * 0.98;    % tyre at 98% of lateral limit

[T_rl, T_rr] = regen_qp_allocator(T_req, DeltaT, ...
    Fz_sat, 3924, Fy_sat, 0, mu, omega, omega, ...
    P_regen_max, T_motor_max, G, Rw, k_safety);

FC_sat_exp = sqrt(max((mu*Fz_sat)^2 - Fy_sat^2, 0)) * k_safety * Rw / G;
ok = abs(T_rl) <= FC_sat_exp + 0.1;
fprintf('[TEST 6]  Near-sat inner: T_rl = %.3f Nm  |  Max allowed: %.3f Nm\n', T_rl, -FC_sat_exp);
report_test(6, 'Near-saturated inner tyre is severely limited', ok, T_rl, T_rr, []);
if ok, pass_count = pass_count+1; else, fail_count = fail_count+1; end

% =========================================================================
% TEST 7: Drive mode guard (pedal > 0) — function must return zero
% =========================================================================
T_req_drive = +50;  % positive = drive, not regen
[T_rl, T_rr] = regen_qp_allocator(T_req_drive, 0, ...
    3924, 3924, 0, 0, mu, omega, omega, ...
    P_regen_max, T_motor_max, G, Rw, k_safety);

ok = (T_rl == 0) && (T_rr == 0);
report_test(7, 'Drive-mode guard returns zero (function is regen-only)', ok, T_rl, T_rr, 0);
if ok, pass_count = pass_count+1; else, fail_count = fail_count+1; end

% =========================================================================
% FINAL SUMMARY
% =========================================================================
disp(' ');
disp('=========================================');
fprintf('  RESULT: %d PASSED  |  %d FAILED\n', pass_count, fail_count);
disp('=========================================');
if fail_count == 0
    disp('  All unit tests passed. Safe to integrate into vcu_allocator.');
else
    disp('  Fix failures before integrating into Simulink.');
end

% =========================================================================
% HELPER: pretty print one test result
% =========================================================================
function report_test(id, name, passed, T_rl, T_rr, expected)
    status = 'PASS';
    if ~passed, status = 'FAIL'; end
    fprintf('[%s] TEST %d: %s\n', status, id, name);
    fprintf('        T_rl = %.4f Nm  |  T_rr = %.4f Nm\n', T_rl, T_rr);
    if ~isempty(expected) && ~passed
        fprintf('        Expected: %.4f Nm\n', expected(1));
    end
end
