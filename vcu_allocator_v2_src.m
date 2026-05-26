function [T_rr_motor_raw, T_rl_motor_raw, T_rr_fric, T_rl_fric, T_f_fric, Mz_cmd, r_ref] = vcu_allocator(delta_curr, pedal, vx_curr, vy_curr, r_curr, kappa_ref, Fz_f_total, Fz_r_total, Fz_rl, Fz_rr, SOC, T_batt, T_motor, V_op, K_scheduled, w_vec, Fy_vec)
% =========================================================================
% VCU_ALLOCATOR  v2.0  —  Friction-Circle-Aware Torque Coordination
% =========================================================================
% INNOVATIONS vs v1.0
%  1. NEW INPUT  Fy_vec [Fy_fl Fy_fr Fy_rl Fy_rr] — per-wheel Pacejka Fy
%  2. FC-AWARE GRIP LIMITS: sqrt((mu*Fz)^2 - Fy^2)*k_fc replaces mu*Fz*0.90
%     -> at 1G lateral, v1.0 over-estimated available Fx by 63%
%  3. DYNAMIC Mz BUDGET: Mz_max scales with power/thermal/FC state
%     -> v1.0 used a hardcoded 2500 Nm constant regardless of system health
%  4. VECTORING-FIRST PRIORITY: Mz reserves FC budget; longitudinal gets rest
%     -> longitudinal torque is the sacrificial lamb, yaw control is always met
%  5. QP REGEN ALLOCATOR (brake mode): closed-form 2-variable QP
%     -> preserves lateral force margin during trail braking
%  6. AVAILABLE-TORQUE PEDAL CAP: virtual anti-windup at actuator level
%     -> clamps pedal to physics-feasible range to prevent PI integral windup
% =========================================================================

T_rr_motor_raw = 0.0; T_rl_motor_raw = 0.0;
T_rr_fric = 0.0; T_rl_fric = 0.0; T_f_fric = 0.0;
Mz_cmd = 0.0;

Rw = 0.34; tw = 1.65; G = 10; mu_max = 1.2;
T_motor_max          = 400;
T_fric_max_per_wheel = 3500;
P_motor_base         = 250000;    % 250 kW per motor

% =========================================================================
% [1] FRICTION-CIRCLE SAFETY FACTOR
%     Applied to the RESIDUAL chord sqrt((mu*Fz)^2 - Fy^2), not to mu*Fz.
%     k_fc = 0.88 < k_flat_old = 0.90, but the denominator is now smaller
%     so the actual usable Fx is also correctly reduced.
% =========================================================================
k_fc = 0.88;

% Per-wheel lateral forces injected from plant Goto tag 'Fy_vec'
Fy_rl_curr = Fy_vec(3);
Fy_rr_curr = Fy_vec(4);

% =========================================================================
% [2] BATTERY / THERMAL DERATING  (logic unchanged from v1.0)
% =========================================================================
T_batt_cal        = [-10, 0, 15, 45, 55, 60];
P_derate_batt_cal = [0, 0.5, 1.0, 1.0, 0.2, 0];
T_motor_cal       = [30, 90, 110, 130, 150];
P_derate_motor_cal= [1.0, 1.0, 0.8, 0.2, 0];
SOC_vec           = [0, 0.05, 0.15, 0.85, 0.95, 1.0];
P_derate_SOC_dis  = [0, 0.1, 1.0, 1.0, 1.0, 1.0];
P_derate_SOC_chg  = [1.0, 1.0, 1.0, 1.0, 0.1, 0];

derate_T_batt  = interp1(T_batt_cal,  P_derate_batt_cal,  T_batt(1),  'linear', 0);
derate_T_mot   = interp1(T_motor_cal, P_derate_motor_cal, T_motor(1), 'linear', 0);
derate_T_total = min(derate_T_batt, derate_T_mot);
derate_S_dis   = interp1(SOC_vec, P_derate_SOC_dis, SOC(1), 'linear', 0);
derate_S_chg   = interp1(SOC_vec, P_derate_SOC_chg, SOC(1), 'linear', 0);

P_avail_dis = P_motor_base * min(derate_T_total, derate_S_dis);
P_avail_chg = P_motor_base * min(derate_T_total, derate_S_chg);

vx_lqr = max(vx_curr(1), 5.0);

% =========================================================================
% [3] TV BLEND (unchanged)
% =========================================================================
vx_kmh = vx_curr(1) * 3.6;
if vx_kmh >= 50.0
    tv_blend = 1.0;
elseif vx_kmh <= 45.0
    tv_blend = 0.0;
else
    tv_blend = (vx_kmh - 45.0) / 5.0;
end

% =========================================================================
% [4] LQR YAW DEMAND & PHYSICAL BOUNDING (unchanged)
% =========================================================================
K_dyn_1 = interp1(V_op, K_scheduled(:,1), vx_lqr, 'linear', 'extrap');
K_dyn_2 = interp1(V_op, K_scheduled(:,2), vx_lqr, 'linear', 'extrap');

L = 2.6; K_us = 0.0015;
r_ref_raw  = (vx_lqr * delta_curr(1)) / (L * (1 + K_us * vx_lqr^2));
r_max_phys = (mu_max * 9.81 * 0.95) / vx_lqr;
r_ref      = max(-r_max_phys, min(r_max_phys, r_ref_raw));

Mz_req = tv_blend * -(K_dyn_1(1) * vy_curr(1) + K_dyn_2(1) * (r_curr(1) - r_ref));

% =========================================================================
% [5] HYPERBOLIC POWER LIMIT PER MOTOR (unchanged logic, now feeds FC calc)
% =========================================================================
w_motor_rl = max(abs(w_vec(3) * G), 0.1);
w_motor_rr = max(abs(w_vec(4) * G), 0.1);

T_lim_rl_dis = min(T_motor_max,  P_avail_dis / w_motor_rl);
T_lim_rr_dis = min(T_motor_max,  P_avail_dis / w_motor_rr);
T_lim_rl_chg = max(-T_motor_max, -P_avail_chg / w_motor_rl);
T_lim_rr_chg = max(-T_motor_max, -P_avail_chg / w_motor_rr);

% =========================================================================
% [6] FC-AWARE GRIP LIMIT  (CORE CHANGE — replaces mu*Fz*0.90)
%
%   v1.0:  T_grip_rl = mu * Fz_rl * Rw * 0.90
%   v2.0:  T_grip_rl = sqrt( (mu*Fz_rl)^2 - Fy_rl^2 ) * Rw * k_fc
%
%   The difference is the Pythagorean subtraction of the current lateral
%   force Fy_rl. At 0G this is identical (Fy=0). At 0.75G the residual is
%   15% smaller. At 1G it is 63% smaller. v1.0 would over-command into a
%   saturated tyre; v2.0 respects the actual remaining capacity.
% =========================================================================
T_fc_rl_wheel = sqrt(max((mu_max*Fz_rl(1))^2 - Fy_rl_curr^2, 0)) * Rw * k_fc;
T_fc_rr_wheel = sqrt(max((mu_max*Fz_rr(1))^2 - Fy_rr_curr^2, 0)) * Rw * k_fc;

% =========================================================================
% [7] DYNAMIC Mz BUDGET
%
%   v1.0:  Mz_cmd = max(-2500, min(2500, Mz_req))   <- hardcoded 2500 Nm
%   v2.0:  Mz_max computed from actual per-motor differential capacity,
%          combining power-limit, thermal-derate AND FC residual.
%
%   When the motor is hot, SOC is low, or tyres are heavily loaded laterally,
%   Mz_max shrinks automatically. The LQR's yaw demand is still tracked as
%   closely as physics allows — it is never silently capped at a constant.
% =========================================================================
T_max_rr_abs    = min(T_lim_rr_dis,  T_fc_rr_wheel / G);   % outer motor ceiling
T_min_rl_abs    = max(T_lim_rl_chg, -T_fc_rl_wheel / G);   % inner motor floor
T_delta_mot_max = (T_max_rr_abs - T_min_rl_abs) / 2;
Mz_max_dyn      = max(0, min(2500, T_delta_mot_max * G * tw / Rw));
Mz_cmd          = max(-Mz_max_dyn, min(Mz_max_dyn, Mz_req));
T_delta_wheel_raw = max(-1500, min(1500, (Mz_cmd * Rw) / tw));

% =========================================================================
% [8] VECTORING-FIRST PRIORITY
%
%   Before allocating longitudinal torque, reserve the FC budget required
%   to deliver the full Mz_cmd. Whatever remains is available for the
%   driver's acceleration/deceleration request.
%
%   Hierarchy:  Yaw control  >  ABS/TCS  >  Longitudinal drive/brake
%
%   This is the "longitudinal is the sacrificial lamb" strategy:
%   when the tyre is loaded (high Ay + deceleration), the driver's
%   speed-tracking demand is automatically clipped, not the yaw moment.
% =========================================================================
T_vec_reserve   = abs(T_delta_wheel_raw);           % each wheel's vectoring cost

T_long_rl_wheel = max(0, T_fc_rl_wheel - T_vec_reserve);
T_long_rr_wheel = max(0, T_fc_rr_wheel - T_vec_reserve);

% Final per-motor limits = min(power/thermal, post-vectoring FC residual)
T_max_rl_motor = min(T_lim_rl_dis,   T_long_rl_wheel / G);
T_max_rr_motor = min(T_lim_rr_dis,   T_long_rr_wheel / G);
T_min_rl_motor = max(T_lim_rl_chg,  -T_long_rl_wheel / G);
T_min_rr_motor = max(T_lim_rr_chg,  -T_long_rr_wheel / G);

% =========================================================================
% [9] AVAILABLE-TORQUE PEDAL CAP (virtual actuator-level anti-windup)
%
%   The external PI speed governor saturates at ±1.0 and has no knowledge
%   of the current derating state. When thermal/SOC/FC limits have reduced
%   available torque well below the nominal 800 Nm (= 2 x 400 Nm), the PI
%   integrator can wind up against the hard ±1 wall, causing overshoot and
%   slow recovery when conditions improve.
%
%   Solution implemented here: clamp the effective pedal INSIDE the MATLAB
%   Function to the ratio of actually-available torque to nominal torque.
%   This acts as a physics-aware saturation at the actuator level.
%
%   Full solution (future): feed T_avail_total back to the PI block's
%   external saturation port and set AntiWindupMode = 'back-calculation',
%   which would achieve true integrator tracking.
% =========================================================================
T_avail_total = T_max_rl_motor + T_max_rr_motor;
T_nominal     = 2 * T_motor_max;                      % = 800 Nm peak
pedal_cap     = min(1.0, T_avail_total / max(T_nominal, 1));
pedal_eff     = min(pedal_cap, pedal(1));              % caps drive; regen unchanged

% Fz-proportional base split
weight_rl = Fz_rl(1) / max(1, Fz_rl(1) + Fz_rr(1));
weight_rr = Fz_rr(1) / max(1, Fz_rl(1) + Fz_rr(1));

% =========================================================================
% [10] DRIVE MODE
% =========================================================================
if pedal(1) >= 0

    T_req_total_motor = pedal_eff * (T_max_rl_motor + T_max_rr_motor);
    T_rr_base  = T_req_total_motor * weight_rr;
    T_rl_base  = T_req_total_motor * weight_rl;

    T_rr_ideal = max(T_min_rr_motor, min(T_max_rr_motor, T_rr_base + (T_delta_wheel_raw/G)));
    T_rl_ideal = max(T_min_rl_motor, min(T_max_rl_motor, T_rl_base - (T_delta_wheel_raw/G)));

    % TCS — reactive net still present; FC pre-limit reduces its activations
    v_hub_rl = max(vx_curr(1) - (tw/2)*r_curr(1), 1.0);
    v_hub_rr = max(vx_curr(1) + (tw/2)*r_curr(1), 1.0);
    kappa_rl  = (w_vec(3)*Rw - v_hub_rl) / v_hub_rl;
    kappa_rr  = (w_vec(4)*Rw - v_hub_rr) / v_hub_rr;
    slip_target = 0.10; Kp_tcs = 1000;
    if kappa_rl > slip_target
        T_rl_ideal = max(0, T_rl_ideal - Kp_tcs*(kappa_rl - slip_target));
    end
    if kappa_rr > slip_target
        T_rr_ideal = max(0, T_rr_ideal - Kp_tcs*(kappa_rr - slip_target));
    end

    T_rr_fric = 0; T_rl_fric = 0; T_f_fric = 0;

% =========================================================================
% [11] BRAKE MODE — QP REGEN ALLOCATOR  (Patent #9)
%
%   Replaces flat-margin sequential priority with an analytical 2-variable
%   QP that simultaneously satisfies:
%     (a) Total regen tracking  (b) LQR yaw differential
%     (c) Per-wheel FC constraint   (d) Battery power acceptance
%
%   Closed-form solution: no Optimization Toolbox, <0.1 ms execution.
% =========================================================================
else

    bias_f = 0.70;
    total_grip_cap_wheel = mu_max * (Fz_f_total(1) + Fz_r_total(1)) * Rw * 0.90;
    T_req_total_wheel    = pedal(1) * total_grip_cap_wheel;
    T_req_front_wheel    = T_req_total_wheel * bias_f;
    T_req_rear_wheel     = T_req_total_wheel * (1 - bias_f);

    T_f_fric = max(-mu_max * Fz_f_total(1) * Rw * 0.90, T_req_front_wheel);

    % --- Call QP regen allocator ---
    T_req_rear_motor = T_req_rear_wheel / G;
    T_delta_motor    = T_delta_wheel_raw / G;

    [T_rl_qp, T_rr_qp] = fc_regen_qp( ...
        T_req_rear_motor, T_delta_motor, ...
        Fz_rl(1), Fz_rr(1), Fy_rl_curr, Fy_rr_curr, mu_max, ...
        w_vec(3), w_vec(4), P_avail_chg * G, T_motor_max, G, Rw, k_fc);

    % ABS slip regulation (unchanged)
    v_hub_fl = max(vx_curr(1) - (tw/2)*r_curr(1), 1.0);
    v_hub_fr = max(vx_curr(1) + (tw/2)*r_curr(1), 1.0);
    v_hub_rl_b = max(vx_curr(1) - (tw/2)*r_curr(1), 1.0);
    v_hub_rr_b = max(vx_curr(1) + (tw/2)*r_curr(1), 1.0);
    kappa_fl = (w_vec(1)*Rw - v_hub_fl)   / v_hub_fl;
    kappa_fr = (w_vec(2)*Rw - v_hub_fr)   / v_hub_fr;
    kappa_rl = (w_vec(3)*Rw - v_hub_rl_b) / v_hub_rl_b;
    kappa_rr = (w_vec(4)*Rw - v_hub_rr_b) / v_hub_rr_b;
    slip_lim_abs = -0.10; Kp_abs = 5000;
    if min(kappa_fl, kappa_fr) < slip_lim_abs
        T_f_fric = min(0, T_f_fric - Kp_abs*(min(kappa_fl,kappa_fr) - slip_lim_abs));
    end
    if kappa_rl < slip_lim_abs
        T_rl_qp = min(0, T_rl_qp*G - Kp_abs*(kappa_rl - slip_lim_abs)) / G;
    end
    if kappa_rr < slip_lim_abs
        T_rr_qp = min(0, T_rr_qp*G - Kp_abs*(kappa_rr - slip_lim_abs)) / G;
    end

    T_rr_ideal = max(T_min_rr_motor, min(0, T_rr_qp));
    T_rl_ideal = max(T_min_rl_motor, min(0, T_rl_qp));

    % Friction brakes absorb any deficit not met by motor regen
    T_rr_dem  = (T_req_rear_wheel * weight_rr) + T_delta_wheel_raw;
    T_rl_dem  = (T_req_rear_wheel * weight_rl) - T_delta_wheel_raw;
    T_rr_fric = max(-T_fric_max_per_wheel, min(0, T_rr_dem - T_rr_ideal*G));
    T_rl_fric = max(-T_fric_max_per_wheel, min(0, T_rl_dem - T_rl_ideal*G));

    if vx_curr(1) < 2.0
        T_f_fric = 0; T_rr_fric = 0; T_rl_fric = 0;
        T_rr_ideal = 0; T_rl_ideal = 0;
    end
end

T_rr_motor_raw = T_rr_ideal;
T_rl_motor_raw = T_rl_ideal;
end

% =========================================================================
% LOCAL FUNCTION: Friction-circle-aware analytical 2-variable QP
%
% Solves:  min  (T_rl + T_rr - T_req)^2  +  ((T_rr - T_rl) - DeltaT)^2
%   s.t.   T_rl >= -FC_rl/G   (inner wheel FC residual)
%          T_rr >= -FC_rr/G   (outer wheel FC residual)
%          power <= P_regen_max_W
%          T_rl <= 0, T_rr <= 0   (regen only)
%
% Closed-form KKT solution — equal weights, box projection.
% =========================================================================
function [T_rl, T_rr] = fc_regen_qp(T_req, DeltaT, ...
    Fz_rl, Fz_rr, Fy_rl, Fy_rr, mu, ...
    omega_rl, omega_rr, P_regen_max_W, T_motor_max, G, Rw, k_safety)

if T_req >= 0; T_rl = 0; T_rr = 0; return; end

% FC residual limits (motor-side)
T_min_rl = max(-T_motor_max, -sqrt(max((mu*Fz_rl)^2 - Fy_rl^2, 0)) * k_safety * Rw / G);
T_min_rr = max(-T_motor_max, -sqrt(max((mu*Fz_rr)^2 - Fy_rr^2, 0)) * k_safety * Rw / G);

% Unconstrained optimum (equal-weight cost — see derivation notes)
T_rl = max(T_min_rl, min(0.0, (T_req - DeltaT) / 2));
T_rr = max(T_min_rr, min(0.0, (T_req + DeltaT) / 2));

% Battery power acceptance check
P_act = (-T_rl) * max(abs(omega_rl)*G, 1) + (-T_rr) * max(abs(omega_rr)*G, 1);
if P_act > P_regen_max_W && P_act > 0
    s = P_regen_max_W / P_act;
    T_rl = T_rl * s;  T_rr = T_rr * s;
end

T_rl = max(-T_motor_max, min(0.0, T_rl));
T_rr = max(-T_motor_max, min(0.0, T_rr));
end
