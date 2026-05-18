% Clear workspace and command window
clear; clc; close all;

% Define the Laplace variable
s = tf('s');

% Define the Transfer Functions
% Friction Brake includes a 50 ms (0.05 seconds) transport delay
sys1 = tf(1, [0.115 1], 'InputDelay', 0.05); 
sys2 = tf(1, [0.005 1]);
sys3 = tf(1, [0.015 1]);
sys4 = tf(1, [0.05 1]);

% Group systems and assign the requested actuator names
systems = {sys1, sys2, sys3, sys4};
sys_names = {'Friction Brake (T=0.115s, Delay=50ms)', ...
             'Car A Motor (T=0.005s)', ...
             'Car B Motor (T=0.015s)', ...
             'Steering Actuator (T=0.050s)'};

% =========================================================
% 1. Frequency Response Analysis (1 Hz to 50 Hz)
% =========================================================
figure('Name', 'Frequency Response Analysis', 'Color', 'w');

% Define frequency range: 1 Hz to 50 Hz
f_min = 1;  % Hz
f_max = 50; % Hz
w = 2 * pi * logspace(log10(f_min), log10(f_max), 500); % rad/s

% Plot Bode
bode(sys1, sys2, sys3, sys4, w);
grid on;
legend(sys_names, 'Location', 'southwest');
title('Bode Plot (1 Hz to 50 Hz)');

% =========================================================
% 2. Step Response Analysis with Performance Parameters
% =========================================================
figure('Name', 'Step Response Analysis', 'Color', 'w');
hold on;

colors = lines(4); % Generate distinct colors for the plot
legend_text = cell(1, 4);

for i = 1:4
    sys = systems{i};
    
    % Simulate and plot step response
    [y, t] = step(sys);
    plot(t, y, 'Color', colors(i,:), 'LineWidth', 1.5);
    
    % Extract performance parameters (Rise Time, Settling Time, etc.)
    info = stepinfo(sys);
    
    % Format a string to note the parameters on the plot via the legend
    legend_text{i} = sprintf('%s | Tr: %.3fs | Ts: %.3fs', ...
                             sys_names{i}, info.RiseTime, info.SettlingTime);
end

% Formatting the step response plot
hold off;
grid on;
title('Actuator Step Response Comparison');
xlabel('Time (seconds)');
ylabel('Amplitude');

% Place the legend containing the performance parameters
legend(legend_text, 'Location', 'southeast', 'FontSize', 9);