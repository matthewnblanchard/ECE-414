% Matthew Blanchard & Forrest Smith
% ECE 414
% Final Project

% ========== Variables ========== %
% Constants
K_s = 10;     % Touch sensor gain: 0.1V/cm = 10V/m
J_s = 1.4e-7;   % Angular sensor inertia
J_g = 6.2e-6;   % Gearbox inertia 
J_m = 5.0e-5;   % Motor inertia
J_T = 1.8e-3;   % Track inertia

B_m = 3.0e-6;   % Motor viscous friction
G_v = 5;        % Voltage amplifier
g = 9.8;        % Gravitational constant

% Selectables
N = 35;         % Gearbox ratio, 10 - 50
N_span = linspace(10, 50, 41);

% Motor Variables
J_eff = J_m + J_g + (1./(N.^2)).*(J_T + J_s);   % Effective Inertia
K_T = [0.225, 0.175, 0.125, 0.275];  % Motor Torque Constant
R_m = [8, 6, 4, 12];                 % Motor Resistance
L_m = [25e-3, 16e-3, 7.5e-3, 32e-3]; % Motor Inductance

% Ball Constants
R_b = 10e-2;     % Ball radius
r_b = 6e-3;      % Distance from ball center to channel wall
A = 1 + ((2 .* (R_b.^2)) ./ (5 .* r_b));

% ========================== %

% Generate the plant

N = 15;          % Selected Gearbox 

% Plant numerator (position)
G_nx = K_T(1);

% Plant denominator, motor
G_dx = [ ...
    (N .* J_eff .* L_m(1)), ...                    % s^3
    (N .* (R_m(1) .* J_eff + B_m .* L_m(1))), ...  % s^2
    (N .* (K_T(1).^2 + R_m(1) .* B_m)), ...        % s^1
    0];                                            % s^0
    
% Plant, motor
G_x = tf(G_nx, G_dx);

% Plant numerator (angle)
G_na = g*K_s;

% Plant denominator (angle)
G_da = [ ...
    (1+A), ...      % s^2
    0, ...            % s^1
    0];               % s^0                                  
G_a = tf(G_na, G_da);

%Motor controller


