% Matthew Blanchard & Forrest Smith
% ECE 414
% Final Project

% ========== Variables ========== %
% Constants
K_s = 10;       % Touch sensor gain: 0.1V/cm = 10V/m
J_s = 1.4e-7;   % Angular sensor inertia
J_g = 6.2e-6;   % Gearbox inertia 
J_m = 5.0e-5;   % Motor inertia
J_T = 1.8e-3;   % Track inertia

B_m = 3.0e-6;   % Motor viscous friction
G_v = 5;        % Voltage amplifier
g = 9.8;        % Gravitational constant

% Selectables
i = 1;                                          % Motor selection
N = 25;                                         % Gearbox ratio, 10 - 50
N_span = linspace(10, 50, 41);
J_eff = J_m + J_g + (1./(N.^2)).*(J_T + J_s);   % Effective Inertia

% Motor Variables
K_T = [0.225, 0.175, 0.125, 0.275];  % Motor Torque Constant
R_m = [8, 6, 4, 12];                 % Motor Resistance
L_m = [25e-3, 16e-3, 7.5e-3, 32e-3]; % Motor Inductance

% Ball Constants
R_b = 10e-3;     % Ball radius
r_b = 6e-3;      % Distance from ball center to channel wall
A = 1 + ((2 .* (R_b.^2)) ./ (5 .* (r_b.^2)));

% ========================== %

% Generate the plant

% Plant numerator (position)


% Plant denominator, motor

G_nx = G_v*K_T(i);

G_dx = [ ...
    (N .* J_eff .* L_m(i)), ...                       % s^3
    (N .* (R_m(i) .* J_eff + B_m .* L_m(i))), ...     % s^2
    (N .* (K_T(i).^2 + R_m(i) .* B_m)), ...           % s^1
    0];                             % s^0

% Plant, motor
G_x = tf(G_nx, G_dx);
G_x = minreal(G_x);

% Plant numerator (angle)
G_na = g*K_s;

% Plant denominator (angle)
G_da = [ ...
    (1+A), ...        % s^2
    0, ...            % s^1
    0];               % s^0                                  
G_a = tf(G_na, G_da);

G_a = minreal(G_a);

% Motor controller

% failed LAM design
%{
os = 0;
n = 6;
ts = .3;

[No, Do] = stepshape(n,os,ts);
[D, T_motor, Tu, Td, L] = lamdesign(G_x, Do);
stepinfo(T_motor)
%}


% Best motor controller design, trying it by hand
%{
[d, c] = pidtune(G_x, 'PDF');
L = G_x*d;
T_motor = feedback(L, 1);
stepinfo(T_motor)
figure();
rlocus(L);
%}

z = -75;
p = -85;

D = zpk(z, p, 1);
figure();
rlocus(G_x*D);

k = 32.6;
D = k*D;
T_motor = feedback(D*G_x, 1);
figure();
step(T_motor);
stepinfo(T_motor)


