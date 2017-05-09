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

% Motor plant numerator
G_nm = G_v .* K_T(i);

% Motor plant denominator
G_dm = [ ...
    (N .* J_eff .* L_m(i)), ...                       % s^3
    (N .* (R_m(i) .* J_eff + B_m .* L_m(i))), ...     % s^2
    (N .* (K_T(i).^2 + R_m(i) .* B_m)), ...           % s^1
    0];                                               % s^0

% Plant, motor
G_m = tf(G_nm, G_dm);
G_m = minreal(G_m);

% Position plant, numerator
G_nx = g .* K_s;

% Position plant, denominator
G_dx = [ ...
    A, ...            % s^2
    0, ...            % s^1
    0];               % s^0     

% Plant,position
G_x = tf(G_nx, G_dx);
G_x = minreal(G_x);

% Motor controller

% failed LAM design
%{
os = 0;
n = 6;
ts = .3;

[No, Do] = stepshape(n,os,ts);
[D, T_motor, Tu, Td, L] = lamdesign(G_m, Do);
stepinfo(T_motor)
%}


% Best motor controller design, trying it by hand
%{
[d, c] = pidtune(G_m, 'PDF');
L = G_m*d;
T_motor = feedback(L, 1);
stepinfo(T_motor)
figure();
rlocus(L);
%}

% Motor controller design
z = -75;
p = -85;

D = zpk(z, p, 1);
figure();
rlocus(G_m .* D);

k = 32.6;
D = k .* D;
T_motor = feedback(D .* G_m, 1);
figure();
step(T_motor);
info_m = stepinfo(T_motor);

% Combine motor controller and position (ball and track) plant
% to get plant of position controller
G_x2 = T_motor * G_x;
G_x2 = minreal(G_x2);

% Position controller design
figure();          
rlocus(G_x2);
title('Position Controller Plant');

% Two poles at origin, need a lead controller to bring them into the 
% RHP. This takes the form of 
%       (s + z)
% D = k --------
%       (s + p)
% where p's magnitude is greater than z's

p = -30;    % Bring p as far out left as possible without crossing 
            % the next pole at -32.17
            
z = 0;      % Need to cancel one of the zeros

figure();
D_x2 = zpk(z, p, 1);
rlocus(G_x2 .* D_x2);

% Let gain k = 1, as k increases the origin poles grow dangerously close
% to the RHP, we want them as close as far away as possible
T_position = feedback(D_x2 .* G_x2, 1);
figure();
step(T_position);
info_x = stepinfo(T_position);