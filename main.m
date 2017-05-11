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
R_b = 10e-3;     % Ball radius
r_b = 6e-3;      % Distance from ball center to channel wall
A = 1 + ((2 .* (R_b.^2)) ./ (5 .* (r_b.^2)));

% Selectables
i = 4;                                          % Motor selection
N = 14;                                         % Gearbox ratio, 10 - 50
J_eff = J_m + J_g + (1./(N.^2)).*(J_T + J_s);   % Effective inertia (depends on N)

% Motor Variables
K_T = [0.225, 0.175, 0.125, 0.275];  % Motor Torque Constant
R_m = [8, 6, 4, 12];                 % Motor Resistance
L_m = [25e-3, 16e-3, 7.5e-3, 32e-3]; % Motor Inductance
% ========================== %

% Generate the plants

% ======== Motor Plant ============= %
% Numerator
G_nm = G_v .* K_T(i);

% Denominator
G_dm = [ ...
    (J_eff .* L_m(i)), ...                       % s^3
    ((R_m(i) .* J_eff + B_m .* L_m(i))), ...     % s^2
    ((K_T(i).^2 + R_m(i) .* B_m)), ...           % s^1
    0];                                               % s^0

% Plant minimum transfer function
G_m = tf(G_nm, G_dm);
G_m = minreal(G_m);
% ================================== %

% ======= Ball & Track Plant ======= %
% Numerator
G_nx = g .* K_s .* (1 ./ N);

% Denominator
G_dx = [ ...
    A, ...            % s^2
    0, ...            % s^1
    0];               % s^0     

% Plant minimum transfer function
G_x = tf(G_nx, G_dx);
G_x = minreal(G_x);
% ================================== %

% Controller Design

% ======== Motor Controller ======== %
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

% Motor controller design (PD)
figure('Name', 'Motor Plant Root Locus');          
rlocus(G_m);
title('Motor Plant Root Locus'); 

% There is a pole at the origin which must be pulled out by a nearby zero.
% The pole of the PD must be further in the LHP than the zero.
z = -50;     % Zero, close enough to origin pole to pull it
p = -180;    % Pole, slightly further right than the conjugate pole pair

% PID controller
D_m = zpk(z, p, 1);     
figure('Name', 'Motor Controller Root Locus');
rlocus(G_m .* D_m);  
title('Motor Controller Root Locus');

% Select gain of 5, pulls the origin pole out but ensures the conjugate
% pairs do not get too large of an angle (we don't want oscillations,
% stability over speed in this application)

k = 5;
T_motor = feedback(D_m .* G_m, k);

% Step response/stats for the motor control system
figure('Name', 'Motor Controller Step Response');
step(T_motor);
title('Motor Controller Step Response');
info_m = stepinfo(T_motor);

% ========= Position Controller ========= %
% Combine motor controller and position (ball and track) plant
% to get plant of position controller
G_x2 = T_motor * G_x;
G_x2 = minreal(G_x2);

% Position controller design
figure('Name', 'Position Plant Root Locus');          
rlocus(G_x2);
title('Position Plant Root Locus'); 

% Two poles at origin, need a lead controller to bring them into the 
% RHP. This takes the form of 
%       (s + z)
% D = k --------
%       (s + p)
% where p's magnitude is greater than z's

p = -20;    % Bring p as far out left as possible without crossing 
            % the next pole at -20.9
            
z = 0;      % Need to cancel one of the poles at the origin with a zero

% Lead controller (special case of PID)
figure('Name', 'Position Controller Root Locus');
D_x2 = zpk(z, p, 1);
rlocus(G_x2 .* D_x2);
title('Position Controller Root Locus');

% Let gain k = 97.9, the meeting point of the origin pole and the
% controller pole. This pulls the origin pole as far into the LHP as
% possible without introducing oscillations from complex conjugate poles
% (that leads to overshoot, which we can't afford)
k = 97.9;   
T_position = feedback(D_x2 .* G_x2, k);

% The position transfer function represents the entire system
T_sys = T_position;
[Tu_sys, Umax_sys] = controleffort(G_x2, T_sys);
info_x = stepinfo(T_sys);

figure('Name', 'Position Controller Step Response');
step(T_sys);
title('Position Controller Step Response');

figure('Name', 'Position Controller Control Effort');
step(Tu_sys);
title('Position Controller Control Effort');

% ========= Operation ========== %
t = linspace(0, 10, 1001);
x(1:200)    =  0   .* k;
x(201:400)  =  2   .* k;
x(401:600)  =  2.5 .* k;
x(601:800)  = -2.5 .* k;
x(801:1001) =  0   .* k;

y = lsim(T_sys, x, t);

figure('Name', 'System Operation With Varying Input');
[a, h1, h2] = plotyy(t, x, t, y);
set(a(1), 'YTick', linspace(-2.5.*k, 2.5.*k, 11));
set(a(1), 'YTickLabel', {'-2.5*k', '-2.0*k', '-1.5*k', ...
    '-1.0*k', '-0.5*k', '0*k', '0.5*k', '1.0*k', '1.5*k', '2.0*k', '2.5*k'});
set(a(1), 'YLim', [-3.*k, 3.*k]);

set(a(2), 'YTick', linspace(-2.5, 2.5, 11));
set(a(2), 'YTickLabel', {'-2.5', '-2.0', '-1.5', '-1.0', '-0.5', '0', ...
    '0.5', '1.0', '1.5', '2.0', '2.5'});
set(a(2), 'YLim', [-3, 3]);

grid 'on';
title('System Operation with Varying Input');
legend('Input', 'Output');
xlabel('Time (s)');
axes(a(1));
ylabel('Input Voltage (V)');
axes(a(2));
ylabel('Touch Sensor Output Voltage (V)');

% plot(t, x);
% yyaxis left;
% xlim([0, 10]);
% ylim([-2.5, 2.5]);
% yticks([-2.5 -2 -1.5 -1 -0.5 0 0.5 1 1.5 2 2.5]);
% yticklabels(['-2.5', '-2', '-1.5', '-1', '-0.5', '0',...
%                         '0.5', '1.0', '1.5', '2.0', '2.5']);
%                     
% yyaxis right;
% plot(t, y);
% ylim([-2.5.*k, 2.5.*k]);
% yticks([-2.5, -2, -1.5, -1, -0.5, 0, 0.5, 1.0, 1.5, 2.0, 2.5].*k);
% yticklabels(['-2.5*k', '-2*k', '-1.5*k', '-1*k', '-0.5*k', '0*k',...
%                         '0.5*k', '1.0*k', '1.5*k', '2.0*k', '2.5*k']);
% 
% 
% 
% 
% 
