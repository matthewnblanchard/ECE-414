% Matthew Blanchard & Forrest Smith
% ECE 414
% Final Project
% Motor Selection

% In order to select the motor, the transfer function for the motor plant
% for each possible motor was examined.

% Constants:
K_T = [0.225, 0.175, 0.125, 0.275];  % Motor Torque Constant
R_m = [8, 6, 4, 12];                 % Motor Resistance
L_m = [25e-3, 16e-3, 7.5e-3, 32e-3]; % Motor Inductance
B_m = 3.0e-6;                        % Motor viscous friction
G_v = 5;                             % Voltage amplifier

J_s = 1.4e-7;   % Angular sensor inertia
J_g = 6.2e-6;   % Gearbox inertia 
J_m = 5.0e-5;   % Motor inertia
J_T = 1.8e-3;   % Track inertia

figure('Name', 'Motor Plants Pole Zero Maps');
% ======== Motor Plant ============= %

for i = 1 : 4
    for N = 10 : 50
        
        J_eff = J_m + J_g + (1./(N.^2)).*(J_T + J_s);   % Effective inertia (depends on N)
        
        % Numerator
        G_nm = G_v .* K_T(i);

        % Denominator
        G_dm = [ ...
            (J_eff .* L_m(i)), ...                    % s^3
            ((R_m(i) .* J_eff + B_m .* L_m(i))), ...  % s^2
            ((K_T(i).^2 + R_m(i) .* B_m)), ...        % s^1
         0];                                          % s^0
    
        % Plant minimum transfer function
        G = tf(G_nm, G_dm);
        G_m(N) = minreal(G);
        
        % Save the desired plant
        if ((N == 13) && (i == 4))
            G_select = G_m(N);
        end
        
    end
    subplot(2, 2, i);
    pzmap(G_m(10), 'g', G_m(11:49), G_m(50), 'r');
    title(sprintf('Motor #%d', i));
    hold 'on';
end

% Choosing Motor #4 With N = 13
figure('Name', 'Motor #4, N = 13');
pzmap(G_select);
title('Motor #4, N = 13');
 