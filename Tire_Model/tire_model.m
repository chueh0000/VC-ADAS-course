% Define parameters
D = 0.8;  % Peak (Friction coefficient mu)
C = 2.45;  % Shape
B = 4;   % Stiffness
E = 0.95; % Curvature

% Define the Magic Formula function
magic_formula = @(x) D * sin(C * atan(B*x - E*(B*x - atan(B*x))));

% Create a range of slip (from 0 to 1, where 1 is 100% slip/locked)
slip_ratio = linspace(0, 1, 100); 

% Calculate friction coefficient (mu)
mu = magic_formula(slip_ratio);

% Plot the results
% figure;
plot(slip_ratio, mu, 'LineWidth', 2);
grid on;
title('Pacejka Magic Formula');  % Friction vs. Slip
xlabel('\kappa');  % Wheel Slip
ylabel('\mu');  % Braking Coefficient