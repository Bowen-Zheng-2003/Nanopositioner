% Parameters (from Table 1)
l2 = 12;    % mm
l3 = 6;     % mm
R = 0.6;    % mm
e = 6;      % mm
m = 1.9;    % mm
E = 196.6e3;% MPa (converted from GPa)
G = 78.6e3; % MPa (converted from GPa)
d1 = 10;    % microns

F = 4;      % Arbitrary force (will cancel out)

% Define the integrand functions as anonymous functions
f1 = @(theta) (sin(theta).^2 .* cos(theta)) ./ (m - 2*R*cos(theta)).^3;
f2 = @(theta) cos(theta) ./ (m - 2*R*cos(theta)).^3;
f3 = @(theta) cos(theta) ./ (m - 2*R*cos(theta));

% Numerical integration over [-pi/2, pi/2]
I1 = integral(f1, -pi/2, pi/2, 'RelTol',1e-10,'AbsTol',1e-12);
I2 = integral(f2, -pi/2, pi/2, 'RelTol',1e-10,'AbsTol',1e-12);
I3 = integral(f3, -pi/2, pi/2, 'RelTol',1e-10,'AbsTol',1e-12);

% Angular deformation (gamma_z)
gamma_z = (12*F*R*(l3 + R))/(E*e) * I2;

% Linear deformation (Delta_x)
Delta_x = (12*F*R^3)/(E*e) * I1 + (12*F*R^2*(l3 + R))/(E*e) * I2 + (F*R)/(G*e) * I3;

% Coordinates and displacements
cx1 = (l2 + l3) * gamma_z;
bx1 = l3 * gamma_z;
cx = cx1 + Delta_x;
bx = bx1 + Delta_x;

% Amplification ratio
Ra = cx / bx;
d2 = d1 * Ra;

% Display result
fprintf('Amplification Ratio Ra = %.3f\n', Ra);
fprintf('Output Displacement d2 = %.3f\n', d2);

% Source: 
% https://www.sciencedirect.com/science/article/abs/pii/S0888327020302673?via%3Dihub

