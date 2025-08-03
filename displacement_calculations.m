% Parameters (from Table 1)
l2 = 7;    % mm
l3 = 3;     % mm
R = 0.5;    % mm
e = 6.35;      % mm
m = 2;    % mm
E = 205e3;% MPa (converted from GPa)
G = 80e3; % MPa (converted from GPa)
d1 = 9.1;    % microns

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
fprintf('Output Displacement d2 = %.3f microns\n', d2);

