% System constants
syms m l g real


% System variables
syms th real
syms dth real
syms d2th real

% Generalized Coordinates
q = [th];
% Generalized Velocitiy
dq = [dth];

% generalized acceleration
d2q = [d2th];

% Kinematics: Position
p = l * [sin(th); -cos(th)];

% Velocity
dp = jacobian(p, q) * dq;

% Kinetic Energy
T = 1/2 * dp' * m * dp;

% Potential Energy
U = m*g*p'*[0; 1];

% Lagrangian
L = T-U;

% Equations of Motion

% Define states
x = [q; dq];
dx = [dq; d2q];
LHS_EoM = jacobian(jacobian(L, dq), x)*dx - jacobian(L, q)
simplify(LHS_EoM)