% rel. coords 
syms x y q1 q2 q3 xdot ydot q1dot q2dot q3dot real

% generalized coords
q = [x y q1 q2 q3];

% generalized velocity
dq = [xdot ydot q1dot q2dot q3dot];

% constants
m_torso = 10;  % kg
m_leg = 5;     % kg
I_torso = 1;   % kg-m^2
I_leg = 0.5;   % kg-m^2
l_torso = 0.5; % m
l_leg = 1;     % m
g = 9.81;      % m/s^2

% Configuration 1
conf1 = [0.5, sqrt(3)/2, deg2rad(150), deg2rad(120), deg2rad(30)];

% Configuration 2
conf2 = [0.3420, 0.9397, deg2rad(170), deg2rad(20), deg2rad(30)];


%% Q1 PART A
p_stleg1 = [x; y] + (l_leg) * [sin(q1 + q3); cos(q1 + q3)];

Pst1 = double(subs(p_stleg1, q, conf1));
Pst2 = double(subs(p_stleg1, q, conf2));

%% Q1 PART B
J_st = jacobian(p_stleg1, q);

Jst1 = double(subs(J_st, q, conf1));
Jst2 = double(subs(J_st, q, conf2));

%% Q1 PART C
Jst_dot = sym(zeros(size(J_st))); 

for i = 1:length(q)
    Jst_dot = Jst_dot + diff(J_st, q(i)) * dq(i);
end

qdot1 = [-0.8049, -0.4430, 0.0938, 0.9150, 0.9298];
qdot2 = [-0.1225, -0.2369, 0.5310, 0.5904, 0.6263];

Jstdot1 = double(subs(Jst_dot, [q, dq], [conf1, qdot1]));
Jstdot2 = double(subs(Jst_dot, [q, dq], [conf2, qdot2]));

%% Q1 PART D
p_leg1 = [x; y] + (l_leg/2) * [sin(q1 + q3); cos(q1 + q3)];
p_leg2 = [x; y] + (l_leg/2) * [sin(q2 + q3); cos(q2 + q3)];
p_torso = [x; y] + (l_torso/2) * [sin(q3); cos(q3)];

P = [p_leg1 p_leg2 p_torso];

P1 = double(subs(P, q, conf1));
P2 = double(subs(P, q, conf2));

qdot1 = [-0.8049, -0.4430, 0.0938, 0.9150, 0.9298];
qdot2 = [-0.1225, -0.2369, 0.5310, 0.5904, 0.6263];

dP_leg1 = simplify(jacobian(p_leg1, q) * dq');
dP_leg2 = simplify(jacobian(p_leg2, q) * dq');
dP_torso = simplify(jacobian(p_torso, q) * dq');

dP = [dP_leg1 dP_leg2 dP_torso];

dP1 = double(subs(dP, [q, dq], [conf1, qdot1]));
dP2 = double(subs(dP, [q, dq], [conf2, qdot2]));

T_leg1 = 0.5 * m_leg * (dP_leg1' * dP_leg1) + 0.5 * I_leg * (q3dot + q1dot)^2;
T_leg2 = 0.5 * m_leg * (dP_leg2' * dP_leg2) + 0.5 * I_leg * (q3dot + q2dot)^2;
T_torso =  0.5 * m_torso * (dP_torso' * dP_torso) + 0.5 * I_torso * (q3dot)^2;

T = T_leg1 + T_leg2 + T_torso;

T1 = double(subs(T, [q, dq], [conf1, qdot1]));
T2 = double(subs(T, [q, dq], [conf2, qdot2]));

e2 = [0; 1];

U_leg1 = m_leg * g * (p_leg1' * e2);
U_leg2 = m_leg * g * (p_leg2' * e2);
U_torso = m_torso * g * (p_torso' * e2);

U = simplify(U_leg1 + U_leg2 + U_torso);

U1 = double(subs(U, [q, dq], [conf1, qdot1]));
U2 = double(subs(U, [q, dq], [conf2, qdot2]));

q_act = [q1; q2];

[D, C, G, B] = LagrangianDynamics(T, U, q', dq', q_act);

D1 = double(subs(D, q', conf1'));
D2 = double(subs(D, q', conf2'));

C1 = double(subs(C, [q'; dq'], [conf1'; qdot1']));
C2 = double(subs(C, [q'; dq'], [conf2'; qdot2']));

G1 = double(subs(G, q', conf1.'));
G2 = double(subs(G, q', conf2.'));

B1 = double(subs(B, q', conf1.'));  
B2 = double(subs(B, q', conf2.'));

% constrained dynamics

Cqdot1 = C1 * qdot1';
Cqdot2 = C2 * qdot2';

P1 = [D1, -Jst1'; Jst1, zeros(2,2)];
Q1 = [-Cqdot1 - G1; -Jstdot1 * qdot1'];

P2 = [D2, -Jst2'; Jst2, zeros(2,2)];
Q2 = [-Cqdot2 - G2; -Jstdot2 * qdot2'];

F_st1 = P1\Q1;
F_st2 = P2\Q2;

qddot1 = F_st1(1:5);
Fst1 = F_st1(6:7);

qddot2 = F_st2(1:5);
Fst2 = F_st2(6:7);

%% QUESTION 2

q_pre = [0.3827, 0.9239, 3.0107, 2.2253, 0.5236];
qdot_pre = [1.4782, -0.6123, 1.6, -1.6, 0];

p_swing = [x; y] + l_leg * [sin(q2 + q3); cos(q2 + q3)];

J_2 = double(subs(jacobian(p_swing, q), q, q_pre));
D_2 = double(subs(D, q', q_pre'));

P = [D_2, -J_2'; J_2, zeros(2,2)];
Q = [D_2 * qdot_pre'; zeros(2,1)];

sol = P\Q;

qdot_plus = sol(1:5);  
F_delta = sol(6:7);