x0 = [0.2065; 0.4130; -0.2052; -0.0172];

function x1 = TwoLinkPoincare(x0)
    % change one of the coords of x0 to ensure x0 is in S
    x0(1) = x0(2)/2; % theta = phi/2
    x0(1) = x0(1) + eps;

    options = odeset('Events', @(t, x) two_link_event(t, x));

    % can get rid of t cuz time invariant
    [~, x_final] = ode45(@(t,x) two_link_dynamics(t,x), [0, 10], x0, options);
    x1 = x_final(end, :);
end

delta = eps;
e1 = [1; 0; 0; 0];
e2 = [0; 1; 0; 0];
e3 = [0; 0; 1; 0];
e4 = [0; 0; 0; 1];

J = zeros(4,4);

pos_peturb1 = TwoLinkPoincare(x0 + delta*e1);
pos_peturb2 = TwoLinkPoincare(x0 + delta*e2);
pos_peturb3 = TwoLinkPoincare(x0 + delta*e3);
pos_peturb4 = TwoLinkPoincare(x0 + delta*e4);

neg_peturb1 = TwoLinkPoincare(x0 - delta*e1);
neg_peturb2 = TwoLinkPoincare(x0 - delta*e2);
neg_peturb3 = TwoLinkPoincare(x0 - delta*e3);
neg_peturb4 = TwoLinkPoincare(x0 - delta*e4);

J(:, 1) = (pos_peturb1 - neg_peturb1)/(2*delta);
J(:, 2) = (pos_peturb2 - neg_peturb2)/(2*delta);
J(:, 3) = (pos_peturb3 - neg_peturb3)/(2*delta);
J(:, 4) = (pos_peturb4 - neg_peturb4)/(2*delta);

J
eig(J)