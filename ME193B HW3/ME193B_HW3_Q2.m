%% VAN DER POL OSCILLATOR

%% QUESTION 1

mu = 1;          
time = [0 50];  

dynamics = @(t,x) [x(2); mu*(1 - x(1)^2)*x(2) - x(1)];

rng(1);
x0s = 10*rand(10,2) - 5; 

figure; hold on;
for i = 1:10
    [t, x] = ode45(dynamics, time, x0s(i,:));
    
    plot(x(:,1), x(:,2));
    plot(x(1,1), x(1,2), 'ko');
end

xlabel('x1'); ylabel('x2'); 
title('Van der Pol Oscillator');
grid on;

%% QUESTION 2

function x1 = VanderPolPoincare(x0)
    mu = 1;

    % change one of the coords of x0 to ensure x0 in S (Poincaré section)
    x0(1) = 0;
    if x0(2) <= 0
        x0(2) = abs(x0(2))+ eps; 
    end    
    
    dynamics = @(t,x) [x(2); mu*(1 - x(1)^2)*x(2) - x(1)];
    options = odeset('Events', @(t,x) crossings(t,x));
    
    [~,~,~,xe,~] = ode45(dynamics, [0 100], x0, options);

    x1 = xe(end,:)';
end

function [value, isterminal, direction] = crossings(t,x)
    value = x(1); %detect event when x(1) crosses 0
    isterminal = 1; % stop integration when event occurs
    direction = 1; %detect events whne going from -ve to +ve
end

%% QUESTION 3

% initial condition
x0 = [0; 2];

N = 20;
X = zeros(2,N+1);
X(:,1) = x0;

for k = 1:N
    X(:,k+1) = VanderPolPoincare(X(:,k));
end

for k = 1:N+1
    fprintf('%3d %8.5f %8.5f\n', k-1, X(1,k), X(2,k));
end

figure; 
plot(0:N, X(2,:), 'o-');
xlabel('Number of iterations'); ylabel('x_2 at S');
title('Convergence of Poincaré Iterations');
grid on;

%% QUESTION 4
delta = 0.01;
fixed = X(:, end);

e1 = [1; 0]; e2 = [0; 1];

pos_peturb1 = VanderPolPoincare(fixed + delta*e1);
neg_peturb1 = VanderPolPoincare(fixed - delta*e1);
pos_peturb2 = VanderPolPoincare(fixed + delta*e2);
neg_peturb2 = VanderPolPoincare(fixed - delta*e2);

A1 = (pos_peturb1 - neg_peturb1)/(2*delta);
A2 = (pos_peturb2 - neg_peturb2)/(2*delta);

J = [A1, A2]
evals = eig(J)