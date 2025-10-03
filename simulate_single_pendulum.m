function simulate_single_pendulum()
    x0=[30*pi/180; 0];
    [t, x]= ode45(@pendulum_dynamics, [0, 10], x0)

    figure; plot(t, x(:, 1));
    grid on;
    xlabel('Time (s)'); ylabel('th (rad)')
end

function dx = pendulum_dynamics(t, x)
    th = x(1);
    dth = x(2);
    
    % System Constants
    m=1;
    g=9.81;
    l=1;

    %l*m*(d2th*l+g*sin(th))]
    u = 0;
    d2th = (u - m*l*g*sin(th)) / (m*l^2);
    
    dx = [dth; d2th];
end