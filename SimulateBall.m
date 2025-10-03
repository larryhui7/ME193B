function SimulateBall()

    x0 = [10; 0];
    Tspan = [0 10];
    options = odeset('Events',@ground_contact);
    
    for j = 1:5
        [t,x] = ode45(@ball_dynamics, Tspan, x0);
        x_preimpact = x(end, :)';
        x_postimpact = ImpactDynamics(x_preimpact);
        x0 = x_postimpact;

        figure; 
        plot(t, x(:, 1));
        grid on;
        title('Ball Position')
        xlabel('Time(s)')
        ylabel('m');
    end

    figure; 
    plot(t, x(:, 1));
    grid on;
    title('Ball Position')
    xlabel('Time(s)')
    ylabel('m');

end

function dx = ball_dynamics(t, x)

y = x(1);
dy = x(2);

g = 9.81;

dx = [dy; -g];
end

function [value, isterminal, direction] = ground_contact(t, x)
y = x(1);
dy = x(2);

value = y; %detect event when y crosses zero
isterminal = 1; % stop integration when event occurs
direction = -1; %detect events whne going from +ve to -ve

end