k  = 20000;   
m  = 80;      
l0 = 1;       
c  = 100;     
g  = 9.81;    
yd = 2;      

reqE = m*g*yd;
init = [5, 4, 3, 2, 1];
figure; hold on;
for i = 1:length(init)
    x0 = [init(i); 0]; t0 = 0;
    t_all = []; x_all = [];
    curr = 'flight';
    
    while t0 < tf
        if curr == "flight"
            options = odeset('Events', @(t,x) landing(t,x,m,k,c,g,l0,yd,reqE));
            [t,x,te,xe,ie] = ode45(@(t,x) flight_dynamics(t,x,m,k,c,g,l0,yd,reqE), [t0 20], x0, options);
            curr = 'stance';
        else
            options = odeset('Events', @(t,x) takeoff(t,x,m,k,c,g,l0,yd,reqE));
            [t,x,te,xe,ie] = ode45(@(t,x) stance_dynamics(t,x,m,k,c,g,l0,yd,reqE), [t0 20], x0, options);
            curr = 'flight';
        end

        t_all = [t_all; t]; x_all = [x_all; x];
        x0 = xe.';  t0 = te;  
    end

    plot(t_all, x_all(:,1),'DisplayName', sprintf('y0=%.1f m',init(i)));
end

yline(yd,'k--','DisplayName','Desired Height');
xlabel('Time (s)'); ylabel('Height (m)');
legend show; grid on;
title('Controlled Hopper');


function dx = flight_dynamics(~,x,~,~,~,g,~,~,~)
    y = x(1); v = x(2);
    dx = [v; -g];
end

function dx = stance_dynamics(~,x,m,k,c,g,l0,~,Ed)
    y = x(1); v = x(2);
    Fs = -k*(y-l0);
    Fc = c*v;

    E = 0.5*m*v^2 + 0.5*k*(y-l0)^2 + m*g*y;

    K = 1500; 
    if v > 0
        u = max(0, K*(Ed - E));
    else
        u = 0;
    end
    F = Fs - Fc + u - m*g;
    dx = [v; F/m];
end

function [value, isterminal, direction] = landing(~,x,~,~,~,~,l0,~,~)
    value = x(1) - l0;
    isterminal = 1;
    direction = -1; 
end

function [value, isterminal, direction] = takeoff(~,x,~,~,~,~,l0,~,~)
    value = x(1) - l0;
    isterminal = 1;
    direction = 1;
end
