function [value, isterminal, direction] = two_link_event(~, x)
    th = x(1); phi = x(2);
    value = phi - 2*th;
    isterminal = 1;            
    direction = +1;             
end