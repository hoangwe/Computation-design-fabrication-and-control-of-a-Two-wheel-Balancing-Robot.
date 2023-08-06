function x = basic_02(a, b, c, d) 
    % a*cos(x) - b*sin(x) = c
    % b*cos(x) + a*sin(x) = d
    sin = (d - b*c/a) / (b^2/a + a);
    cos = (c + b*d/a) / (b^2/a + a);
    x = atan2(sin,cos);
end