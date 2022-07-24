function [cx,cy] = circlePoints(x,y,R,N)

theta = linspace(-pi,pi,N);
cx = R*cos(theta) + x;
cy = R*sin(theta) + y;

end