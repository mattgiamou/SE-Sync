function [R] = random_orientation_matrix(theta)
%random_orientation_quaternion Sample a random quaternion with angle
%magnitude (in axis-angle form) of theta

t = rand()*2*pi;
z = rand()*2 - 1;
x = sqrt(1-z^2)*cos(t);
y = sqrt(1-z^2)*sin(t);

c = cos(theta);
s = sin(theta);

R = [c+x^2*(1-c) x*y*(1-c)-z*s x*z*(1-c)+y*s;
     y*x*(1-c)+z*s c+y^2*(1-c) y*z*(1-c)-x*s;
     z*x*(1-c)-y*s z*y*(1-c)+x*s c+z^2*(1-c)];
 
end

