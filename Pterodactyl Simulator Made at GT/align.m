clear all
clc

vb = [0;0.6;0.8];
vi = [0 0 1;0 1 0;-1 0 0]*vb;

d = dot(vb,vi);
theta = acos(d);

w = cos(theta/2);

c = cross(vb,vi);
c = c/norm(c);
c = c*sin(theta/2);
x = c(1);
y = c(2);
z = c(3);

w^2 + x^2 +y^2 + z^2

Rxx = 1 - 2*(y^2 + z^2);
Rxy = 2*(x*y - z*w);
Rxz = 2*(x*z + y*w);

Ryx = 2*(x*y + z*w);
Ryy = 1 - 2*(x^2 + z^2);
Ryz = 2*(y*z - x*w );

Rzx = 2*(x*z - y*w );
Rzy = 2*(y*z + x*w );
Rzz = 1 - 2 *(x^2 + y^2);

R = [ 
    Rxx,    Rxy,    Rxz;
    Ryx,    Ryy,    Ryz;
    Rzx,    Rzy,    Rzz]

w2 = w^2;
x2 = x^2;
y2 = y^2;
z2 = z^2;
vi - R*vb
[w2+x2-y2-z2 2*(x*y-w*z) 2*(w*y+x*z);2*(x*y+w*z) w2-x2+y2-z2 2*(y*z-w*x);2*(x*z-w*y) 2*(w*x+y*z) w2-x2-y2+z2];