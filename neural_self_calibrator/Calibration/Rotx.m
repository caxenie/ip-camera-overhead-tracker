function R = Rotx(phi)

R = [1 0 0 0;
    0 cos(phi) -sin(phi) 0;
    0 sin(phi) cos(phi) 0;
    0 0 0 1];