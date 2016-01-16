function T_extr = extr_trafo(cam)


if cam==1
    alpha = 0.8567;
    tx = 0;
    ty = 3;
    tz = -2.6;
elseif cam==2
    alpha = -0.8567;
    tx = 0;
    ty = -3;    
    tz = -2.6;
end

T_extr = Roty(-alpha)*Roty(pi)*Rotz(pi/2)*transl(tx,ty,tz);