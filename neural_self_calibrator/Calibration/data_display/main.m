clear
close all



% Frame size
FRAME_WIDTH = 780;
FRAME_HEIGHT = 582;

%Sensor size
SENSOR_WIDTH = 5.371e-3;
SENSOR_HEIGHT = 4.035e-3;

% Element size
sx = 8.3e-6;
sy = 8.3e-6;

% Focal length
f = 5e-3;
fx = f/sx;
fy = f/sy;

% Principal point
cx = FRAME_WIDTH/2;
cy = FRAME_HEIGHT/2;



% Camera matrix
K = [   
        fx, 0,  cx, 0;
        0,  fy, cy, 0;
        0,  0,  1,  0
    ];
            

%% create extrinsic and intrinsic transformation matrices

%% Right camera
%Rotation
alpha = 20 * pi/180;
beta = 40* pi/180;
%Translation
tx = 3;
ty = -1;
tz = 2.5;
% Extrinsic transformation matrix
Tr = Rotx(-beta)*Roty(-alpha)*Rotz(-pi/2)*Roty(-pi/2)*transl(tx,-ty,-tz);
% Full perspective projection matrix
Kr = K * Tr;

   
%% Left camera
% Rotation    
alpha = -20 * pi/180;
beta = 40* pi/180;
% Translation
tx = 3;
ty = 1;    
tz = 2.5;

Tl = Rotx(-beta)*Roty(-alpha)*Rotz(-pi/2)*Roty(-pi/2)*transl(tx,-ty,-tz);

Kl = K * Tl;



%%
%data creation

%point origins
data_cam_r = [];
data_cam_l = [];

% Target distance
d = 0.1562;

marker = [d/2 -d/2; 0 0; 0 0; 1 1];
for x=-1.2:0.06:1.2
    for y=-1.2:0.06:1.2
        for z=0:0.05:1
        	
            % marker is randomly rotated, otherwise network output degenerates to a plane	
            phi = 2*pi.*rand(1);
            gamma = 2*pi.*rand(1);
            
            % Transformation matrix to rotate marker and transform into world coordinates
            T_markerWorld = transl(x,y,z) * Roty(gamma) * Rotz(phi);
            
            
            %% Capture marker with cameras:
            
            % right cam:
            r_pxl = Kr * T_markerWorld * marker;
            u_cam_r = r_pxl(1,:) ./ r_pxl(3,:);
            v_cam_r = r_pxl(2,:) ./ r_pxl(3,:);
            
            data_cam_r = [data_cam_r;u_cam_r(1),v_cam_r(1),1];
            
            
            % left cam:
            r_pxl = Kl * T_markerWorld * marker;
            u_cam_l = r_pxl(1,:) ./ r_pxl(3,:);
            v_cam_l = r_pxl(2,:) ./ r_pxl(3,:);
        
            data_cam_l = [data_cam_l;u_cam_l(1),v_cam_l(1),1];
            
        end
    end
end

data_cam_l(:,1:2) = data_cam_l(:,1:2) + normrnd(0,0.5,size(data_cam_l(:,1:2)));
data_cam_r(:,1:2) = data_cam_r(:,1:2) + normrnd(0,0.5,size(data_cam_r(:,1:2)));

save("-ascii","cam0_log.txt","data_cam_r");
save("-ascii","cam1_log.txt","data_cam_l");


[F,e1,e2] = fundmatrix(data_cam_l',data_cam_r');
F./F(3,3) 

%[fg1,T1] = normalise2dpts(data_cam_l');
%[fg2,T2] = normalise2dpts(data_cam_r');



sz = size(data_cam_l);

e = 0;
for kl=1:sz(1)
    e = e + abs(data_cam_r(kl,:) * F * data_cam_l(kl,:)');
end
e = e / sz(1)


%{
figure(1),
plot(data_cam_l(:,1),data_cam_l(:,2),"x"), hold on;
axis([0 780 0 582]);
title("left camera");

figure(2),
plot(data_cam_r(:,1),data_cam_r(:,2),"x"); 
axis([0 780 0 582]);
title("right camera");


el = F * data_cam_l(100,:)';
x0 = 0;
y0 = -1/el(2) * (el(1)*x0 + el(3));
x1 = 779;
y1 = -1/el(2) * (el(1)*x1 + el(3));

figure,
plot(data_cam_l(100,1),data_cam_l(100,2),"x"),hold on;
axis([0 780 0 582]);
figure,
plot(data_cam_r(100,1),data_cam_r(100,2),"x"),hold on;
plot([x0,x1],[y0,y1]);
axis([0 780 0 582]);



figure,
for N=1:50:size(data_cam_r,1)
    el = F * data_cam_r(N,:)';
    x0 = 1;
    y0 = -1/el(2) * (el(1)*x0 + el(3));
    x1 = 780;
    y1 = -1/el(2) * (el(1)*x1 + el(3));


    plot(data_cam_l(N,1),data_cam_l(N,2),"x"),hold on;
    plot([x0,x1],[y0,y1]),hold off;
    axis([1 780 1 582]);
    pause(0.5);

end
%}

