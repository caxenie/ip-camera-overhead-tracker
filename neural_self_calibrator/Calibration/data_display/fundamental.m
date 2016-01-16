% fundamental matrix

clear
close all



% Frame size
FRAME_WIDTH = 780;
FRAME_HEIGHT = 582;

%Sensor size
SENSOR_WIDTH = 5.371e-3;
SENSOR_HEIGHT = 4.035e-3;

% Element size
sx = SENSOR_WIDTH/FRAME_WIDTH;
sy = SENSOR_HEIGHT/FRAME_HEIGHT;

% Focal length
f = 4e-3;
fx = f/sx;
fy = f/sy;

% Principal point
cx = FRAME_WIDTH/2;
cy = FRAME_HEIGHT/2;

alpha_h = 2*atan(SENSOR_WIDTH/(2*f))*180/pi
alpha_v = 2*atan(SENSOR_HEIGHT/(2*f))*180/pi

% Target distance
d = 0.1562;

% Camera matrix
K = [fx, 0, cx 0;
            0 fy cy 0;
            0 0 1 0];
            

%% create extrinsic and intrinsic transformation matrices



%% Right camera
%Rotation
alpha = 0.66;
beta = 0.4;
%Translation
tx = 3;
ty = 1.5;
tz = -2.5;

Tr = Roty(beta)*Rotx(-alpha)*Rotz(pi)*Roty(-pi/2)*transl(tx,ty,tz);

Kr = K * Tr;

%% Left camera
%Rotation
alpha = -0.46;
beta = 0.4;
%Translation
tx = 3;
ty = -1.5;
tz = -2.5;

Tl = Roty(beta)*Rotx(-alpha)*Rotz(pi)*Roty(-pi/2)*transl(tx,ty,tz);

Kl = K * Tl;


data_cam_r = [];
data_cam_l = [];

for z=0:0.1:2
	for y=-1.25:0.25:1.25 
		for x=-1:0.25:1 

			r = [x;y;z;1];
			
			r_cam_r = Kr * r;
			rx_cam_r = r_cam_r(2,:)./r_cam_r(3,:);
			ry_cam_r = r_cam_r(1,:)./r_cam_r(3,:);
			data_cam_r = [data_cam_r;[rx_cam_r,ry_cam_r]];

			r_cam_l = Kl * r;

			rx_cam_l = r_cam_l(2,:)./r_cam_l(3,:);
			ry_cam_l = r_cam_l(1,:)./r_cam_l(3,:);

			data_cam_l = [data_cam_l;[rx_cam_l,ry_cam_l]];				
		end
	end
end


save("-ascii","cam0_log.txt","data_cam_l");
save("-ascii","cam1_log.txt","data_cam_r");