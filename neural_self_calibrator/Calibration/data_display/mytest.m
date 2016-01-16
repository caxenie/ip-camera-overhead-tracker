clear 
close all

FRAME_WIDTH = 1280;
FRAME_HEIGHT = 960;

SENSOR_WIDTH = 5.371e-3;
SENSOR_HEIGHT = 4.035e-3;

sx = SENSOR_WIDTH/FRAME_WIDTH;
sy = SENSOR_HEIGHT/FRAME_HEIGHT;

f = 4.37e-3;
fx = f/sx;
fy = f/sy;
d = 0.05;

cx = FRAME_WIDTH/2;
cy = FRAME_HEIGHT/2;

cube = [1 1 1 1 -1 -1 -1 -1;
    -1 1 -1 1 -1 1 -1 1;
    0 0 1.5 1.5 0 0 1.5 1.5
    1 1 1 1 1 1 1 1];

floor = [-0.6 0.6 -0.6 0.6; 
        -6 -6 6 6; 
        0 0 0 0; 
        1 1 1 1];

%figure, plot3(cube(1,:),cube(2,:),cube(3,:),'x');
%axis([-1 1 -1 1 -1 1]);
%grid on;

T_intr = [fx, 0, cx 0;
            0 fy cy 0;
            0 0 1 0];


tx = 0;
ty = 3;
tz = -2.6;


alpha = 0.8567;
T_extr_cam1 = Roty(-alpha)*Roty(pi)*Rotz(pi/2)*transl(tx,ty,tz);   
T_extr_cam2 = Roty(alpha)*Roty(pi)*Rotz(pi/2)*transl(tx,-ty,tz); 
T_cam1 = T_intr * T_extr_cam1;
T_cam2 = T_intr * T_extr_cam2;

%%
%plot coordinate axes -> orthogonal cosy should be orthogonal again
ex = [0 1; 0 0; 0 0; 1 1];
ey = [0 0; 0 1; 0 0; 1 1];
ez = [0 0; 0 0; 0 1; 1 1];

ex_cam1 = T_extr_cam1 * ex;
ey_cam1 = T_extr_cam1 * ey;
ez_cam1 = T_extr_cam1 * ez;

%figure('Name','World Cosy Unit Vectors ins Camera Cosy')
%plot3(ex_cam1(1,:),ex_cam1(2,:),ex_cam1(3,:),'r'), hold on;
%plot3(ey_cam1(1,:),ey_cam1(2,:),ey_cam1(3,:),'g'), hold on;
%plot3(ez_cam1(1,:),ez_cam1(2,:),ez_cam1(3,:),'b'), hold on;
%grid on;
%xlabel('x');
%xlabel('y');
%zlabel('z');


%%
c_cam1 = T_cam1 * cube;
u_cam1 = c_cam1(1,:) ./ c_cam1(3,:);
v_cam1 = c_cam1(2,:) ./ c_cam1(3,:);
    
c_cam2 = T_cam2 * cube;
u_cam2 = c_cam2(1,:) ./ c_cam2(3,:);
v_cam2 = c_cam2(2,:) ./ c_cam2(3,:);


%figure('Name','Cam1'),
%plot(u_cam1,v_cam1,'x'), hold on;
%plot([u_cam1(1),u_cam1(2)],[v_cam1(1),v_cam1(2)],'k')
%plot([u_cam1(1),u_cam1(3)],[v_cam1(1),v_cam1(3)],'k')
%plot([u_cam1(3),u_cam1(4)],[v_cam1(3),v_cam1(4)],'k')
%plot([u_cam1(2),u_cam1(4)],[v_cam1(2),v_cam1(4)],'k')
%plot([u_cam1(5),u_cam1(6)],[v_cam1(5),v_cam1(6)],'k')
%plot([u_cam1(5),u_cam1(7)],[v_cam1(5),v_cam1(7)],'k')
%plot([u_cam1(7),u_cam1(8)],[v_cam1(7),v_cam1(8)],'k')
%plot([u_cam1(6),u_cam1(8)],[v_cam1(6),v_cam1(8)],'k')
%plot([u_cam1(1),u_cam1(5)],[v_cam1(1),v_cam1(5)],'k')
%plot([u_cam1(2),u_cam1(6)],[v_cam1(2),v_cam1(6)],'k')
%plot([u_cam1(3),u_cam1(7)],[v_cam1(3),v_cam1(7)],'k')
%plot([u_cam1(4),u_cam1(8)],[v_cam1(4),v_cam1(8)],'k')
%axis ij;
%axis([0 1280 0 960]);
%grid on;

%figure('Name','Cam2'),
%plot(u_cam2,v_cam2,'x'), hold on;
%plot([u_cam2(1),u_cam2(2)],[v_cam2(1),v_cam2(2)],'k')
%plot([u_cam2(1),u_cam2(3)],[v_cam2(1),v_cam2(3)],'k')
%plot([u_cam2(3),u_cam2(4)],[v_cam2(3),v_cam2(4)],'k')
%plot([u_cam2(2),u_cam2(4)],[v_cam2(2),v_cam2(4)],'k')
%plot([u_cam2(5),u_cam2(6)],[v_cam2(5),v_cam2(6)],'k')
%plot([u_cam2(5),u_cam2(7)],[v_cam2(5),v_cam2(7)],'k')
%plot([u_cam2(7),u_cam2(8)],[v_cam2(7),v_cam2(8)],'k')
%plot([u_cam2(6),u_cam2(8)],[v_cam2(6),v_cam2(8)],'k')
%plot([u_cam2(1),u_cam2(5)],[v_cam2(1),v_cam2(5)],'k')
%plot([u_cam2(2),u_cam2(6)],[v_cam2(2),v_cam2(6)],'k')
%plot([u_cam2(3),u_cam2(7)],[v_cam2(3),v_cam2(7)],'k')
%plot([u_cam2(4),u_cam2(8)],[v_cam2(4),v_cam2(8)],'k')
%axis ij;
%axis([0 1280 0 960]);
%grid on;



%create extrinsic and intrinsic transformation matrices
T_extr_cam1 = extr_trafo(1);
T_extr_cam2 = extr_trafo(2);
T_intr = [fx, 0, cx 0;
            0 fy cy 0;
            0 0 1 0];

        
data_cam1 = [];
data_cam2 = [];
%target distance of two markers


marker = [0.03 -0.03; 0 0; 0 0; 1 1];
%figure('Name','Marker in World Coordinates'),
%plot3(marker(1,:),marker(2,:),marker(3,:),'x');
%xlabel('x');
%ylabel('y');
%zlabel('z');
%grid on
T_markerWorld = transl(0.5,1,0) * Rotz(pi/2);
marker_world = T_markerWorld * marker;
%figure('name','marker in world coordinates')
%plot3(marker_world(1,:),marker_world(2,:),marker_world(3,:),'x');
%xlabel('x');
%ylabel('y');
%zlabel('z');
%grid on
r_pxl = T_intr * T_extr_cam1 * marker_world;
u_cam1 = r_pxl(1,:) ./ r_pxl(3,:);
v_cam1 = r_pxl(2,:) ./ r_pxl(3,:);
%figure('Name','Marker in Camera Coordinates');
%plot(u_cam1,v_cam1,'x');
%axis ij;
%axis([0 1280 0 960]);
%grid on
%xlabel('u');
%ylabel('v');
            
for x=-0.7:0.35:0.7
    for y=-0.7:0.35:0.7
        for z=0:0.2:1.2
            phi = pi.*rand(1);
            gamma = pi.*rand(1);
            
            %T_markerWorld =  Roty(gamma) * Rotz(phi) * transl(x,y,z);
            T_markerWorld = transl(x,y,z); %* Roty(gamma) * Rotz(phi);
            
            %cam 1:
            r_pxl = T_intr * T_extr_cam1 * T_markerWorld * marker;
            u_cam1 = r_pxl(1,:) ./ r_pxl(3,:);
            v_cam1 = r_pxl(2,:) ./ r_pxl(3,:);

            data_cam1 = [data_cam1;u_cam1(1),v_cam1(1),u_cam1(2),v_cam1(2)];
            
            
            %cam 2:
            r_pxl = T_intr * T_extr_cam2 * T_markerWorld * marker;
            u_cam2 = r_pxl(1,:) ./ r_pxl(3,:);
            v_cam2 = r_pxl(2,:) ./ r_pxl(3,:);
            data = [u_cam2;v_cam2];
            data_cam2 = [data_cam2;u_cam2(1),v_cam2(1),u_cam2(2),v_cam2(2)];
            
        end
    end
end

figure('name','data points cam1'),
plot(data_cam1(:,1),data_cam1(:,2),'x'),hold on;
plot(data_cam1(:,3),data_cam1(:,4),'x');
axis([0 1280 0 960]);
axis ij
figure('name','data points cam2'),
plot(data_cam2(:,1),data_cam2(:,2),'x'), hold on;
plot(data_cam2(:,3),data_cam2(:,4),'x');
axis([0 1280 0 960]);
axis ij
