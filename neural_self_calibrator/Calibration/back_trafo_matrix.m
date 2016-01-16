H = load("-ascii","vector_log.txt");
H = H';


gx = [1;0;0];
gy = [0;1;0];
gz = [0;0;1];

hx = H(:,1) ./ norm(H(:,1));
hy = H(:,2) ./ norm(H(:,2));
hz = H(:,3) ./ norm(H(:,3));


%Rotation: Determine angle and axis of rotation between real unit vector and computed vector from network
phi_x = atan2( norm( cross( gx,hx ) ) ,  gx'*hx);
n_x = 1/(norm(gx)*norm(hx)) * cross(gx,hx);
n_x = n_x / norm(n_x);


%rotate ex' around vector n_x with angle -phi_x to receive regular unit vector ex
Rot1 = eye(3)*cos(-phi_x) + [0, -n_x(3), n_x(2); n_x(3), 0, -n_x(1); -n_x(2), n_x(1), 0]*sin(-phi_x) + (1-cos(-phi_x))*n_x*n_x';
Rot1 = [Rot1,[0,0,0]';[0,0,0,1]];

%rotate cosy
H = [[hx,hy,hz];[1 1 1 ]];
H = Rot1 * H;


%read new vectors;
hx = H(1:3,1) ./ norm(H(1:3,1));
hy = H(1:3,2) ./ norm(H(1:3,2));
hz = H(1:3,3) ./ norm(H(1:3,3));

sign(cross(hx,hy)'*hz);
phi_y = atan2(norm(cross(gy,hy)),gy'*hy);
n_y = 1/(norm(gy)*norm(hy)) * cross(gy,hy);
n_y = n_y / norm(n_y);


Rot2 = Rotx(-1*sign(hx'*n_y)*phi_y);



H = Rot2 * H;

%read new vectors and normalize
hx = H(1:3,1) ./ norm(H(1:3,1));
hy = H(1:3,2) ./ norm(H(1:3,2));
hz = H(1:3,3) ./ norm(H(1:3,3));


%is the cosy right-handed, do we need to flip the z axis
M = eye(4);
if(sign(cross(hx,hy)'*hz)<0)
	disp("switched");
	M(3,3) = -1;
end

H = M * H;
  
T_matrix = M * Rot2 * Rot1;
T_matrix = T_matrix(1:3,1:3);

save("-ascii","RotationMatrix.txt","T_matrix");