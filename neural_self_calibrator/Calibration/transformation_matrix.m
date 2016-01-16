clear
close all

%{
sigma = 0;
T0 = transl(0.1,-0.2,0.5)*Roty(3.1)*Rotz(pi/2)*Rotx(1.3)*transl(1.1,-0-.55,-1.8)
A = rand(5000,3);
B = T0*[A,ones(size(A,1),1)]';
B = B(1:3,:)';

A = A + sigma * randn(5000,3);
B = B + sigma * randn(5000,3);
%}

A = load("-ascii","net_log_0.txt");
B = load("-ascii","net_log_1.txt");

if(size(A)!=size(B))
	disp("A and B not the same size");
	exit;
end

ma = sum(A)/size(A,1);
mb = sum(B)/size(B,1);

ax = 0;
for k=1:size(A,1)
    ax = ax + sum((A(k,:)-ma).^2);
end
va = ax / size(A,1);


C = zeros(3,3);
for k=1:size(A,1)
    C = C + (B(k,:)'- mb')*(A(k,:)'- ma')';
end
C = C/size(A,1);

[U,D,V] = svd(C);


S = eye(3);
if abs(det(U)*det(V)-1.0)>1e-6
	S(3,3) = -1;
end 


c = trace(D)/va;
R = U * S * V';
t = mb' - c*R*ma';

T = [c*R,t;[0 0 0 1]]
TT = inv(T)
E = [A,ones(size(A,1),1)]' - TT*[B,ones(size(B,1),1)]';
e = 1/size(A,1) * sum(sum(E(1:3,:).^2))

a = A(1,:)
b = B(1,:)
c = TT * [b,1]';
c = c(1:3)'