#test fundamental

close all
clear

load 'fundamental.txt'
load 'cam0_log.txt'
load 'cam1_log.txt'

cam0_log(:,1) = 780/2 * (cam0_log(:,1)+1);
cam0_log(:,2) = 582/2 * (cam0_log(:,2)+1);
cam1_log(:,1) = 780/2 * (cam1_log(:,1)+1);
cam1_log(:,2) = 582/2 * (cam1_log(:,2)+1);

F = reshape(fundamental,3,3)'


sz = size(cam0_log);

cam_r = [cam0_log,ones(sz(1),1)];
cam_l = [cam1_log,ones(sz(1),1)];


d = 0;

for i=1:sz(1)
	d = d + abs(cam_l(i,:) * F * cam_r(i,:)');
end

d = d / sz(1)


figure(1);

for i=1:800:size(cam_r,1)
	subplot(211),plot(cam_r(i,1),cam_r(i,2),'x')	
	axis([0 780 0 582]);
	set(gca(),"ydir","reverse");
	
	el = F * cam_r(i,:)';
	x0 = 0;
	y0 = -1/el(2) * (el(3) + el(1)*x0);
	x1 = 779;
	y1 = -1/el(2) * (el(3) + el(1)*x1);
	
	subplot(212),plot(cam_l(i,1),cam_l(i,2),'x'), hold on;
	plot([x0,x1],[y0,y1]),hold off;
	axis([0 780 0 582]);
	set(gca(),"ydir","reverse");

	pause(0.5);
end
