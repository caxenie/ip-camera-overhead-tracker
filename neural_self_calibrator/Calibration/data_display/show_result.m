clear
close all


load -ascii ../result.txt

figure("Name","Simulation Results of Calibration Neural Network")
plot3(result(:,1),result(:,2),result(:,3),'x');
grid on
xlabel('x');
ylabel('y');
zlabel('z');
