%% 生成可达工作空间
clc
clear
close all
%% 初始化
x = zeros(10000,1);
y = zeros(10000,1);
z = zeros(10000,1);
%% 随机模拟方法
robot = importrobot('robotic_arm\robotic_arm.urdf');
for i = 1:10000
    q = randomConfiguration(robot);
    T0t = getTransform(robot,q,'gripper','base_link'); % 读取末端执行器的位置和姿态
    x(i) = T0t(1,4);
    y(i) = T0t(2,4);
    z(i) = T0t(3,4);
end
show(robot,'Frames','off');
hold on
plot3(x,y,z,'b.')
hold off
grid on
xlim([-0.5 0.5])
ylim([-0.5 0.5])
zlim([0 0.6])
