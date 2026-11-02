clc
clear
close all
%% 初始化
T0t = [ 0  0  1  0.2203
        0 -1  0    0
        1  0  0  0.3017
        0  0  0    1 ];
theta_old = zeros(1,5);
theta = IK(T0t,theta_old);
robot = importrobot('robotic_arm\robotic_arm.urdf');
q = randomConfiguration(robot); % 生成随机位形
for i = 1:5
    q(i).JointPosition = 0; % 位形初始化
end
for i = 1:5
    for t = 0:theta(i)/10:theta(i)
        q(i).JointPosition = t;
        subplot(1,2,1)
        show(robot,q,'Frames','off');
        title('初始化中')
        xlim([-0.2 0.5])
        ylim([-0.3 0.3])
        zlim([0 0.5])
        subplot(1,2,2)
        show(robot,q,'Frames','off');
        view(2)
        title('初始化中')
        xlim([-0.2 0.5])
        ylim([-0.3 0.3])
        zlim([0 0.5])
        pause(0.1);
    end
end
subplot(1,2,1)
title('可以进行键盘控制')
subplot(1,2,2)
title('可以进行键盘控制')
theta_old = theta;
%% 键盘控制
while (1)
    ch = getkey;
    T0t_old = T0t;
    if (ch == 'w')
        R = [ 1  0  0   0
              0  1  0   0
              0  0  1  0.01
              0  0  0   1 ];
        T0t = R*T0t;
    elseif (ch == 's')
        R = [ 1  0  0    0
              0  1  0    0
              0  0  1  -0.01
              0  0  0    1 ];
        T0t = R*T0t;
    elseif (ch == 'a')
        R = [ 1  0  0    0
              0  1  0    0
              0  0  1  -0.01
              0  0  0    1 ];
        T0t = T0t*R;
    elseif (ch == 'd')
        R = [ 1  0  0   0
              0  1  0   0
              0  0  1  0.01
              0  0  0   1 ];
        T0t = T0t*R;
    elseif (ch == 'i')
        R = [ 0.9848  -0.1736  0   0
              0.1736   0.9848  0   0
                0        0     1   0
                0        0     0   1 ];
        T0t = T0t*R;
    elseif (ch == 'k')
        R = [ 0.9848  0.1736  0   0
             -0.1736  0.9848  0   0
                0       0     1   0
                0       0     0   1 ];
        T0t = T0t*R;
    elseif (ch == 'j')
        R = [ 0.9848  0  0.1736  0
                0     1    0     0
             -0.1736  0  0.9848  0
                0     0    0     1 ];
        T0t = T0t*R;
    elseif (ch == 'l')
        R = [ 0.9848  0  -0.1736  0
                0     1     0     0
              0.1736  0   0.9848  0
                0     0     0     1 ];
        T0t = T0t*R;
    elseif (ch == 'z')
        gamma_old = atan2(T0t(2,4),T0t(1,4));
        gamma = atan2(T0t(2,4)+0.01,T0t(1,4));
        angle = gamma - gamma_old;
        R_l = [ cos(angle)  -sin(angle)  0  0
                sin(angle)   cos(angle)  0  0
                   0            0        1  0
                   0            0        0  1 ];
        distance = T0t(1,4)/cos(gamma) - T0t(1,4)/cos(gamma_old);
        R_r = [ 1  0  0     0
                0  1  0     0
                0  0  1  distance
                0  0  0     1 ];
        T0t = R_l*T0t*R_r;
    elseif (ch == 'x')
        gamma_old = atan2(T0t(2,4),T0t(1,4));
        gamma = atan2(T0t(2,4)-0.01,T0t(1,4));
        angle = gamma - gamma_old;
        R_l = [ cos(angle)  -sin(angle)  0  0
                sin(angle)   cos(angle)  0  0
                   0            0        1  0
                   0            0        0  1 ];
        distance = T0t(1,4)/cos(gamma) - T0t(1,4)/cos(gamma_old);
        R_r = [ 1  0  0     0
                0  1  0     0
                0  0  1  distance
                0  0  0     1 ];
        T0t = R_l*T0t*R_r;
    elseif (ch == 'q')
        break
    end
    theta = IK(T0t,theta_old);
    if (~isempty(theta))
        for i = 1:5
            q(i).JointPosition = theta(i);
        end
        subplot(1,2,1)
        show(robot,q,'Frames','off');
        title('可以进行键盘控制')
        xlim([-0.2 0.5])
        ylim([-0.3 0.3])
        zlim([0 0.5])
        subplot(1,2,2)
        show(robot,q,'Frames','off');
        view(2)
        title('可以进行键盘控制')
        xlim([-0.2 0.5])
        ylim([-0.3 0.3])
        zlim([0 0.5])
        pause(0.1);
    else
        subplot(1,2,1)
        title('无法继续移动')
        subplot(1,2,2)
        title('无法继续移动')
        T0t = T0t_old;
        theta = theta_old;
    end
    theta_old = theta;
end
