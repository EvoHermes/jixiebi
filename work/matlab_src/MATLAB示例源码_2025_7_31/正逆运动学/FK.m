%% 正运动学分析
clc
clear
close all
%% 初始化
theta_i = zeros(1,5);
T05 = [1 0 0 0
       0 1 0 0
       0 0 1 0
       0 0 0 1];
%% 输入关节角
for i = 1:5
    fprintf('请输入关节角%d(rad)',i);
    switch i
        case 1
            fprintf('(-%s/2 ~ %s/2): ',char(960),char(960));
        case 2
            fprintf('(-%s/2 ~ %s/2): ',char(960),char(960));
        case 3
            fprintf('(-%s/2 ~ %s/2): ',char(960),char(960));
        case 4
            fprintf('(-%s/2 ~ 1  ): ' ,char(960));
        case 5
            fprintf('(-%s/2 ~ %s/2): ',char(960),char(960));
    end
    theta_i(i) = input('');
end
%% D-H参数表
l = 0.12273;
alpha_i_1 = [0 pi/2 0 0 pi/2];
a_i_1 = [0 0 0.105 0.0975 0.02822];
d_i = [0.1085 0 0 0 0.0501];
%% 求解夹爪相对于坐标系0的变换矩阵
for i = 1:5
    if (i==2 || i==4)
        theta_i(i) = theta_i(i) + pi/2;
    end
    T05 = T05*T(alpha_i_1(i),a_i_1(i),d_i(i),theta_i(i));
end
T5t = [1 0 0 0
       0 1 0 0
       0 0 1 l
       0 0 0 1];
T0t = T05*T5t;
if (T0t(3,4) < 0)
    clc
    clear
    close all
    error('夹爪末端触碰到地面，请重新运行程序！');
end
%% 运动仿真
robot = importrobot('robotic_arm\robotic_arm.urdf');
q = randomConfiguration(robot); % 生成随机位形
for i = 1:5
    q(i).JointPosition = 0; % 初始化
end
for i = 1:5
    if (i==2 || i==4)
        temp = theta_i(i) - pi/2;
    else
        temp = theta_i(i);
    end
    for t = 0:temp/20:temp
        q(i).JointPosition = t;
        show(robot,q);
        xlim([-0.5 0.5])
        ylim([-0.5 0.5])
        zlim([0 0.6])
        pause(0.1);
    end
end
show(robot,q);
xlim([-0.5 0.5])
ylim([-0.5 0.5])
zlim([0 0.6])
%% 打印结果
result = getTransform(robot,q,'gripper','base_link');
for i = 1:4
    for j = 1:4
        result(i,j) = round(result(i,j)*1e4)/1e4; % 结果保留四位小数
        T0t(i,j) = round(T0t(i,j)*1e4)/1e4;       % 结果保留四位小数
    end
end
fprintf('仿真结果为：\n');
disp(result);
fprintf('计算结果为：\n');
disp(T0t);
