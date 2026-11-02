%% 逆运动学分析
clc
clear
close all
%% D-H参数表
l = 0.12273;
a_i_1 = [0 0 0.105 0.0975 0.02822];
d_i = [0.1085 0 0 0 0.0501];
%% 初始化
theta_i = zeros(2,5);
t234 = zeros(2,1); % t234表示θ2+θ3+θ4
M = zeros(2,1);
N = zeros(2,1);
K = zeros(2,1);
k = 0;
label = 1;
index = 0;
%% 生成机械臂的随机位形
robot = importrobot('robotic_arm\robotic_arm.urdf');
while (label)
    q0 = randomConfiguration(robot);
    T0t = getTransform(robot,q0,'gripper','base_link'); % 读取末端执行器的位置和姿态
    if (T0t(3,4) > 1e-2) % 设置末端执行器的位置在地面以上
        label = 0;
    end
end
%% 求解关节5相对于坐标系0的变换矩阵
T5t = [1 0 0 0
       0 1 0 0
       0 0 1 l
       0 0 0 1];
T05 = T0t*T5t^-1;
%% 求解关节角1
theta_i(1,1) = atan2(T05(2,4),T05(1,4));
theta_i(2,1) = atan2(-T05(2,4),-T05(1,4));
%% 求解关节角2+关节角3+关节角4
for i = 1:2
    if (abs(sin(theta_i(i,1))) < 1e-3) % 判断sin(θ1)是否等于0
        t234(i) = atan2(-T05(1,3)/cos(theta_i(i,1)),T05(3,3));
        if (abs( T05(2,3) + sin(t234(i)) * sin(theta_i(i,1)) ) > 1e-3) % 判断θ2+θ3+θ4是否存在
            t234(i) = NaN;
        end
    else
        t234(i) = atan2(-T05(2,3)/sin(theta_i(i,1)),T05(3,3));
        if (abs( T05(1,3) + sin(t234(i)) * cos(theta_i(i,1)) ) > 1e-3) % 判断θ2+θ3+θ4是否存在
            t234(i) = NaN;
        end
    end
end
%% 求解关节角5
for i = 1:2
    if (abs(sin(t234(i))) < 1e-3) % 判断sin(θ2+θ3+θ4)是否等于0
        if (abs(T05(3,3)-1) < 1e-3) % 判断cos(θ2+θ3+θ4)是否等于1
            theta_i(i,5) = atan2(-T05(2,1),-T05(1,1)) - theta_i(i,1);
        elseif (abs(T05(3,3)+1) < 1e-3) % 判断cos(θ2+θ3+θ4)是否等于-1
            theta_i(i,5) = theta_i(i,1) - atan2(T05(2,1),T05(1,1));
        end
    else
        theta_i(i,5) = atan2(T05(3,2)/sin(t234(i)),-T05(3,1)/sin(t234(i)));
    end
end
%% 求解关节角3
for i = 1:2
    if (abs(sin(theta_i(i,1))) < 1e-3) % 判断sin(θ1)是否等于0
        M(i) = (-T05(1,4)/cos(theta_i(i,1))) - a_i_1(5)*cos(t234(i)) - d_i(5)*sin(t234(i));
    else
        M(i) = (-T05(2,4)/sin(theta_i(i,1))) - a_i_1(5)*cos(t234(i)) - d_i(5)*sin(t234(i));
    end
    N(i) = T05(3,4) - d_i(1) + a_i_1(5)*sin(t234(i)) - d_i(5)*cos(t234(i));
    K(i) = (M(i)^2 + N(i)^2 - (a_i_1(3))^2 - (a_i_1(4))^2) / (2*a_i_1(3)*a_i_1(4));
    if (abs(K(i)-1) < 1e-3)
        K(i)=1;
    elseif (abs(K(i)+1) < 1e-3)
        K(i)=-1;
    end
    if (abs(K(i)) > 1)
        theta_i(i,3) = NaN;
    else
        theta_i(i,3) = acos(K(i));
    end
end
theta_i = [theta_i; theta_i];
for i = 3:4
    theta_i(i,3) = -theta_i(i,3);
end
t234 = [t234; t234];
M = [M; M];
N = [N; N];
%% 求解关节角2
for i = 1:4
    P = a_i_1(3) + a_i_1(4)*cos(theta_i(i,3));
    Q = a_i_1(4)*sin(theta_i(i,3));
    s2 = (P*M(i) - Q*N(i))/(P^2 + Q^2);
    c2 = (Q*M(i) + P*N(i))/(P^2 + Q^2);
    theta_i(i,2) = atan2(s2,c2);
end
%% 求解关节角4
for i = 1:4
    theta_i(i,4) = t234(i) - theta_i(i,2) - theta_i(i,3);
end
%% 筛选合适的解
temp = theta_i;
for i = 1:4
    for j = 1:5
        if(isnan(theta_i(i,j)))
            temp(i-k,:)=[];
            k = k+1;
            break
        end
    end
end
theta_i = temp;
%% 换算至合理的区间范围
for i = 1:(4-k)
    for j = 1:5
        if (theta_i(i,j) > pi)
            theta_i(i,j) = theta_i(i,j) - 2*pi;
        elseif (theta_i(i,j) < -pi)
            theta_i(i,j) = theta_i(i,j) + 2*pi;
        end
        theta_i(i,j) = round(theta_i(i,j)*1e3)/1e3;
    end
end
%% 寻找最优解
theta_i = unique(theta_i,'rows'); % 去除重复解
n = size(theta_i,1);
temp = zeros(n,1);
for i = 1:n
    temp(i) = norm(theta_i(i,:),1); % 计算每组解的1范数
end
id = find(temp == min(temp)); % 以1-范数最小为最优条件
if (length(id) > 1)
    id = id(1);
end
theta_i = theta_i(id,:);
%% 运动仿真
q1 = randomConfiguration(robot); % 生成随机位形
for i = 1:5
    q1(i).JointPosition = 0; % 初始化
end
for i = 1:5
    temp = theta_i(i);
    for t = 0:temp/20:temp
        index = index + 1;
        q1(i).JointPosition = t;
        show(robot,q0,"Visuals","off");
        hold on
        show(robot,q1);
        hold off
        xlim([-0.5 0.5])
        ylim([-0.5 0.5])
        zlim([0 0.6])
        make_gif('IK.gif',index) % 生成动态图
        pause(0.1);
    end
end
show(robot,q0,"Visuals","off");
hold on
show(robot,q1);
hold off
xlim([-0.5 0.5])
ylim([-0.5 0.5])
zlim([0 0.6])
%% 打印结果
result = getTransform(robot,q1,'gripper','base_link');
for i = 1:4
    for j = 1:4
        result(i,j) = round(result(i,j)*1e4)/1e4; % 结果保留四位小数
        T0t(i,j) = round(T0t(i,j)*1e4)/1e4;       % 结果保留四位小数
    end
end
fprintf('目标位姿为：\n');
disp(T0t);
fprintf('实际位姿为：\n');
disp(result);
