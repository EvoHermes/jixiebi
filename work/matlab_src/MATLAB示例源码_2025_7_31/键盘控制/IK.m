function theta = IK(T0t,theta_old)
%% D-H参数表
l = 0.12273;
a_i_1 = [0 0 0.105 0.0975 0.02822];
d_i = [0.1085 0 0 0 0.0501];
%% 初始化
theta = zeros(2,5);
t234 = zeros(2,1); % t234表示θ2+θ3+θ4
M = zeros(2,1);
N = zeros(2,1);
K = zeros(2,1);
k = 0;
%% 求解关节5相对于坐标系0的变换矩阵
T5t = [1 0 0 0
       0 1 0 0
       0 0 1 l
       0 0 0 1];
T05 = T0t*T5t^-1;
%% 求解关节角1
theta(1,1) = atan2(T05(2,4),T05(1,4));
theta(2,1) = atan2(-T05(2,4),-T05(1,4));
%% 求解关节角2+关节角3+关节角4
for i = 1:2
    if (abs(sin(theta(i,1))) < 1e-3) % 判断sin(θ1)是否等于0
        t234(i) = atan2(-T05(1,3)/cos(theta(i,1)),T05(3,3));
        if (abs( T05(2,3) + sin(t234(i)) * sin(theta(i,1)) ) > 1e-3) % 判断θ2+θ3+θ4是否存在
            t234(i) = NaN;
        end
    else
        t234(i) = atan2(-T05(2,3)/sin(theta(i,1)),T05(3,3));
        if (abs( T05(1,3) + sin(t234(i)) * cos(theta(i,1)) ) > 1e-3) % 判断θ2+θ3+θ4是否存在
            t234(i) = NaN;
        end
    end
end
%% 求解关节角5
for i = 1:2
    if (abs(sin(t234(i))) < 1e-3) % 判断sin(θ2+θ3+θ4)是否等于0
        if (abs(T05(3,3)-1) < 1e-3) % 判断cos(θ2+θ3+θ4)是否等于1
            theta(i,5) = atan2(-T05(2,1),-T05(1,1)) - theta(i,1);
        elseif (abs(T05(3,3)+1) < 1e-3) % 判断cos(θ2+θ3+θ4)是否等于-1
            theta(i,5) = theta(i,1) - atan2(T05(2,1),T05(1,1));
        end
    else
        theta(i,5) = atan2(T05(3,2)/sin(t234(i)),-T05(3,1)/sin(t234(i)));
    end
end
%% 求解关节角3
for i = 1:2
    if (abs(sin(theta(i,1))) < 1e-3) % 判断sin(θ1)是否等于0
        M(i) = (-T05(1,4)/cos(theta(i,1))) - a_i_1(5)*cos(t234(i)) - d_i(5)*sin(t234(i));
    else
        M(i) = (-T05(2,4)/sin(theta(i,1))) - a_i_1(5)*cos(t234(i)) - d_i(5)*sin(t234(i));
    end
    N(i) = T05(3,4) - d_i(1) + a_i_1(5)*sin(t234(i)) - d_i(5)*cos(t234(i));
    K(i) = (M(i)^2 + N(i)^2 - (a_i_1(3))^2 - (a_i_1(4))^2) / (2*a_i_1(3)*a_i_1(4));
    if (abs(K(i)-1) < 1e-3)
        K(i)=1;
    elseif (abs(K(i)+1) < 1e-3)
        K(i)=-1;
    end
    if (abs(K(i)) > 1)
        theta(i,3) = NaN;
    else
        theta(i,3) = acos(K(i));
    end
end
theta = [theta; theta];
for i = 3:4
    theta(i,3) = -theta(i,3);
end
t234 = [t234; t234];
M = [M; M];
N = [N; N];
%% 求解关节角2
for i = 1:4
    P = a_i_1(3) + a_i_1(4)*cos(theta(i,3));
    Q = a_i_1(4)*sin(theta(i,3));
    s2 = (P*M(i) - Q*N(i))/(P^2 + Q^2);
    c2 = (Q*M(i) + P*N(i))/(P^2 + Q^2);
    theta(i,2) = atan2(s2,c2);
end
%% 求解关节角4
for i = 1:4
    theta(i,4) = t234(i) - theta(i,2) - theta(i,3);
end
%% 筛选合适的解
temp = theta;
for i = 1:4
    for j = 1:5
        if(isnan(theta(i,j)))
            temp(i-k,:)=[];
            k = k+1;
            break
        end
    end
end
theta = temp;
%% 换算至合理的区间范围
for i = 1:(4-k)
    for j = 1:5
        if (theta(i,j) > pi)
            theta(i,j) = theta(i,j) - 2*pi;
        elseif (theta(i,j) < -pi)
            theta(i,j) = theta(i,j) + 2*pi;
        end
        theta(i,j) = round(theta(i,j)*1e3)/1e3;
    end
end
%% 寻找最优解
theta = unique(theta,'rows'); % 去除重复解
n = size(theta,1);
temp = zeros(n,1);
for i = 1:n
    temp(i) = norm((theta(i,:)-theta_old),1); % 计算每组解的1范数
end
id = find(temp == min(temp)); % 以1-范数最小为最优条件
if (length(id) > 1)
    id = id(1);
end
theta = theta(id,:);
end
