clc
clear
close all
syms t1
syms t2
syms t3
syms t4
syms t5
syms d1
syms d5
syms a2
syms a3
syms a4
T01 = [ cos(t1) -sin(t1)  0   0
        sin(t1)  cos(t1)  0   0
              0        0  1  d1
              0        0  0   1];
T12 = [-sin(t2) -cos(t2)  0   0
              0        0 -1   0
        cos(t2) -sin(t2)  0   0
              0        0  0   1];
T23 = [ cos(t3) -sin(t3)  0  a2
        sin(t3)  cos(t3)  0   0
              0        0  1   0
              0        0  0   1];
T34 = [-sin(t4) -cos(t4)  0  a3
        cos(t4) -sin(t4)  0   0
              0        0  1   0
              0        0  0   1];
T45 = [ cos(t5) -sin(t5)  0  a4
              0        0 -1 -d5
        sin(t5)  cos(t5)  0   0
              0        0  0   1];
T05 = T01*T12*T23*T34*T45;
T05 = simplify(T05);
