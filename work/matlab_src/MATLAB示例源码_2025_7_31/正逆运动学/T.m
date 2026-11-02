function output = T(alpha_i_1, a_i_1, d_i, theta_i)
T_11 = cos(theta_i);
T_12 = -sin(theta_i);
T_13 = 0;
T_14 = a_i_1;
T_21 = sin(theta_i)*cos(alpha_i_1);
T_22 = cos(theta_i)*cos(alpha_i_1);
T_23 = -sin(alpha_i_1);
T_24 = -sin(alpha_i_1)*d_i;
T_31 = sin(theta_i)*sin(alpha_i_1);
T_32 = cos(theta_i)*sin(alpha_i_1);
T_33 = cos(alpha_i_1);
T_34 = cos(alpha_i_1)*d_i;
T_41 = 0;
T_42 = 0;
T_43 = 0;
T_44 = 1;
output = [T_11 T_12 T_13 T_14
          T_21 T_22 T_23 T_24
          T_31 T_32 T_33 T_34
          T_41 T_42 T_43 T_44];
end
