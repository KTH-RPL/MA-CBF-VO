function h_vo_inequality = h_vo_inequality_fun(in1,in2,in3,in4,in5,R_A,R_B,alpha_vo)
%H_VO_INEQUALITY_FUN
%    H_VO_INEQUALITY = H_VO_INEQUALITY_FUN(IN1,IN2,IN3,IN4,IN5,R_A,R_B,ALPHA_VO)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    28-Feb-2025 13:10:04

p_A1 = in1(1,:);
p_A2 = in1(2,:);
p_B1 = in2(1,:);
p_B2 = in2(2,:);
u_A1 = in5(1,:);
u_A2 = in5(2,:);
v_A1 = in3(1,:);
v_A2 = in3(2,:);
v_B1 = in4(1,:);
v_B2 = in4(2,:);
t2 = R_A+R_B;
t3 = -p_B1;
t4 = -p_B2;
t5 = -v_A1;
t6 = -v_A2;
t7 = t2.^2;
t8 = p_A1+t3;
t9 = p_A2+t4;
t13 = t5+v_B1+2.220446049250313e-16;
t14 = t6+v_B2+2.220446049250313e-16;
t10 = -t7;
t11 = t8.^2;
t12 = t9.^2;
t16 = t13.^2;
t17 = t14.^2;
t15 = t10+t11+t12;
t21 = t16+t17;
t18 = sqrt(t15);
t22 = sqrt(t21);
t19 = conj(t18);
t23 = 1.0./t22;
t20 = 1.0./t19;
h_vo_inequality = -u_A1.*(-p_A1+p_B1+(t19.*t23.*(v_A1.*-2.0+v_B1.*2.0+4.440892098500626e-16))./2.0)-u_A2.*(-p_A2+p_B2+(t19.*t23.*(v_A2.*-2.0+v_B2.*2.0+4.440892098500626e-16))./2.0)+v_A1.*(v_A1-v_B1+(t20.*t22.*(p_A1.*2.0-p_B1.*2.0))./2.0-2.220446049250313e-16)+v_A2.*(v_A2-v_B2+(t20.*t22.*(p_A2.*2.0-p_B2.*2.0))./2.0-2.220446049250313e-16)-alpha_vo.*(t8.*t13+t9.*t14-t18.*t22);
end
