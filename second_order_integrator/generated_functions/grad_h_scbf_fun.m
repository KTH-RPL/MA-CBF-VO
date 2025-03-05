function grad_h_scbf = grad_h_scbf_fun(in1,in2,in3,in4,in5,u_max,delta,alpha_c)
%GRAD_H_SCBF_FUN
%    GRAD_H_SCBF = GRAD_H_SCBF_FUN(IN1,IN2,IN3,IN4,IN5,U_MAX,DELTA,ALPHA_C)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    28-Feb-2025 13:10:05

p_A1 = in1(1,:);
p_A2 = in1(2,:);
p_B1 = in2(1,:);
p_B2 = in2(2,:);
v_A1 = in3(1,:);
v_A2 = in3(2,:);
v_B1 = in4(1,:);
v_B2 = in4(2,:);
t2 = p_A1.*2.0;
t3 = p_A2.*2.0;
t4 = p_B1.*2.0;
t5 = p_B2.*2.0;
t6 = -p_B1;
t8 = -p_B2;
t10 = 1.0./u_max;
t11 = -v_A1;
t12 = -v_A2;
t7 = -t4;
t9 = -t5;
t13 = p_A1+t6;
t14 = p_A2+t8;
t22 = t11+v_B1+2.220446049250313e-16;
t23 = t12+v_B2+2.220446049250313e-16;
t15 = t2+t7;
t16 = t3+t9;
t17 = t13.^2;
t18 = t14.^2;
t19 = t17+t18;
t20 = 1.0./sqrt(t19);
t21 = t20.^3;
t24 = t13.*t20.*t22;
t25 = t14.*t20.*t23;
t26 = t24+t25;
grad_h_scbf = [(t15.*t20)./2.0+t10.*t26.*(-t20.*t22+(t13.*t15.*t21.*t22)./2.0+(t14.*t15.*t21.*t23)./2.0);(t16.*t20)./2.0+t10.*t26.*(-t20.*t23+(t13.*t16.*t21.*t22)./2.0+(t14.*t16.*t21.*t23)./2.0);t10.*t13.*t20.*t26;t10.*t14.*t20.*t26];
end
