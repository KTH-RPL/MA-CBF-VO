function grad_h_scbf = grad_h_scbf_fun(in1,in2,theta,v,in5,in6,u_max,delta,alpha_c)
%GRAD_H_SCBF_FUN
%    GRAD_H_SCBF = GRAD_H_SCBF_FUN(IN1,IN2,THETA,V,IN5,IN6,U_MAX,DELTA,ALPHA_C)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    03-Mar-2025 11:02:01

p_A1 = in1(1,:);
p_A2 = in1(2,:);
p_B1 = in2(1,:);
p_B2 = in2(2,:);
v_B1 = in5(1,:);
v_B2 = in5(2,:);
t2 = cos(theta);
t3 = sin(theta);
t4 = p_A1.*2.0;
t5 = p_A2.*2.0;
t6 = p_B1.*2.0;
t7 = p_B2.*2.0;
t10 = -p_B1;
t12 = -p_B2;
t14 = 1.0./u_max;
t8 = t2.*v;
t9 = t3.*v;
t11 = -t6;
t13 = -t7;
t15 = p_A1+t10;
t16 = p_A2+t12;
t17 = -t8;
t18 = -t9;
t19 = t4+t11;
t20 = t5+t13;
t21 = t15.^2;
t22 = t16.^2;
t23 = t21+t22;
t26 = t17+v_B1+2.220446049250313e-16;
t27 = t18+v_B2+2.220446049250313e-16;
t24 = 1.0./sqrt(t23);
t25 = t24.^3;
t28 = t15.*t24.*t26;
t29 = t16.*t24.*t27;
t30 = t28+t29;
grad_h_scbf = [(t19.*t24)./2.0+t14.*t30.*(-t24.*t26+(t15.*t19.*t25.*t26)./2.0+(t16.*t19.*t25.*t27)./2.0);(t20.*t24)./2.0+t14.*t30.*(-t24.*t27+(t15.*t20.*t25.*t26)./2.0+(t16.*t20.*t25.*t27)./2.0);t14.*t30.*(t8.*t16.*t24+t15.*t18.*t24);t14.*t30.*(t2.*t15.*t24+t3.*t16.*t24)];
end
