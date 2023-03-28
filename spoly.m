function [X,Y,O,Fi] = spoly(Omega_0,Omega_T,T,ks,l,t,x_0,x_T,y_0,y_T)
%SPOLY
%    [X,Y,O,FI] = SPOLY(OMEGA_0,OMEGA_T,T,KS,L,T,X_0,X_T,Y_0,Y_T)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    22-Jun-2021 17:34:27

t2 = cos(Omega_0);
t3 = cos(Omega_T);
t4 = sin(Omega_0);
t5 = sin(Omega_T);
t6 = t.^2;
t7 = t.^3;
t8 = x_0.*3.0;
t9 = x_0.*6.0;
t10 = x_T.*3.0;
t11 = x_T.*6.0;
t12 = y_0.*3.0;
t13 = y_0.*6.0;
t14 = y_T.*3.0;
t15 = y_T.*6.0;
t20 = 1.0./T;
t16 = ks.*t2;
t17 = ks.*t3;
t18 = ks.*t4;
t19 = ks.*t5;
t21 = t20.^2;
t22 = t20.^3;
t31 = t.*t20;
t23 = t16.*2.0;
t24 = t17.*2.0;
t25 = t18.*2.0;
t26 = t19.*2.0;
t27 = -t16;
t29 = -t17;
t32 = -t18;
t34 = -t19;
t36 = t31.*2.0;
t37 = t31-1.0;
t38 = t6.*t10.*t21;
t39 = t6.*t14.*t21;
t51 = t6.*t21.*x_T.*-3.0;
t52 = t6.*t21.*y_T.*-3.0;
t28 = -t23;
t30 = -t24;
t33 = -t25;
t35 = -t26;
t40 = t8+t27;
t42 = t10+t29;
t44 = t12+t32;
t46 = t14+t34;
t48 = t36-1.0;
t49 = t37.^2;
t50 = t37.^3;
t41 = t9+t28;
t43 = t11+t30;
t45 = t13+t33;
t47 = t15+t35;
t53 = t8.*t49;
t54 = t12.*t49;
t55 = t6.*t21.*t42;
t56 = t6.*t21.*t46;
t57 = t40.*t49;
X = -t31.*t57-t37.*t55-t50.*x_0+t7.*t22.*x_T;
if nargout > 1
    t58 = t44.*t49;
    Y = -t31.*t58-t37.*t56-t50.*y_0+t7.*t22.*y_T;
end
if nargout > 2
    t59 = t31.*t37.*t41;
    t60 = t31.*t37.*t43;
    t61 = t31.*t37.*t45;
    t62 = t31.*t37.*t47;
    t63 = t51+t53+t55+t57+t59+t60;
    t64 = t52+t54+t56+t58+t61+t62;
    O = atan(t64./t63);
end
if nargout > 3
    t65 = 1.0./t63.^2;
    Fi = -atan(l.*t65.*1.0./(t64.^2.*t65+1.0).^(3.0./2.0).*(t31.*t47+t37.*t45+t45.*t48+t47.*t48+t37.*y_0.*6.0-t31.*y_T.*6.0));
end
