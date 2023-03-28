%% Praca dyplomowa "Metody generowania trajerktorii robotow mobilnych"


%% Model trójkołowego robota mobilnego 
clear all
syms fi fi1 fi2 Pxdot Pydot Pxref Pyref x1 y1 r l m ax ay e x y;

fi = 0;
fi1 = 1.5;
fi2=1;
% Macierze obiektu
Mo=(r/2*l)*[l*sin(fi) l*sin(fi); l*cos(fi) l*cos(fi); 1 -1]*[fi1;fi2];
% x=Mo(1,1);
% y=Mo(2,1);
% Fi=Mo(3,1);
% Polozenie kola sterowego
B=atan(((m/l)*(fi1-fi2))/(fi1+fi2)); 
% Predkosc kola sterowego
fi3=(m/2*l)*((fi1-fi2)/sin(B));

% Ograniczenie predkosci obrotu kola sterowego
% Bpr=(((2*m)/l)*cos(B))*((fi11*fi2-fi22*fi1)/(fi1+fi2)^2);



%% Regulator bazujący na dyfeomorfizmie

u=(1/r*e)*[l*cos(fi)+e*sin(fi) e*cos(fi)-l*sin(fi); -l*cos(fi)+e*sin(fi) e*cos(fi)+l*sin(fi)]*[[Pxdot;Pydot]-[ax 0;0 ay]*[(x1+e*sin(fi))-Pxref;...
    (y1+e*cos(fi))-Pyref]];
tex = 'plots/';
 set(figure(1), 'Position',  [1410, 100, 700, 400]) 
  subplot(2,1,1);
  plot(out.x,out.y)
  title('Polozenie robota')
  xlabel('X')
  ylabel('Y')
  subplot(2,1,2);
 
  plot(out.tout,out.x_dot)
  hold on
  plot(out.tout,out.y_dot)
  xlabel('Czas[s]')
  ylabel('Prędkość ^{m}/_{s}')
  title('Rozkład prędkości')
  legend('Prędkość w osi X','Prędkość w osi Y')
  
print('-f1', '-dpng', strcat([tex 'regulator_wynik']), '-r500') 
%% Test modelu

clear all
r=0.2;
l=0.25;
m=0.5;
ax=0.1950;
ay=0.1950;
e=1;

tex = 'plots';

out = sim('model_robot','ReturnWorkspaceOutputs','on');

 set(figure(1), 'Position',  [1410, 100, 700, 400]) 
  subplot(3,1,1);
 
  plot(out.tout,out.x)
  title('Polozenie na osi X')
  
  subplot(3,1,2);
 
  plot(out.tout,out.y)
  title('Polozenie na osi Y')
  
  subplot(3,1,3);
 
  plot(out.tout,out.b)
  title('Kąt obrotu koła sterowego \beta')
print('-f1', '-dpng', strcat([tex '\x_y_b_model']), '-r500')  
 
set(figure(2), 'Position',  [1410, 100, 700, 400])
  plot(out.x,out.y)
  xlabel('X')
  ylabel('Y')
  title('$Polozenie robota dla \phi_1 = 2.1 ^{m}/_{s} i \phi_2 = 1 ^{m}/_{s}')
  print('-f2', '-dpng', strcat([tex '\x_y_model']), '-r500')
  
  
  ylabel('$\dot{\gamma}$', 'Interpreter','latex')
  
 
%% Generatory trajektorii 


%% Flatness Trajectory

clear all


syms T t beta_0 beta_T Theta_0 Theta_T x_0 y_0 x_T y_T l ;

a1 = ((2*(x_T - x_0)) - abs(x_T - x_0))/(2*T);
a2 = (abs(x_T -x_0))/(T^2);
a3 = (2*(x_T -x_0) + abs(x_T -x_0))/(2*T);

A = [T^3 T^4 T^5; 3*T^2 4*T^3 5*T^4; 6*T 12*T^2 20*T^3]; 

c = [(y_T - y_0 - T*a1*tan(Theta_0)-T^2 * ((a2*tan(beta_0))/(2*l*cos(Theta_0).^3)));...
    (a3*tan(Theta_T)- a1*tan(Theta_0)-T * ((a2*tan(beta_0))/(l*cos(Theta_0).^3)));...
    ((a2*tan(beta_T))/(l*cos(Theta_T).^3)) - ((a2*tan(beta_0))/(l*cos(Theta_0).^3))];

b = A^(-1)*c;

b1 = b(1,1);
b2 = b(2,1);
b3 = b(3,1);


xt = ((T-t)/T)*x_0 + (t/T)*x_T + abs(x_T -x_0)*(t*(t-T)/(2*T^2));
yt = y_0 + t*a1*tan(Theta_0) + t^2*((a2*tan(beta_0))/(2*l*cos(Theta_0).^3)) + ...
    t^3*b1 + t^4*b2 + t^5*b3;

                   
Theta = atan(2*T^2*(a1*tan(Theta_0) + ((a2*tan(beta_0))/(l*cos(Theta_0).^3))*t^2+3*b1*t^2 + 4*b2*t^3 + 5*b3*t^4 )...
    /(2*T*(x_T-x_0) - T*abs(x_T - x_0) + 2*abs(x_T - x_0)*t));

fi =atan(((((a2*tan(beta_0))/(l*cos(Theta_0).^3)) + 6*b1*t + 12*b2*t^2 + 20*b3*t^3)*(2*T^2)^2*cos(Theta_0).^3 )...
    /((2*T*(x_T-x_0) - T*abs(x_T - x_0) + 2*abs(x_T - x_0)*t))^2) ;


f = matlabFunction(xt,yt,Theta,fi,'File','myfile',...
                   'Outputs',{'X','Y','O','Fi'}); 




%% test 

close all
clear all

ts = 2500;
beta_0 = 0;
beta_T = 0;
x_0 = 0;
x_T = 105;
y_0 = 0;
y_T = 105;
T = 1200;
Theta_0 = 80*(pi/180);
Theta_T = 0;
tp=12000;
l = 0.5;

X = zeros(1,tp);
Y = zeros(1,tp);
O = zeros(1,tp);
Fi = zeros(1,tp);
k=1;
tic
for i = 0:0.1:T-0.1  
    t=i;
    [X(k),Y(k),O(k),Fi(k)] = myfile(Theta_0,Theta_T,T,beta_0,beta_T,l,t,x_0,x_T,y_0,y_T) ;              
    k = k+1;               
end
toc


beta_0 = Fi(end);
beta_T = 0;
x_0 = X(end);
x_T = 130;
y_0 = Y(end);
y_T = 25;
T = 1200;
Theta_0 = O(end);
Theta_T = 45*(pi/180);

[X1,Y1,O1,Fi1] = evaluation(Theta_0,Theta_T,T,beta_0,beta_T,l,tp,x_0,x_T,y_0,y_T);

X = [X X1];
Y = [Y Y1];

tp = tp*2;

%plot(X,Y)
load('default.mat');
out = sim('untitled1','ReturnWorkspaceOutputs','on');

tex = 'plots\flatness';
plotresults(out,X,Y,tex,1)
save([tex '\F'],'out')

%%  Polynomial Trajectory

clear all


syms T t beta_0 beta_T Theta_0 Theta_T x_0 y_0 x_T y_T;


g =((x_T-x_0)/(T));

D = [T (1/2)*T^2 (1/3)*T^3; (1/2)*g*T^2 (1/6)*g*T^3 (1/12)*g*T^4;...
    (1/6)*g^2*T^3 (1/24)*g^2*T^4 (1/60)*g^2*T^5];

e = [(beta_T - beta_0);(Theta_T - Theta_0 - (g*beta_0*T));...
    (y_T - y_0 - g*Theta_0*T - (0.5*g^2*beta_0*T^2))];


h = D^(-1)*e;

h1 = h(1,1);
h2 = h(2,1);
h3 = h(3,1);

xt1 = x_0 + g*t;
yt1 = y_0 + g*Theta_0*t + (1/2)*g^2*beta_0*t^2 + (1/6)*g^2*h1*t^3 +...
    (1/24)*g^2*h2*t^4 + (1/60)*g^2*h3*t^5;

Theta = Theta_0 + g*beta_0*t + (1/2)*g*h1*t^2 + (1/6)*g*h2*t^3 + (1/12)*g*h3*t^4;

Fi = beta_0 + h1*t + (1/2)*h2*t^2 + (1/3)*h3*t^3;

f1 = matlabFunction(xt1,yt1,Theta,Fi,'File','poly',...
                   'Outputs',{'X1','Y1','O','F'}); 




%% Test

clear all

ts = 2500;
beta_0 = 0;
beta_T = 0;
x_0 = 0;
x_T = 105;
y_0 = 0;
y_T = 105;
T = 1200;
Theta_0 = 80*(pi/180);
Theta_T = 0;
tp=12000;


X = zeros(1,tp);
Y = zeros(1,tp);
k=1;

tic
for i = 0:0.1:T-0.1  
    t=i;
    [X(k),Y(k),O(k),Fi(k)] = poly(Theta_0,Theta_T,T,beta_0,beta_T,t,x_0,x_T,y_0,y_T) ;              
    k = k+1;               
end
toc

load('default.mat');


beta_0 = Fi(end);
beta_T = 0;
x_0 = X(end);
x_T = 130;
y_0 = Y(end);
y_T = 25;
T = 1200;
Theta_0 = O(end);
Theta_T = 45*(pi/180);


X1 = zeros(1,tp);
Y1 = zeros(1,tp);
k=1;

tic
for i = 0:0.1:T-0.1  
    t=i;
    [X1(k),Y1(k),O1(k),Fi1(k)] = poly(Theta_0,Theta_T,T,beta_0,beta_T,t,x_0,x_T,y_0,y_T) ;              
    k = k+1;               
end
toc


 X = [X X1];
 Y = [Y Y1];
 O = [O O1];
 Fi = [Fi Fi1];
 tp = tp*2;

load('default.mat');
out = sim('untitled1','ReturnWorkspaceOutputs','on');

%plot(0:0.1:2*T-0.1,(rad2deg(atan((O)))-90)*-1)
%hold on
%plot(out.tout,out.Omega)
%hold off

%plot(0:0.1:2*T-0.1,(rad2deg(atan((Fi)))))
%hold on
%plot(out.tout,out.fi)
%hold off

tex = 'plots\polyminal';
plotresults(out,X,Y,tex,2)
save([tex '\P'],'out')

%%  Symmetric Polynomial Trajectory

clear all


syms T t beta_0 beta_T Theta_0 Theta_T x_0 y_0 x_T y_T ks l;


ax = ks*cos(Theta_T)-3*x_T;
bx = ks*cos(Theta_0)-3*x_0;

ay = ks*sin(Theta_T)-3*y_T;
by = ks*sin(Theta_0)-3*y_0;

xt = -((t/T)-1)^3*x_0 + (t/T)^3*x_T + ax*(t/T)^2*((t/T)-1)...
    + bx*(t/T)*((t/T)-1)^2;

yt = -((t/T)-1)^3*y_0 + (t/T)^3*y_T + ay*(t/T)^2*((t/T)-1)...
    + by*(t/T)*((t/T)-1)^2;

xt_dot =  -3*((t/T)-1)^2*x_0 + 3*(t/T)^2*x_T + ax*2*(t/T)*((t/T)-1)+ ax*(t/T)^2 ...
    + bx*((t/T)-1)^2 + bx*2*(t/T)*((t/T)-1);

yt_dot =  -3*((t/T)-1)^2*y_0 + 3*(t/T)^2*y_T + ay*2*(t/T)*((t/T)-1)+ ay*(t/T)^2 ...
    + by*((t/T)-1)^2 + by*2*(t/T)*((t/T)-1);

xt_dot2 =  -6*((t/T)-1)*x_0 + 6*(t/T)*x_T + ax*2*(2*(t/T)-1)+ ax*2*(t/T) ...
    + bx*2*((t/T)-1) + bx*2*(2*(t/T)-1);

yt_dot2 =  -6*((t/T)-1)*y_0 + 6*(t/T)*y_T + ay*2*(2*(t/T)-1)+ ay*2*(t/T) ...
    + by*2*((t/T)-1) + by*2*(2*(t/T)-1);

Theta = atan(yt_dot/xt_dot);
fi = atan(l*((cos(Theta).^3*yt_dot2)/(xt_dot)^2)); 
f = matlabFunction(xt,yt,Theta,fi,'File','spoly',...
                   'Outputs',{'X','Y','O','Fi'}); %uzyj raz przy tworzeniu funkcji
               
%% Test

clear all

ts = 2500;
ks = 1;
l = 0.5;
beta_0 = 0;
beta_T = 0;
x_0 = 0;
x_T = 105;
y_0 = 0;
y_T = 105;
T = 1200;
Theta_0 = 80*(pi/180);
Theta_T = 0;
tp = 12000;

X = zeros(1,tp);
Y = zeros(1,tp);
k=1;

tic
for i = 0:0.1:T-0.1  
    t=i;
    [X(k),Y(k),O(k),Fi(k)] = spoly(Theta_0,Theta_T,T,ks,l,t,x_0,x_T,y_0,y_T);             
    k = k+1;               
end
toc



beta_0 = Fi(end);
beta_T = 0;
x_0 = X(end);
x_T = 130;
y_0 = Y(end);
y_T = 25;
T = 1200;
Theta_0 = O(end);
Theta_T = 45*(pi/180);


X1 = zeros(1,tp);
Y1 = zeros(1,tp);
k=1;

tic
for i = 0:0.1:T-0.1  
    t=i;
    [X1(k),Y1(k),O1(k),Fi1(k)] =spoly(Theta_0,Theta_T,T,ks,l,t,x_0,x_T,y_0,y_T);              
    k = k+1;               
end
toc


 X = [X X1];
 Y = [Y Y1];
 tp = tp*2;


load('default.mat');
out = sim('untitled1','ReturnWorkspaceOutputs','on');
tex = 'plots\spolyminal';
plotresults(out,X,Y,tex,3)
save([tex '\Sp'],'out')



%% Podsumowanie

tex = 'plots\podsumowanie';

plotend(tex)












               

