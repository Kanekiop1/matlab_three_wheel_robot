function plotresults(out,X,Y,tex,g)

figure(1)
set(figure(1), 'Position',  [1410, 100, 700, 400])

%plot(X,Y,'--k')
plot(out.x,out.y)
save([tex '\gen_x_y'],'X','Y')
Xr = out.x;
Yr = out.y;
save([tex '\rob_x_y'],'Xr','Yr')
xlabel('Pozycja po X')
ylabel('Pozycja po Y')
title('Położenie robota')
%axis([0 110 0 110])
hold on
%plot(out.x,out.y)
plot(105,105,'-o','MarkerSize',8,'MarkerEdgeColor','r');
plot(130,25,'-o','MarkerSize',8,'MarkerEdgeColor','r')

%legend('Generator','Robot')
hold off


%print('-f1', '-dpng', '/plots', '-r500')

print('-f1', '-dpng', strcat([tex '\1_x_y']), '-r500')


figure(2)

set(figure(2), 'Position',  [1410, 100, 700, 400])
plot(out.tout,out.velocity)
xlabel('Czas [s]')
ylabel('Prędkość ^{m}/_{s}')
title('Prędkość liniowa na wyjściu robota')
print('-f2', '-dpng', strcat([tex '\5_vliniowa']), '-r500')

figure(3)

set(figure(3), 'Position',  [1410, 100, 700, 400])
plot(out.tout,out.Omega)
xlabel('Czas [s]')
ylabel('Kąt ^{o}')
title('Położenie kątowe \theta robota')
%legend('Trajektoria')
print('-f3', '-dpng', strcat([tex '\2_theta']), '-r500')

figure(4)
set(figure(4), 'Position',  [1410, 100, 700, 400])

plot(out.tout,out.fi)
xlabel('Czas [s]')
ylabel('Kąt ^{o}')
title('Położenie kątowe \beta koła sterowego robota')
print('-f4', '-dpng', strcat([tex '\3_fi']), '-r500')



figure(5)

set(figure(5), 'Position',  [1410, 100, 700, 400])
plot(out.tout,out.w1)
xlabel('Czas [s]')
ylabel('Prędkość ^{m}/_{s}')
title('Prędkość liniowa na wejściu robota')
hold on
plot(out.tout,out.w2)
legend('\phi_1','	\phi_2')
print('-f5', '-dpng', strcat([tex '\4_predkosci']), '-r500')



figure(6)

set(figure(6), 'Position',  [1410, 100, 700, 400])
plot(out.x,out.y,'*')    
xlabel('Pozycja po X')
ylabel('Pozycja po Y')
title('Położenie robota z uwzględnieniem kąta obrotu \theta')
hold on
for i = 1:100:25000

quiver(out.x(i),out.y(i),sind(out.Omega(i)),cosd(out.Omega(i)),10,'r')
pause(0.01);

end
print('-f6', '-dpng', strcat([tex '\6_symulacja']), '-r500')



figure(7)


set(figure(7), 'Position',  [1410, 100, 700, 400])

subplot(3,1,1)

plot(out.x,out.y)
xlabel('Pozycja po X')
ylabel('Pozycja po Y')

title('Położenie robota')
if g == 3
    axis([0 140 -20 110]) % dla 3
else
    axis([0 140 -5 110])
end
hold on

plot(105,105,'-o','MarkerSize',8,'MarkerEdgeColor','r');
plot(130,25,'-o','MarkerSize',8,'MarkerEdgeColor','r')
hold off

subplot(3,1,2)
plot(out.tout,out.Omega)
xlabel('Czas [s]')
ylabel('Kąt ^{o}')
title('Położenie kątowe \theta robota')
%legend('Trajektoria')


subplot(3,1,3)
plot(out.tout,out.fi)
xlabel('Czas [s]')
ylabel('Kąt ^{o}')
title('Położenie kątowe \beta koła sterowego robota')

print('-f7', '-dpng', strcat([tex '\7_sub_stan']), '-r500')


figure(8)

subplot(2,1,1)
set(figure(8), 'Position',  [1410, 100, 700, 400])
plot(out.tout,out.velocity)
xlabel('Czas [s]')
ylabel('Prędkość ^{m}/_{s}')
title('Prędkość liniowa na wyjściu robota')



subplot(2,1,2)
plot(out.tout,out.w1)
xlabel('Czas [s]')
ylabel('Prędkość ^{m}/_{s}')
title('Prędkość liniowa na wejściu robota')
hold on
plot(out.tout,out.w2)
legend('$\dot{\varphi_1}$','$\dot{\varphi_2}$', 'Interpreter','latex')
print('-f8', '-dpng', strcat([tex '\8_vliniowa']), '-r500')

close all
end