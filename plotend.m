function plotend(tex)


Fg = load('plots\flatness\gen_x_y.mat');
Fr = load('plots\flatness\F.mat');

Pg = load('plots\polyminal\gen_x_y.mat');
Pr = load('plots\polyminal\P.mat');

Spg = load('plots\spolyminal\gen_x_y.mat');
Spr = load('plots\spolyminal\Sp.mat');

SE= load('plots\spolyminal\Sp.mat');

figure(1)

set(figure(1), 'Position',  [1410, 100, 700, 400])
subplot(2,1,1)
plot(Fg.X,Fg.Y)
axis([0 145 -50 110]) % dla 3
xlabel('Pozycja po X')
ylabel('Pozycja po Y')
title('Generatory trajektorii')
hold on
plot(Pg.X,Pg.Y)
plot(Spg.X,Spg.Y)
legend({'Płaski','Wielomianowy','Symetryczny wielomianowy'},'Location','southeast','NumColumns',3)
hold off

subplot(2,1,2)
plot(Fr.out.x,Fr.out.y)
axis([0 145 -50 110]) % dla 3
xlabel('Pozycja po X')
ylabel('Pozycja po Y')
title('Odpowiedź robota na zadaną trajektorię')
hold on
plot(Pr.out.x,Pr.out.y)
plot(Spr.out.x,Spr.out.y)
legend({'Płaską','Wielomianową','Symetryczną wielomianową'},'Location','southeast','NumColumns',3)
hold off
print('-f1', '-dpng', strcat([tex '\1_x_y']), '-r500')

figure(2)

set(figure(2), 'Position',  [1410, 100, 700, 400])


plot(Fr.out.tout,Fr.out.Omega)
xlabel('Czas [s]')
ylabel('Kąt ^{o}')
title('Położenie kątowe \theta robota')
hold on

plot(Pr.out.tout,Pr.out.Omega)
plot(Spr.out.tout,Spr.out.Omega)
legend('Płaski','Wielomianowy','Symetryczny wielomianowy')
hold off
print('-f2', '-dpng', strcat([tex '\2_theta']), '-r500')


figure(3)

set(figure(3), 'Position',  [1410, 100, 700, 400])
plot(Fr.out.tout,Fr.out.fi)
xlabel('Czas [s]')
ylabel('Kąt ^{o}')
title('Położenie kątowe \beta koła sterowego robota')
hold on
plot(Pr.out.tout,Pr.out.fi)
plot(Spr.out.tout,Spr.out.fi)
legend('Płaski','Wielomianowy','Symetryczny wielomianowy')
hold off
print('-f3', '-dpng', strcat([tex '\3_beta']), '-r500')


figure(4)

set(figure(4), 'Position',  [1410, 100, 700, 400])
plot(Fr.out.tout,Fr.out.velocity)
xlabel('Czas [s]')
ylabel('Prędkość ^{m}/_{s}')
title('Prędkość liniowa na wyjściu robota')
hold on
plot(Pr.out.tout,Pr.out.velocity)
plot(Spr.out.tout,Spr.out.velocity)
legend('Płaski','Wielomianowy','Symetryczny wielomianowy')
hold off
print('-f4', '-dpng', strcat([tex '\4_predkosc']), '-r500')

figure(5)

set(figure(5), 'Position',  [1410, 100, 700, 400])
plot(Fr.out.x,Fr.out.y)
axis([0 145 -50 110]) % dla 3
xlabel('Pozycja po X')
ylabel('Pozycja po Y')
title('Odpowiedź robota na zadaną trajektorię')
hold on
plot(Pr.out.x,Pr.out.y)
plot(Spr.out.x,Spr.out.y)
legend({'Płaską','Wielomianową','Symetryczną wielomianową'},'Location','southeast','NumColumns',3)
hold off
print('-f5', '-dpng', strcat([tex '\5_x_y']), '-r500')
close all
end