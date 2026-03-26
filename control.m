clear all
close all
clc

L = 1; %distancia entre ruedas
r = 0.1; %radio llanta

%definición de condiciones iniciales
x = 0;
y = 0;
theta = 0;

%definición de variables de simulación
t = 0;
dt = 0.01;
Tf = 100;

%definición de ganacias
kpt = 2;
kpr = 10;

posd=[3 3; -1 1; 1 -1; -3 -3];%vector de puntos
posr=zeros(2,4);%vector para guardar posición real del robot al alcanzar la pose deseada
i = 1;
%velocidades máximas de los actuadores
wmax = 10;
vmax = 1.5;

vecX=[];
vecY=[];
while t<Tf
    %ley de control
    d=sqrt(((posd(i,1)-x)^2)+((posd(i,2)-y)^2));
    thetad=atan2((posd(i,2)-y),(posd(i,1)-x));
    thetae=theta-thetad;
    if thetae>pi
        thetae = thetae-pi*2;
    elseif thetae<-pi
        thetae = thetae+pi*2;
    end
   
    if abs(d)<0.1 
        posr(1,i)=x;
        posr(2,i)=y;
        if i == 4
            disp("termina");
            break;
        end
        i=i+1;

    end
    disp(i);
    V = vmax*tanh(kpt*d/vmax);
    w = -kpr*tanh(thetae/wmax);%-1*wmax*tanh(kpr*thetae/wmax);

    %vel ang de ruedas
    wl = (V-L*w/2)/r;
    wr = (V+L*w/2)/r;
    %vel lin de ruedas
    vl = wl*r;
    vr = wr*r;
    %vel lin y ang robot
    v =(vl+vr)/2;
    w =(wr-wl)/L;
    %ecuaciones cinemáticas
    xp = v*cos(theta);
    yp = v*sin(theta);
    thetap = w;
    %resolver ecuaciones
    dt=0.01;
    x = x + xp*dt;
    y = y + yp*dt;
    theta = theta + thetap*dt;
    %vectores para guardar los puntos de la ruta seguida por el robot
    vecX=[vecX x];
    vecY=[vecY y];
    %graficado
    figure (1)
    plot(x,y,'bo','LineWidth',2);
    hold on;
    plot([x x+0.5*cos(theta)], [y y+0.5*sin(theta)]);%Objeto gráfico que muestra la pose actual del  robot 
    plot(vecX, vecY,'o--','LineWidth',1);%dibuja la tryectoria seguida
    plot(posr(1,:), posr(2,:),'go','LineWidth',2);%dibuja los puntos alcanzados
    hold off

    axis([-5 5 -5 5])
    t = t + dt;
end