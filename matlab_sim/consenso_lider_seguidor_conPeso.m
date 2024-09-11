clear all
close all
clc

%% Variables de simulación y control
l = 0.45; %%Distancia entre llantas de un pioneer estandar (Obtenida después de una breve investigación)
t = 0; %%Tiempo actual
dt = 0.01; %%Tiempo de step estándar en Coppelia
tf = 30; %%Tiempo de simulación


%% Referencia a los puntos objetivo
%%Coordenadas del objetivo 1
xd = 2%2*cos(t)-2
yd = 4%2*sin(t)


%% Variables de Control
kpr = 2; %%Influencia en la agresividad/velocidad de giro
kpra = 10;
kpd = 2; %%Ganacias de control
kpda1 = 30;
kpda2 = 30;
v_max = 0.15; %%Velocidad máxima alcanzada por el sistema
v_maxs = 0.15;
w_max = 1;

x1 = 0; %rand()*20-10;
y1 = 0; %rand()*20-10;
theta1 = pi/2;

x2 = -0.5;%rand()*20-10;
y2 = 0;%rand()*20-10;
theta2 = pi/2;



dd = 0.5;
dda = dd*2;
Thetae = [];
i = 1;
j = 0;
tic %%Usando la configuración de tiempo real en Coppelia, se usa el tiempo real

if sqrt((xd-x1)^2+(yd-y1)^2) > sqrt((xd-x2)^2+(yd-y2)^2)
    lid = 2;
    v_maxs = v_max;
    v_max = 0.15;
else
    lid = 1;
end

while t < tf


    %% Control de robots

    %% Calculo de vectores de posición

    if lid == 1

        PeL = [xd;yd]-[x1;y1];
        
        Pe2 = [x1;y1]-[x2;y2];
        d1 = norm(Pe2);
        de2 = d1-dd;
        
        V1L  =  (PeL/norm(PeL))*kpd;
        V12d = -(Pe2/norm(Pe2))*kpda1*de2;
        V21d =  (Pe2/norm(Pe2))*kpda2*de2;
    
        V1d = V1L + V12d;
        V2d = V21d;
    else
        PeL = [xd;yd]-[x2;y2];
        
        Pe2 = [x1;y1]-[x2;y2];
        d1 = norm(Pe2);
        de2 = d1-dd;
        
        V1L  =  (PeL/norm(PeL))*kpd;
        V12d = -(Pe2/norm(Pe2))*kpda2*de2;
        V21d =  (Pe2/norm(Pe2))*kpda1*de2;
    
        V1d = V12d;
        V2d = V1L + V21d;
    end


    %% Cálculo de referencias
    theta1d = atan2(V1d(2),V1d(1));
    theta2d = atan2(V2d(2),V2d(1));


%% Robot 1
    %% Cálculo de errores

    thetae1 = theta1 - theta1d;
    if thetae1 > pi
        thetae1 = thetae1 - 2*pi;
    elseif thetae1 < - pi
        thetae1 = thetae1 + 2*pi;
    end

    

    %% Leyes de control

    V1 =  norm(V1d);
    if V1 > (v_max)
        V1 = v_max;
    end
    w1 = -kpra*thetae1;

    if w1 > (w_max)
        w1 = w_max;
    end

%% Robot 2
    %% Cálculo de errores

    thetae2 = theta2 - theta2d;
    if thetae2 > pi
        thetae2 = thetae2 - 2*pi;
    elseif thetae2 < - pi
        thetae2 = thetae2 + 2*pi;
    end

    

    %% Leyes de control

    V2 =  norm(V2d);
    if V2 > (v_maxs)
        V2 = v_maxs;
    end
    w2 = -kpr*thetae2;

    if w2 > (w_max)
        w2 = w_max;
    end
    


    % if (de <= 0.2) && (V1 > 0.2)
    %     V2 = V1;
    % end
    % 
    % if (de <= 0.001)
    %     V2 = V1;
    %     w2 = w1;
    % end

    %% Condicionales de logro
     if sqrt((xd-x1)^2+(yd-y1)^2)<0.05 && lid == 1
        V1 = 0;
        w1 = 0;
        if de2 < 0.005
            V2 = 0;
            w2 = 0;
        end
    elseif sqrt((xd-x2)^2+(yd-y2)^2)<0.05 && lid == 2
        V2 = 0;
        w2 = 0;
        if de2 < 0.005
            V1 = 0;
            w1 = 0;
        end
    end


    %% Simulación del robot

    x1p = V1*cos(theta1);
    y1p = V1*sin(theta1);
    theta1p = w1;

    x2p = V2*cos(theta2);
    y2p = V2*sin(theta2);
    theta2p = w2;

    x1 = x1 + x1p*dt;
    y1 = y1 + y1p*dt;
    theta1 = theta1 + theta1p*dt;

    x2 = x2 + x2p*dt;
    y2 = y2 + y2p*dt;
    theta2 = theta2 + theta2p*dt;

    dt = toc;
    t = t + dt;
    tic;

    Thetae(i) = d1;
    i = i+1;

    figure(1)
    scatter(x1,y1,'color','blue','LineWidth',2);
    hold on
    scatter(x2,y2,'color','yellow','LineWidth',2);
    scatter(xd,yd,'color','green','LineWidth',2);
    plot([x1,x1+0.5*cos(theta1)],[y1,y1+0.5*sin(theta1)],'color','red','LineWidth',2)
    plot([x2,x2+0.5*cos(theta2)],[y2,y2+0.5*sin(theta2)],'color','red','LineWidth',2)

    hold off
    grid on
    axis([-10,10,-10,10]);
end

figure(2)
plot(Thetae)