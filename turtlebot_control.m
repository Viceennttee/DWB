
setenv("ROS_DOMAIN_ID","0")
%ros2 node list
%ros2 topic list

clear all
close all
clc

%% Variables de simulación y control
l = 0.45; %%Distancia entre llantas de un pioneer estandar (Obtenida después de una breve investigación)
t = 0; %%Tiempo actual
dt = 0.01; %%Tiempo de step estándar en Coppelia
tf = 50; %%Tiempo de simulación


%% Referencia a los puntos objetivo
%%Coordenadas del objetivo 1
xd = 4
yd = 2


%% Variables de Control
kpr = 2; %%Influencia en la agresividad/velocidad de giro
kpra = 8;
kprl = 4;
kpd = 3; %%Ganacias de control
kpda = 10;
kpdl = 6;
kgamma = 10; %%Ganacias de control
v_max = 0.1; %%Velocidad máxima alcanzada por el sistema
w_max = 1;

x1 = 0; %rand()*20-10;
y1 = 0; %rand()*20-10;
theta1 = 0;

x2 = -0.25;%rand()*20-10;
y2 = 0;%rand()*20-10;
theta2 = 0;

x3 = 0.25;%rand()*20-10;
y3 = 0;%rand()*20-10;
theta3 = 0;


dd = 0.25;
dda = dd*2;
Thetae = [];
i = 1;
j = 0;
tic %%Usando la configuración de tiempo real en Coppelia, se usa el tiempo real


%% Conexión a ROS
testnode = ros2node("/nodo_consenso");
testpub = ros2publisher(testnode,"/cmd_vel","geometry_msgs/Twist");
chatterMSG = ros2message("geometry_msgs/Twist")
chatterMSG.linear.x = 0.0; 
chatterMSG.angular.z = 0.0; 
send(testpub,chatterMSG)


while t < tf

    %% Control del robot seguidor

    PeL = [xd;yd]-[x1;y1];
    
    Pe = [x1;y1]-[x2;y2];
    d1 = norm(Pe);
    de = d1-dd;

    Pe23 = [x2;y2]-[x3;y3];
    d23 = norm(Pe23);
    de23 = d23-dda;

    Pe3 = [x1;y1]-[x3;y3];
    d3 = norm(Pe3);
    de3 = d3-dd;

    V1Ld =  (PeL/norm(PeL))*kpd;
    V12d = -(Pe/norm(Pe))*kpdl*de;
    V13d = -(Pe3/norm(Pe3))*kpdl*de3;
    V21d =  (Pe/norm(Pe))*kpdl*de;
    V23d = -(Pe23/norm(Pe23))*kpda*de23;
    V32d =  (Pe23/norm(Pe23))*kpda*de23;
    V31d =  (Pe3/norm(Pe3))*kpdl*de3;
    
    V1d = V12d + V13d + V1Ld;
    V2d = V21d + V23d;
    V3d = V32d + V31d;


    %% Cálculo de referencias
    theta1d = atan2(V1d(2),V1d(1));
    theta2d = atan2(V2d(2),V2d(1));
    theta3d = atan2(V3d(2),V3d(1));

%% Robot 1
    
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
    w1 = -kpr*thetae1;

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
    if V2 > (v_max)
        V2 = v_max;
    end
    w2 = -kpra*thetae2;

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

%% Robot3
    %% Cálculo de errores

    thetae3 = theta3 - theta3d;
    if thetae3 > pi
        thetae3 = thetae3 - 2*pi;
    elseif thetae3 < - pi
        thetae3 = thetae3 + 2*pi;
    end

    %% Leyes de control

    V3 =  norm(V3d);
    if V3 > (v_max)
        V3 = v_max;
    end
    w3 = -kpra*thetae3;
    
    if w3 > (w_max)
        w3 = w_max;
    end

    % if (de3 <= 0.2) && (V1 > 0.2)
    %     V3 = V1;
    % end

    if sqrt((xd-x1)^2+(yd-y1)^2)<0.05
        V1 = 0;
        w1 = 0;
        if de < 0.05 && de3 < 0.05
            V2 = 0;
            w2 = 0;
            V3 = 0;
            w3 = 0;
        end
    end

    %% Simulación del robot

    x1p = V1*cos(theta1);
    y1p = V1*sin(theta1);
    theta1p = w1;

    x2p = V2*cos(theta2);
    y2p = V2*sin(theta2);
    theta2p = w2;

    x3p = V3*cos(theta3);
    y3p = V3*sin(theta3);
    theta3p = w3;

    x1 = x1 + x1p*dt;
    y1 = y1 + y1p*dt;
    theta1 = theta1 + theta1p*dt;

    x2 = x2 + x2p*dt;
    y2 = y2 + y2p*dt;
    theta2 = theta2 + theta2p*dt;

    x3 = x3 + x3p*dt;
    y3 = y3 + y3p*dt;
    theta3 = theta3 + theta3p*dt;
    dt = toc;
    t = t + dt;
    tic;

    Thetae(i) = d23;
    i = i+1;

    %% Mandar datos a ROS
    chatterMSG.linear.x = V2; 
    chatterMSG.angular.z = w2; 
    send(testpub,chatterMSG)

    figure(1)
    scatter(x1,y1,'color','blue','LineWidth',2);
    hold on
    scatter(x2,y2,'color','yellow','LineWidth',2);
    scatter(x3,y3,'color','green','LineWidth',2);
    plot([x1,x1+0.5*cos(theta1)],[y1,y1+0.5*sin(theta1)],'color','red','LineWidth',2)
    plot([x2,x2+0.5*cos(theta2)],[y2,y2+0.5*sin(theta2)],'color','red','LineWidth',2)
    plot([x3,x3+0.5*cos(theta3)],[y3,y3+0.5*sin(theta3)],'color','red','LineWidth',2)

    hold off
    grid on
    axis([-10,10,-10,10]);
end
chatterMSG.linear.x = 0.0; 
chatterMSG.angular.z = 0.0; 
send(testpub,chatterMSG)

figure(2)
plot(Thetae)
