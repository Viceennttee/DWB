% Inicializar la variable global para almacenar datos
global data;
data = struct('time', [], 'ax', [], 'ay', [], 'az', []);

% Configuración del entorno ROS y el suscriptor
setenv("ROS_DOMAIN_ID", "0");
testnode = ros2node("/node_imu");
imuSub = ros2subscriber(testnode, "/imu", @imu_callback);

% Función de callback para almacenar los datos
function [] = imu_callback(msg)
    global data;
    
    % Acceder a los datos del mensaje
    ax = msg.linear_acceleration.x;
    ay = msg.linear_acceleration.y;
    az = msg.linear_acceleration.z;
    
    % Obtener el tiempo actual
    t = datetime('now');
    
    % Almacenar los datos
    data.time(end+1) = datenum(t);
    data.ax(end+1) = ax;
    data.ay(end+1) = ay;
    data.az(end+1) = az;
end
