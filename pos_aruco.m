cam = webcam(2);

% Paso 1: Cargar los parámetros de calibración
load('parametros_laptop.mat', 'params');

setenv("ROS_DOMAIN_ID","0")
%ros2 node list
%ros2 topic list
i = 0;

testnode = ros2node("/pc_externa");
aruco_pub = ros2publisher(testnode,"/arucos_pose","geometry_msgs/PoseArray");
aruco_array = ros2message("geometry_msgs/PoseArray");
aruco0 = ros2message('geometry_msgs/Pose');
aruco1 = ros2message('geometry_msgs/Pose');
aruco2 = ros2message('geometry_msgs/Pose');

aruco0.position.x = 0;
aruco0.position.y = 0;
aruco0.orientation.z = 0;

aruco1.position.x = 1;
aruco1.position.y = 1;
aruco1.orientation.z = 1;

aruco2.position.x = 2;
aruco2.position.y = 2;
aruco2.orientation.z = 2;

aruco_array.header.frame_id ='arucos';
aruco_array.poses = [aruco0, aruco1, aruco2];


while true
    %pause(0.25); 
    % Capturar una imagen de la cámara
    I = snapshot(cam);

    % Detectar los marcadores ArUco en la imagen
    [ids, locs, detectedFamily] = readArucoMarker(I);
    numMarkers = length(ids);

    for i = 1:numMarkers
        %pause(0.25)
        loc = locs(:,:,i);

        % Mostrar el ID del marcador y la familia
        %disp("Detected marker ID, Family: " + ids(i) + ", " + detectedFamily(i));  
        disp(ids(i));
        arucoID = ids(i);
        % Insertar bordes del marcador
        I = insertShape(I,"polygon",{loc},Opacity=1,ShapeColor="green",LineWidth=4);
        
        % Insertar esquinas del marcador
        markerRadius = 6;
        numCorners = size(loc, 1);
        markerPosition = [loc, repmat(markerRadius, numCorners, 1)];
        I = insertShape(I,"FilledCircle", markerPosition, ShapeColor="red", Opacity=1);
        
        % Calcular el centro del marcador en coordenadas de píxeles
        center = mean(loc);

        % Convertir el centro de píxeles a coordenadas reales (en mm o cm)
        centerWorld = pointsToWorld(params.Intrinsics, params.RotationMatrices(:,:,1), ...
                                    params.TranslationVectors(1,:), center);

        % Mostrar el centro en coordenadas reales
        %fprintf('Centro del marcador ID: %d\n', ids(i));
        % fprintf('Coordenadas en el mundo real (X, Y) en mm: %.2f, %.2f\n', ...
        %         centerWorld(1), centerWorld(2));

        % Obtener el ángulo de inclinación del marcador
        % Definir el vector entre dos esquinas consecutivas del marcador en píxeles
        v1 = loc(1,:) - loc(2,:); % Vector entre dos esquinas del marcador

        % Calcular el ángulo de inclinación en el plano XY (plano del piso)
        angleRadians = atan2(v1(2), v1(1)); % ángulo en radianes
        angleDegrees = rad2deg(angleRadians); % Convertir a grados

        % Mostrar el ángulo de inclinación
        fprintf('Ángulo de inclinación en el plano XY (grados): %.2f\n', angleDegrees);

        % Insertar los IDs del marcador en la imagen
        I = insertText(I, center, ids(i), FontSize=30, BoxOpacity=1);

        
        % En este switch se le asigna la posición a cada aruco del topico de ROS
        switch arucoID
            case 0
                aruco0.position.x = centerWorld(1);
                aruco0.position.y = centerWorld(2);
                aruco0.orientation.z = angleDegrees;
            case 1
                aruco1.position.x = centerWorld(1);
                aruco1.position.y = centerWorld(2);
                aruco1.orientation.z = angleDegrees;
            case 2
                aruco2.position.x = centerWorld(1);
                aruco2.position.y = centerWorld(2);
                aruco2.orientation.z = angleDegrees;
            otherwise
                disp('Lectura falsa');
        end







        aruco_array.poses = [aruco0, aruco1, aruco2];
        send(aruco_pub, aruco_array);

        
    end
    
    % Mostrar la imagen procesada con las anotaciones

    imshow(I);
    % Mostrar la posición del centro del marcador en píxeles (opcional)
    %disp(center);
end
