cam = webcam(1);

% Paso 1: Cargar los parámetros de calibración
load('parametros_laptop.mat', 'params');

while true
    pause(0.25); 
    % Capturar una imagen de la cámara
    I = snapshot(cam);

    % Detectar los marcadores ArUco en la imagen
    [ids, locs, detectedFamily] = readArucoMarker(I);
    numMarkers = length(ids);

    for i = 1:numMarkers
        pause(0.25)
        loc = locs(:,:,i);

        % Mostrar el ID del marcador y la familia
        disp("Detected marker ID, Family: " + ids(i) + ", " + detectedFamily(i));  
        
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
        fprintf('Coordenadas en el mundo real (X, Y) en mm: %.2f, %.2f\n', ...
                centerWorld(1), centerWorld(2));

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
    end
    
    % Mostrar la imagen procesada con las anotaciones
    imshow(I);
    
    % Mostrar la posición del centro del marcador en píxeles (opcional)
    %disp(center);
end
