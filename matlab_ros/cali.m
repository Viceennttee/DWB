% Paso 1: Leer las imágenes de calibración
imageDir = '/home/vicente/Pictures/Webcam';
images = imageDatastore(imageDir);

% Paso 2: Detectar puntos del tablero de ajedrez
[imagePoints, boardSize] = detectCheckerboardPoints(images.Files);

% Paso 3: Definir el tamaño de cada cuadrado en el tablero de ajedrez (en mm)
squareSize = 22; % Por ejemplo, 25 mm
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Paso 4: Calibrar la cámara
I = readimage(images, 1); % Leer la primera imagen para obtener las dimensiones de la imagen
imageSize = [size(I, 1), size(I, 2)];
params = estimateCameraParameters(imagePoints, worldPoints, 'ImageSize', imageSize);

% Paso 5: Mostrar los resultados de calibración
figure;
showReprojectionErrors(params);

% Paso 6: Guardar los parámetros intrínsecos para usarlos más tarde
save('cameraParams.mat', 'params');