cam = webcam(2);
%preview(cam);

%camera paremeters
focalLength    = [800 800]; 
principalPoint = [320 240];
imageSize      = [480 640];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

%other parameters
markerSizeInMM = 66;
markerFamily = "DICT_6X6_250";


while(1)
    img = snapshot(cam);
    [img,camIntrinsics] = undistortFisheyeImage(img,intrinsics);
    imshow(img)
    [ids,locs,poses] = readArucoMarker(I,markerFamily,camintrinsics,markerSizeInMM);


    % Origin and axes vectors for the object coordinate system
    worldPoints = [0 0 0; markerSizeInMM/2 0 0; 0 markerSizeInMM/2 0; 0 0 markerSizeInMM/2];
    
    for i = 1:length(poses)
      % Get image coordinates for axes
      imagePoints = world2img(worldPoints,poses(i),camIntrinsics);
     
      axesPoints = [imagePoints(1,:) imagePoints(2,:);
                    imagePoints(1,:) imagePoints(3,:);
                    imagePoints(1,:) imagePoints(4,:)];
      % Draw colored axes
      img = insertShape(I, "Line", axesPoints, ...
          Color = ["red","green","blue"], LineWidth=10);
    imshow(img)
    end
end