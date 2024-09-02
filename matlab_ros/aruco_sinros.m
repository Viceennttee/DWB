I = imread("/home/vicente/Downloads/aruco3.jpg");
[ids,locs,detectedFamily] = readArucoMarker(I);
numMarkers = length(ids);
for i = 1:numMarkers
  loc = locs(:,:,i);
  
   
  % Display the marker ID and family
  disp("Detected marker ID, Family: " + ids(i) + ", " + detectedFamily(i))  
 
  % Insert marker edges
  I = insertShape(I,"polygon",{loc},Opacity=1,ShapeColor="green",LineWidth=4);
 
  % Insert marker corners
  markerRadius = 6;
  numCorners = size(loc,1);
  markerPosition = [loc,repmat(markerRadius,numCorners,1)];
  I = insertShape(I,"FilledCircle",markerPosition,ShapeColor="red",Opacity=1);
   
  % Insert marker IDs
  center = mean(loc);
  I = insertText(I,center,ids(i),FontSize=30,BoxOpacity=1);
end

imshow(I)