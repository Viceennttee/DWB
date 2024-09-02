cam = webcam(2);
preview(cam);

img = snapshot(cam);

% Display the frame in a figure window.
image(img);