clear all

function [] = img_callback(msg)
        img = rosReadImage(msg);
        disp("si está entrando al cb")
        imshow(img)
end

setenv("ROS_DOMAIN_ID","0")
i = 0;

testnode = ros2node("/nodo_cam");
camSub = ros2subscriber(testnode,"/image_raw",@img_callback);

