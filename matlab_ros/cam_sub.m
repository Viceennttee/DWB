function img_callback(msg)
    while(1)
        img = rosReadImage(msg);
        ros2 topic list
    end
end
setenv("ROS_DOMAIN_ID","0")
%ros2 node list
%ros2 topic list
i = 0;

testnode = ros2node("/nodo_victor");
%testpub = ros2publisher(testnode,"/cmd_vel","geometry_msgs/Twist");
%chatterMSG = ros2message("geometry_msgs/Twist");
camSub = ros2subscriber(testnode,"/image_raw","sensor_msgs/Image",img_callback);


