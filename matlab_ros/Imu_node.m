function [] = imu_callback(msg)

        disp(msg.orientation)
end

setenv("ROS_DOMAIN_ID","0")
%ros2 node list
%ros2 topic list
i = 0;

testnode = ros2node("/node_imu");
%testpub = ros2publishetestnode,"/cmd_vel","geometry_msgs/Twist");
%chatterMSG = ros2message("geometry_msgs/Twist");
imuSub = ros2subscriber(testnode,"/imu",@imu_callback);
while(1)
end
