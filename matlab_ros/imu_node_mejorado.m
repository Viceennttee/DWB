function [] = imu_callback(msg)
        orient = msg.orientation; 
        w = msg.angular_velocity;
        acceleration = msg.linear_acceleration;
        %disp(orient)
        %disp(w)
        disp(acceleration)
end

setenv("ROS_DOMAIN_ID","0")
%ros2 node list
%ros2 topic list


testnode = ros2node("/node_imu");
%testpub = ros2publishetestnode,"/cmd_vel","geometry_msgs/Twist");
%chatterMSG = ros2message("geometry_msclgs/Twist");
imuSub = ros2subscriber(testnode,"/imu",@imu_callback);
%while(1)
%end