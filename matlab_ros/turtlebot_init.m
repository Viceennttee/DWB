
setenv("ROS_DOMAIN_ID","0")
%ros2 node list
%ros2 topic list
i = 0;

testnode = ros2node("/nodo_victor");
testpub = ros2publisher(testnode,"/cmd_vel","geometry_msgs/Twist");
chatterMSG = ros2message("geometry_msgs/Twist")

while i<100
    
    chatterMSG.linear.x = 0.0; 

    send(testpub,chatterMSG)

    i=i+1;

end


