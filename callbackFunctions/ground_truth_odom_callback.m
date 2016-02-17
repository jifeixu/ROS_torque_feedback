function ground_truth_odom_callback( ~, msg )
%GROUND_TRUTH_ODOM_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global odometer;

odometer.posX=msg.Pose.Pose.Position.X;
odometer.posY=msg.Pose.Pose.Position.Y;
odometer.velX=msg.Twist.Twist.Linear.X;
odometer.velY=msg.Twist.Twist.Linear.Y;



end

