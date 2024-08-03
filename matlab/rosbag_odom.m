bag = rosbag('C:\Users\olivas\Desktop\SAut\2024-05-10-14-44-59.bag');
% Select messages with the topic '/scan'
%scan_msgs = select(bag, 'Topic', '/odom');

% Read the messages
odom_msgs = readMessages(select(bag, 'Topic', '/odom'));

% Extract position information
positions_x = zeros(1, numel(odom_msgs));
positions_y = zeros(1, numel(odom_msgs));

for i = 1:numel(odom_msgs)
    % Access odometry message fields
    odom_msg = odom_msgs{i};
    
    % Extract x and y position
    positions_x(i) = odom_msg.Pose.Pose.Position.X;
    positions_y(i) = odom_msg.Pose.Pose.Position.Y;
end

% Plot the position of the robot
figure;
plot(positions_x, positions_y);
title('Robot Position Over Time');
xlabel('X Position (m)');
ylabel('Y Position (m)');
grid on;
