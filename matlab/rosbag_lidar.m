bag = rosbag('C:\Users\olivas\Desktop\SAut\2024-05-10-19-51-09.bag');
% Select messages with the topic '/scan'
scan_msgs = select(bag, 'Topic', '/scan');

% Read the messages
scan_data = readMessages(scan_msgs);

%% Initialize video
myVideo = VideoWriter('myVideoFile'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)

% Process and visualize the LiDAR data
figure; % Create a new figure for the plot
polarAxes = polaraxes; % Create polar axes
for i = 1:numel(scan_data)
    % Extract scan data
    ranges = scan_data{i}.Ranges; % Range data from LiDAR
    angles = scan_data{i}.AngleMin + scan_data{i}.AngleIncrement * (0:numel(ranges)-1); % Corresponding angles
    
    % Filter out invalid measurements
    valid_indices = ranges > scan_data{i}.RangeMin & ranges < scan_data{i}.RangeMax;
    valid_ranges = ranges(valid_indices);
    valid_angles = angles(valid_indices);
    
    % Visualize LiDAR data
    polarplot(polarAxes, valid_angles, valid_ranges, '.'); % Plot points instead of a line
    title('LiDAR Scan');
    rticklabels(polarAxes, {'0', '', '', '', ''}); % Set radial tick labels
    text(0, 0, 'Angle (radians)', 'HorizontalAlignment', 'center');
    pause(0.05); % Pause to display each scan (adjust as needed)

    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
    
    % Set fixed radial limits
    rlim(polarAxes, [0, max(scan_data{i}.RangeMax)]);
end

close(myVideo)