r = UR3;
% Load the 'q' matrix from the .mat file
load('ur3_q.mat');
tr = r.model.fkine(q);

% Specify the time step and total time for the trajectory
time_step = 0.01; % Adjust this value as needed
total_time = size(q, 1) * time_step;

% Create a time vector
time_vector = 0:time_step:total_time;

% Iterate through the joint angles and set the robot's configuration
for i = 1:size(q, 1)
    % Set the joint angles for the current step
    current_q = q(i, :);
    
    % Plot or visualize the robot's configuration (optional)
    r.model.animate(current_q);
    
    % Pause for a short time to control the speed of the trajectory
    pause(time_step);
end


