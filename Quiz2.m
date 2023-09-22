%% Quiz 2 Re-attempt
% %% Q6 - This one gives the right answer now!

% Create a UR3 robot instance
ur3Robot = UR3();

% Define the joint angles
q = [0,pi/10,-pi/2,0,pi/4,0];

% Calculate the end effector transformation matrix
endEffectorPose = ur3Robot.model.fkineUTS(q);

% Extract the Z-coordinate of the end effector position
endEffectorZ = endEffectorPose(3, 4);

% Calculate the distance from the end effector to the floor (table height is 0.4 meters)
distanceToFloor = endEffectorZ + 0.4; % Add the table height

% Round the result to 4 decimal places
distanceToFloor = round(distanceToFloor, 4);

% Display the result
disp(['Distance from end effector to floor: ', num2str(distanceToFloor), ' meters']);

%% Quiz 2 - Question 7 -
% % Load the Baxter robot model
% mdl_baxter;
% 
% % Define the joint configurations for the left and right arms
% qLeft = [0,-pi/2,0,0,6*pi/8,0,0];
% qRight = [0,3*pi/8,0,0,0,0,-3*pi/4];
% 
% % Calculate the end effector positions for both arms
% leftEndEffectorPose = left.fkineUTS(qLeft);
% rightEndEffectorPose = right.fkineUTS(qRight);
% 
% % Extract the translation vectors (end effector positions)
% leftEndEffectorPosition = leftEndEffectorPose(1:3, 4);
% rightEndEffectorPosition = rightEndEffectorPose(1:3, 4);
% 
% % Calculate the distance between the two end effectors
% distanceBetweenEndEffectors = norm(leftEndEffectorPosition - rightEndEffectorPosition) * 1000; % Convert to millimeters
% 
% % Round to the nearest whole number
% roundedDistance = round(distanceBetweenEndEffectors);
% 
% % Display the result
% fprintf('Distance between the left and right end effectors: %d millimeters\n', roundedDistance);
%% Lab3Solution - Question 8 -

function Question3()

clf

% Create the DensoVM6083 robot
densoRobot = DensoVM6083;

% Define the start and finish joint configurations
q1 = [0, -0.6981, 0, 0.3491, 0.5236, 0.1745];
q2 = [-0.2618, -0.6981, -1.571, 0.5236, 0, -0.3491];

% Generate a trajectory with a Quintic Polynomial velocity profile
trajectory = jtraj(q1, q2, 75);

% Initialize variables to store the largest distance
largestDistance = 0;

% Iterate through each step in the trajectory
for i = 1:size(trajectory, 1)

    % Get the joint angles for the current step
    q = trajectory(i, :);

    % Calculate the end effector's transformation matrix
    endEffectorPose = densoRobot.model.fkineUTS(q);

    % Extract the end effector's position (4th column of the transformation matrix)
    endEffectorPosition = endEffectorPose(1:3, 4);

    % Add the 4-meter extension along the Z-axis
    endEffectorPosition(3) = endEffectorPosition(3) + 12.0; % Adding 4 meters along Z-axis

    % Calculate the distance from the robot base (origin) to the end effector
    distanceToOrigin = norm(endEffectorPosition);

    % Update the largest distance if the current distance is larger
    largestDistance = max(largestDistance, distanceToOrigin);

end



% Display the largest distance in meters rounded to 4 decimal places
fprintf('Largest distance between robot base and laser: %.4f meters\n', largestDistance);
end

% %% Lab3Solution - Question 9 -
% function Question4()
% close all
% clc
% 
% 
% %% Options
% interpolation = 1;
% % 1 means the quintic polynomial, run this and record the number, change it to 2 and
% % then run it for the trapezodial velocity. quintic polynomial - trapezoidal velocity = answer
% % (haven't tested very well but it should be positive if not just swap the order for calculation e.g. quntin polymial - trapezoidal = answer)
% steps = 50; % change number of steps based on what they ask e.g. 50 or 100 etc
% 
% %% Load Model
% mdl_puma560                                                                 
% qlim = p560.qlim;                                                           
% 
% %% Define End-Effector transformation, use inverse kinematics to get joint angles
% q1 = [-0.4382 -0.5942 -0.0178 0.0000 -0.4550 -0.9113];
% q2 = [0.9113 0.2842 -0.7392 -3.1416 0.6120 -2.7034];              
% 
% %% Interpolate joint angles, also calculate relative velocity
% switch interpolation
%     case 1
%         qMatrix = jtraj(q1, q2, steps);
%     case 2
%         s = lspb(0, 1, steps);                                             	
%         qMatrix = nan(steps, 6);                                            
%         for i = 1:steps
%             qMatrix(i,:) = (1 - s(i)) * q1 + s(i) * q2;                   	
%         end
%     otherwise
%         error('interpolation = 1 for Quintic Polynomial, or 2 for Trapezoidal Velocity')
% end
% 
% % Calculate relative velocities for joint 6
% velocity = zeros(steps, 6);
% for i = 2:steps
%     velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);                          
% end
% 
% % Calculate absolute relative velocities for joint 6
% abs_relative_velocity = abs(velocity(:,6));  % Change the index to 6 for joint 6
% 
% % Find the largest difference between the two profiles
% largest_difference = max(abs_relative_velocity);
% 
% % Display the largest difference
% fprintf('The largest difference in absolute relative velocities for joint 6 is: %.4f\n', largest_difference);
% 
% end
