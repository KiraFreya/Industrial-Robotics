L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
robot = SerialLink([L1 L2 L3],'name','myRobot');

%workspace = [-4 4 -4 4 -4 4];                                       % Set the size of the workspace when drawing the robot        
%scale = 0.5;        
%q = zeros(1,3);                                                     % Create a vector of initial joint angles        
%robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
        
%robot.teach;                                                        % Open a menu to move the robot manually

robot.base = troty(pi);               % Rotate the base around the Y axis so the Z axis faces down ways

% Set workspace, scale and initial joint state, then plot and teach
q = zeros(1,3);
robot.plot(q,'workspace',[-2 2 -2 2 -0.05 2],'scale',0.5);
robot.teach;

%1.4) Move the robot around with “teach” and observe that there is 
% no way to affect the Z, roll or pitch values, no matter what joint value you choose.

%1.5) Consider a pen that is mounted to the Z-axis of the final joint. 
% Note how this means we don't care about the yaw angle 
% (if it's a pen where the Z axis is rotating, it doesn't affect the drawing result.

%1.6) Get a solution for the end effector at [-0.75,-0.5,0], 
% and make sure you mask out the impossible-to-alter values (i.e. z, roll and pitch) 
% and the value we don%t care about (i.e. yaw). Thus we need a mask of [1,1,0,0,0,0]:

%newQ = zeros(1,3);
q = robot.ikine(transl(-0.75,-0.5,0), 'q0', newQ, 'mask', [1,1,0,0,0,0]);

% 1.7) Plot the new joint state and check how close it got to the [x, y] 
% in the goal transform (i.e. transl(-0.75,-0.5,0))
robot.plot(q)
robot.fkine(q)

%1.8) Go through a loop using the previous joint as the guess 
% to draw a line from [-0.75,-0.5,0] to [-0.75,0.5,0] 
% and animate each new joint state trajectory with:
robot.animate(q);