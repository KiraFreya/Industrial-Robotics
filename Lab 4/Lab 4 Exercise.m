L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
robot = SerialLink([L1 L2 L3],'name','myRobot');

workspace = [-4 4 -4 4 -4 4];                                       % Set the size of the workspace when drawing the robot        
scale = 0.5;        
q = zeros(1,3);                                                     % Create a vector of initial joint angles        
robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
        
robot.teach;                                                        % Open a menu to move the robot manually
input('Press enter to continue');

robot.base = troty(pi);               % Rotate the base around the Y axis so the Z axis faces down ways
        