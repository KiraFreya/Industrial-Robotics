classdef Lab3Solution < handle 
    methods
        function self = Lab3Solution( )
            self.Question1();
			% self.Question2();
			% self.Question3();
        end
    end
    methods(Static)
        %% Exercise 1: Derive the DH parameters for each of the manipulators provided. Use these to generate a model of the manipulator using the Robot Toolbox in MATLAB. 
        % 0 = Week 2 - 3-Link Planar Robot
        % 1 = 3-Link 3D Robot
        % 2 = AANBOT UR10 arm
        % 3 = SPIR igus arm
        % 4 = Sawyer Robot (7DOF) ( No solution since this is Lab Assignment #1 question)
        function Question1(modelIndex)
            clf
            if nargin < 1
                modelIndex = 1;
                display(['No model passed so assuming model',num2str(modelIndex)]);
            end
            
            switch modelIndex
                case 0   % Week 2 - 3-Link Planar Robo     
                    % Define the DH Parameters to create the Kinematic model
                    L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
                    L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
                    L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
                    
                    robotModel = SerialLink([L1 L2 L3],'name','myRobot')                     % Generate the model
                    % 
                    % workspace = [-4 4 -4 4 -4 4];                                       % Set the size of the workspace when drawing the robot        
                    % scale = 0.5;        
                    % q = zeros(1,3);                                                     % Create a vector of initial joint angles        
                    % robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
                    % 
                    % robot.teach;                                                        % Open a menu to move the robot manually
                    % input('Press enter to continue');
                    % 
                    % q = robot.getpos();                                                 % Get the joint angles at the current position        
                    % J = robot.jacob0(q);        
                    % J = J(1:3,1:3)                                                      % Grab only the first 3 rows        
                    % display('There is a deliberate problem here. Notice the failure to invert and thus no way to plot vellipse at this pose');
                    % inv(J)                                                              % Invert the Jacobian - what happens?        
                    % robot.vellipse(q);                                                  % Draw the manipulability ellipsoid
                    
                case 1 % 3-Link 3D Robot
                    % Specify the DH Parameters for the robot
                    L1 = Link('d',1,'a',0,'alpha',pi/2,'qlim',[-pi/2 pi/2])
                    L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi/2 pi/2])
                    L3 = Link('d',0,'a',1,'alpha',-pi/2,'qlim',[-pi/2 pi/2])
                            
                    robotModel = SerialLink([L1 L2 L3], 'name', '3-Link 3D Robot')                   % Create the robot model
                    
                    % workspace = [-4 4 -4 4 -4 4];                                       % Define the boundaries of the workspace        
                    % scale = 0.5;                                                        % Scale the robot down        
                    % q = zeros(1,3);                                                     % Generate a vector of joint angles
                    % 
                    % robot.plot(q,'workspace',workspace,'scale',scale)                   % Plot the robot
                    % 
                    % robot.teach;
                    % input('Press enter to continue');
                    % q = robot.getpos();                                                 % Get the joint angles at the current position        
                    % J = robot.jacob0(q);        
                    % J = J(1:3,1:3)                                                      % Grab only the first 3 rows        
                    % display('There is a deliberate problem here. Notice the failure to invert and thus no way to plot vellipse at this pose');
                    % inv(J)    
                    % robot.vellipse(robot.getpos());                                     % Plot the velocity ellipsoid
                    
                case 2 % AANBOT UR10 arm     
    	            L1 = Link('d',0.1273,'a',0,'alpha',pi/2,'offset',0);
                    L2 = Link('d',0,'a',-0.612,'alpha',0,'offset',0);
                    L3 = Link('d',0,'a',-0.5723,'alpha',0,'offset',0);
                    L4 = Link('d',0.163941,'a',0,'alpha',pi/2,'offset',0);
                    L5 = Link('d',0.1157,'a',0,'alpha',-pi/2,'offset',0);
                    L6 = Link('d',0.0922,'a',0,'alpha',0,'offset',0);
                    
                   robotModel = SerialLink([L1 L2 L3 L4 L5 L6],'name','ANNBOT UR10 arm');
                   % 
                   % q = zeros(1,6);       
                   % robot.plot(q);       
                   % robot.teach();
                   % input('Press enter to continue');
                   % 
                   % q = robot.getpos();                                                 % Get the joint angles at the current position        
                   % J = robot.jacob0(q);        
                   % inv(J)    
                   % robot.vellipse(robot.getpos());    
                           
                case 3 % SPIR igus arm
    	            L1 = Link('d',0.09625,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
                    L2 = Link('d',0,'a',0.27813,'alpha',0,'offset',1.2981,'qlim',[deg2rad(-74.3575),deg2rad(105.6425)]);
                    L3 = Link('d',0,'a',0,'alpha',-pi/2,'offset',-2.8689,'qlim',[deg2rad(-90),deg2rad(90)]);
                    L4 = Link('d',0.23601,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-135),deg2rad(135)]);
                    L5 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
                    L6 = Link('d',0.13435,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-135),deg2rad(135)]);
                    
                    robotModel = SerialLink([L1 L2 L3 L4 L5 L6],'name','SPIR Igus arm');
                    
                    % q = zeros(1,6);        
                    % igus.plot(q);
                    % igus.teach();
                    % input('Press enter to continue');
                    % q = igus.getpos();                                                 % Get the joint angles at the current position        
                    % J = igus.jacob0(q);        
                    % inv(J)    
                    % igus.vellipse(igus.getpos());                                                
                    
                case 4 %bad practicfe
                    r = Sawyer;
                    robotModel = r.model
                    % warning('Sawyer Robot (7DOF) ( No solution since this is Lab Assignment #1 question)');
            end
            % Set the size of the workspace
            roughMinMax = sum(abs(robotModel.d)+abs(robotModel.a));
            workspace = [-roughMinMax roughMinMax -roughMinMax roughMinMax -0.01 roughMinMax];
            scale = 0.5;

            % Creates a vector of initial joint angles 
            q = zeros(1,robotModel.n); 

            % Plot the robot model
            robotModel.plot(q,'workspace',workspace,'scale',scale);
            
            %1.3) Use teach to change the q variable (i.e. the values for each joint), and check that the model matches the images provided. 
            robotModel.teach(q);
            input('USe teach to move robot into a new position then press enter to continue');
            %1.4) Get the current joint angles in radians from the current plot of the model.
            q = robotModel.getpos()

            %1.5) Calculate the transformation matrix of the end effector at a particular joint angle, q, using:
            T = robotModel.fkine(q)

            %1.6) Reverse this and use the end effector transformation, T, and find the joint angles, q.
            if robotModel.n == 6 
                q = robotModel.ikine(T); % N.B. DOES NOT WORK FOR 3DOF MANIPULATORS
            end
            %1.7) Get the Jacobian matrix in the base frame, which maps joint velocities to end-effector velocities.
            J = robotModel.jacob0(q);
            
            %1.8) Try and invert the Jacobian, both with and without the above command, J(1:3,1:3), which limits the Jacobian to the first 3 joint velocities and translational velocities for the end effector.
            try
                inv(J)
            catch ME_1
                disp(ME_1)
            end
            
            % Select the first 3 rows and columns to show how the fist 3
            % joints affect the end effector position
            try
                J = J(1:3,1:3); % For the 3-Link robots, we only need the first 3 rows. You should try it both with and without for the 6DOF robots.
                inv(J)   
                % 1.10) You can visualise how fast the end-effector can move in Cartesian space with the following command:
                robotModel.vellipse(q);
            catch ME_1
                disp(ME_1)
            end   
            %1.9) The Jacobian can't be inverted at certain joint configurations (potentially like the one below). Check to see if there are other configurations where this happens. 
            % q = zeros(1,robotModel.n)
            % J = robotModel.jacob0(q)
            % inv(J)
            % robotModel.vellipse(q);
        end
        
        %% Exercise 2
        % 2.1 Download the PDF of the robot from UTSOnline
        
        % 2.2 Determine the D&H parameters based upon the link measurements on the PDF
        % & 2.3 Determine and include the joint limits in your model
        function Question2()
            profile clear;
            profile on;

            L1=Link('alpha',-pi/2,'a',0.180, 'd',0.475, 'offset',0, 'qlim',[deg2rad(-170), deg2rad(170)]);
            L2=Link('alpha',0,'a',0.385, 'd',0, 'offset',-pi/2, 'qlim',[deg2rad(-90), deg2rad(135)]);
            L3=Link('alpha',pi/2,'a',-0.100, 'd',0, 'offset',pi/2, 'qlim',[deg2rad(-80), deg2rad(165)]);
            L4=Link('alpha',-pi/2,'a',0, 'd',0.329+0.116, 'offset',0, 'qlim',[deg2rad(-185), deg2rad(185)]);
            L5=Link('alpha',pi/2,'a',0, 'd',0, 'offset',0, 'qlim',[deg2rad(-120), deg2rad(120)]);
            L6=Link('alpha',0,'a',0, 'd',0.09, 'offset',0, 'qlim',[deg2rad(-360), deg2rad(360)]);
                
            densoRobot = SerialLink([L1 L2 L3 L4 L5 L6],'name','Denso VM6083G');
            densoRobot.name = 'Denso VM6083G';
            % Use glyphs to draw robot, don't display the name
            densoRobot.plotopt = {'nojoints', 'noname', 'noshadow', 'nowrist'}; %
                
            % 2.4 Sample the joint angles within the joint limits at 30 degree increments between each of the joint limits
            % & 2.5 Use fkine to determine the point in space for each of these poses, so that you end up with a big list of points
            stepRads = deg2rad(30);
            qlim = densoRobot.qlim;
            % Don't need to worry about joint 6
            pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            % If using fkine and passing the whole trajectory
            qAll = zeros(pointCloudeSize,6);
            
            % precallulate the DH transforms for an RRRRRR robot for speed
            baseTr = densoRobot.base.T;
            for i = 1:densoRobot.n
                % From the Canvas learning module on DH parameters
                T_zd = [1,0,0,0;0,1,0,0;0,0,1,densoRobot.links(i).d;0,0,0,1];
                T_xa = [1,0,0,densoRobot.links(i).a;0,1,0,0;0,0,1,0;0,0,0,1];
                cosAlpha = cos(densoRobot.links(i).alpha);
                sinAlpha = sin(densoRobot.links(i).alpha);
                T_Rx = [1,0,0,0;0,cosAlpha,-sinAlpha,0;0,sinAlpha,cosAlpha,0;0,0,0,1];
                transl_d_x_transl_a_x_trotx_alpha{i} = T_zd * T_xa * T_Rx;
                thetaOffset(i) = densoRobot.links(i).offset;
            end
            densoRobotTool = densoRobot.tool.T;
            %% Go through all the poses

            tic
            
            for q1 = qlim(1,1):stepRads:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                % Don't need to worry about joint 6, just assume it=0
                                q6 = 0;
            %                     for q6 = qlim(6,1):stepRads:qlim(6,2)
                                    q = [q1,q2,q3,q4,q5,q6];
                                    tr = densoRobot.fkine(q);                        
                                    pointCloud(counter,:) = tr.t(:,1)';
                                    counter = counter + 1; 
                                    if mod(counter/pointCloudeSize * 100,1) == 0
                                        display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                                    end
            %                     end
                            end
                        end
                    end
                end
            end
            
            % 2.6 Create a 3D model showing where the end effector can be over all these samples.  
            plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
            % profile off;
            % profile viewer;
            
        end
        function Question3()
            % 2.7: (Bonus) Get the start of the blast using fkine (to get end effector transform)
            clf
            densoRobot = DensoVM6083;
            q = [0,pi/2,0,0,0,0];
            densoRobot.model.animate(q);
            blastStartTr = densoRobot.model.fkine(q).T;
            % blastStartVariable = blastStartTr.T;
            blastStartPnt = blastStartTr(1:3,4)';
            
            % 2.8 (Bonus) Get the end of the stream with TR * transl (blast stream length (i.e. 1m along z) 
            blastEndTr = densoRobot.model.fkine(q).T * transl(0,0,1);
            % blastEndVariable = blastEndTr.T * transl(0,0,1);
            blastEndPnt = blastEndTr(1:3,4)';
            
            % 2.9	(Bonus) Project a line out of the end effector (i.e. a mock grit-blasting stream)
            hold on;
            blastPlot_h = plot3([blastStartPnt(1),blastEndPnt(1)],[blastStartPnt(2),blastEndPnt(2)],[blastStartPnt(3),blastEndPnt(3)],'r');
            axis equal;
            
            % 2.10 (Bonus) Create a surface plane that goes through [1.5,0,0] with a normal [-1,0,0]
            planeXntersect = 1.5;
            planeBounds = [planeXntersect-eps,planeXntersect+eps,-2,2,-2,2]; 
            [Y,Z] = meshgrid(planeBounds(3):0.1:planeBounds(4),planeBounds(5):0.1:planeBounds(6));
            X = repmat(planeXntersect,size(Y,1),size(Y,2));
            surf(X,Y,Z);
            
            % 2.11 (Bouns) Determine if and where the blast stream intersects with the surface plane
            planePnt = [1.5,0,0];
            planeNormal = [-1,0,0];
            
            [intersectionPoints,check] = LinePlaneIntersection(planeNormal,planePnt,blastStartPnt,blastEndPnt);
            if check == 1
                intersectionPointPlot_h = plot3(intersectionPoints(:,1),intersectionPoints(:,2),intersectionPoints(:,3),'g*');
            end
            
            jointMidRadians = sum(densoRobot.model.qlim,2)'/2;
            
            % (Bonus)(Bonus) Move the robot around some random joint states, keeping
            % joint 1 == 0 (so the robot basically faces the wall)
            for i = 1:100
                % Pick a random joint configuration with some joints set and other close to 0s and move arm there
                goalQ = [0,pi/2,0,jointMidRadians(4:6) + 0.5 * (rand(1,3)-0.5) .* (densoRobot.model.qlim(4:end,2)' - densoRobot.model.qlim(4:end,1)')];
                
                % Get a trajectory
                jointTrajectory = jtraj(densoRobot.model.getpos(),goalQ,20);
                for trajStep = 1:size(jointTrajectory,1)
                    q = jointTrajectory(trajStep,:);
                    densoRobot.model.animate(q);    
                        
                    blastStartTr = densoRobot.model.fkine(q);
                    blastStartVariable = blastStartTr.T;
                    blastStartPnt = blastStartVariable(1:3,4)';
                    blastEndTr = densoRobot.model.fkine(q);
                    blastEndVariable = blastEndTr.T * transl(0,0,1);
                    blastEndPnt = blastEndVariable(1:3,4)';
            
                    [intersectionPoints(end+1,:),check] = LinePlaneIntersection(planeNormal,planePnt,blastStartPnt,blastEndPnt);
                    if check == 1 ...
                    && all(planeBounds([1,3,5]) < intersectionPoints(end,:)) ... 
                    && all(intersectionPoints(end,:) < planeBounds([2,4,6]))
                        try delete(intersectionPointPlot_h);end
                        intersectionPointPlot_h = plot3(intersectionPoints(:,1),intersectionPoints(:,2),intersectionPoints(:,3),'g*');
                        intersectionPoints(end,:)
                        blastEndPnt = intersectionPoints(end,:);            
                    end
                    % Now plot the stream
                    try delete(blastPlot_h);end
                    blastPlot_h = plot3([blastStartPnt(1),blastEndPnt(1)] ...
                                           ,[blastStartPnt(2),blastEndPnt(2)] ...
                                           ,[blastStartPnt(3),blastEndPnt(3)],'r');
                    drawnow();
                    pause(0.2);
                end
            end
        end
       
    %% LinePlaneIntersection
        % Given a plane (normal and point) and two points that make up another line, get the intersection
        % Check == 0 if there is no intersection
        % Check == 1 if there is a line plane intersection between the two points
        % Check == 2 if the segment lies in the plane (always intersecting)
        % Check == 3 if there is intersection point which lies outside line segment
        function [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)

        intersectionPoint = [0 0 0];
        u = point2OnLine - point1OnLine;
        w = point1OnLine - pointOnPlane;
        D = dot(planeNormal,u);
        N = -dot(planeNormal,w);
        check = 0; %#ok<NASGU>
        if abs(D) < 10^-7        % The segment is parallel to plane
            if N == 0           % The segment lies in plane
                check = 2;
                return
            else
                check = 0;       %no intersection
                return
            end
        end

        %compute the intersection parameter
        sI = N / D;
        intersectionPoint = point1OnLine + sI.*u;

        if (sI < 0 || sI > 1)
            check= 3;          %The intersection point  lies outside the segment, so there is no intersection
        else
            check=1;
        end
        end
    end
end  

