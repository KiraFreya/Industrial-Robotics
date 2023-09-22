clc
clear all
hold on
axis equal
workspace = [-1.5 1.5 -1.5 1.5 -0 4];

%% Debug and Logger
%LOG = log4matlab('logLinearUR3.log');
scriptname = 'Assignment1';
%% DH Parameters of Linear UR3
name = ['LinearUR3',datestr(now,'yyyymmddTHHMMSSFFF')];

L(1) = Link([pi     0         0       pi/2    1]); % PRISMATIC Link (Linear Rail)
L(2) = Link([0      0.1519    0       pi/2    0]);
L(3) = Link([0      0     -0.24365     0      0]);
L(4) = Link([0      0     -0.21325     0      0]);
L(5) = Link([0      0.11235   0       pi/2    0]);
L(6) = Link([0      0.08535   0      -pi/2	  0]);
L(7) = Link([0      0.0819    0        0      0]);

%Linear Rail limits
L(1).qlim = [-0.8 0];
L(2).qlim = [-360 360]*pi/180;
L(3).qlim = [-90 90]*pi/180;
L(4).qlim = [-170 170]*pi/180;
L(5).qlim = [-360 360]*pi/180;
L(6).qlim = [-360 360]*pi/180;
L(7).qlim = [-360 360]*pi/180;

L(3).offset = -pi/2;
L(5).offset = -pi/2;

scale = 0.5;
q = zeros(1,7);
base = eye(4);

LinearUR3 = SerialLink(L,'name',name,'base',base);
LinearUR3.base = LinearUR3.base*transl(0.5,0.5,0.32)*trotx(pi/2)*troty(-pi);
display(['Load DH Paramaters for Linear UR3 Complete']);

%% Debug Test
if LinearUR3.base(3,4) ~= 0.32
%    LOG.mlog = {LOG.Warn,scriptname,'Base of Linear UR3 at wrong height!'};
end

%% Texture for Linear UR3
for linkIndex = 0:LinearUR3.n
    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR3Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
    LinearUR3.faces{linkIndex+1} = faceData;
    LinearUR3.points{linkIndex+1} = vertexData;
end

LinearUR3.plot3d(zeros(1,LinearUR3.n),'noarrow','workspace',workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
LinearUR3.delay = 0;

for linkIndex = 0:LinearUR3.n
    handles = findobj('Tag', LinearUR3.name);
    h = get(handles,'UserData');
    try
        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
            , plyData{linkIndex+1}.vertex.green ...
            , plyData{linkIndex+1}.vertex.blue]/255;
        h.link(linkIndex+1).Children.FaceColor = 'interp';
    catch ME_1
        disp(ME_1);
        continue;
    end
end

display(['Load Texture for LinearUR3 Complete']);
%% DH Parameters of Gripper
name1 = ['GripperL',datestr(now,'yyyymmddTHHMMSSFFF')];
name2 = ['GripperR',datestr(now,'yyyymmddTHHMMSSFFF')];

Lg(1) = Link([0      0      0.05      0      0]);
Lg(2) = Link([0      0      0.045      0      0]);
Lg(3) = Link([0      0      0.045       0      0]);

Lg(1).qlim = [-90 90]*pi/180;
Lg(2).qlim = [-90 90]*pi/180;
Lg(3).qlim = [-90 90]*pi/180;

Lg(1).offset = -16*pi/180;
Lg(2).offset = 58*pi/180;
Lg(3).offset = 48*pi/180;

gripperbase = eye(4);

%PlaceObject('gripper.ply',[0,0,0])

GripperL = SerialLink(Lg,'name',name1,'base',gripperbase);
GripperR = SerialLink(Lg,'name',name2,'base',gripperbase);
GripperL.base = LinearUR3.fkine(LinearUR3.getpos)*trotx(pi/2);
GripperR.base = LinearUR3.fkine(LinearUR3.getpos)*trotz(pi)*trotx(pi/2);

display(['Load DH parameters for the Gripper Complete']);
%% Texture of Left Gripper
for linkIndex = 0:GripperL.n
    [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
    GripperL.faces{linkIndex + 1} = faceData;
    GripperL.points{linkIndex + 1} = vertexData;
end

% Display robot
GripperL.plot3d(zeros(1,GripperL.n),'noarrow','workspace',workspace);
% view(3)
% drawnow()
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
GripperL.delay = 0;

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:GripperL.n
    handles = findobj('Tag', GripperL.name);
    h = get(handles,'UserData');
    try
        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
            , plyData{linkIndex+1}.vertex.green ...
            , plyData{linkIndex+1}.vertex.blue]/255;
        h.link(linkIndex+1).Children.FaceColor = 'interp';
    catch ME_1
        disp(ME_1);
        continue;
    end
end

display(['Load Texture for Left Gripper Complete']);
%% Texture of Right Gripper
for linkIndex = 0:GripperR.n
    [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
    GripperR.faces{linkIndex + 1} = faceData;
    GripperR.points{linkIndex + 1} = vertexData;
end

% Display robot
GripperR.plot3d(zeros(1,GripperR.n),'noarrow','workspace',workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
GripperR.delay = 0;

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:GripperR.n
    handles = findobj('Tag', GripperR.name);
    h = get(handles,'UserData');
    try
        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
            , plyData{linkIndex+1}.vertex.green ...
            , plyData{linkIndex+1}.vertex.blue]/255;
        h.link(linkIndex+1).Children.FaceColor = 'interp';
    catch ME_1
        disp(ME_1);
        continue;
    end
end
LinearUR3.delay = 0;
GripperL.delay = 0;
GripperR.delay = 0;
display(['Load Texture for Right Gripper Complete']);
%% Qmatrix for Gripper animation
qopen = zeros(1,3);
qclose = [-0.2513    0.6912   -0.4398];
qmatc = jtraj(qopen,qclose,100);
qmato = jtraj(qclose,qopen,100);

display(['Calculate Qmatrix for Gripper animation Complete']);
%% Brick Original Placement
[f,v,data] = plyread('HalfSizedRedGreenBrick.ply','tri');
VCount = size(v,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

BrickSLoc = [0.01,-0.15,0.32;    -0.01,0,0.32;   0,0.15,0.32; ...
    0,0.3,0.32;  0,0.45,0.388;  0,0.45,0.354; ...
    0,0.45,0.32;   0,0.6,0.354;  0,0.6,0.32];
for i=1:9
    Brick_h(i) = trisurf(f,v(:,1)+BrickSLoc(i,1),v(:,2)+ BrickSLoc(i,2), v(:,3)+ BrickSLoc(i,3) ...
        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    BrickSTR(:,:,i) = transl(BrickSLoc(i,1),BrickSLoc(i,2),BrickSLoc(i,3)+0.033)*troty(pi);
end
display(['Generate Starting Brick Transformation Location and Model Complete']);
%% Brick Final Placements
BrickFLoc = [0.15,0,0.32;        0.15,0.135,0.32;        0.15,0.27,0.32; ...
    0.15,0,0.354;       0.15,0.135,0.354;       0.15,0.27,0.354; ...
    0.15,0,0.388;       0.15,0.135,0.388;       0.15,0.27,0.388];
for i=1:9
    BrickFTR(:,:,i) = transl(BrickFLoc(i,1),BrickFLoc(i,2),BrickFLoc(i,3)+0.033)*troty(pi);
end
display(['Generate Final Brick Transformation Location Complete']);
%% Environment
hold on
axis equal
axis on
PlaceObject('environment.ply',[0,0,0]);

display(['Generate Environment Complete']);
%% Rearranging the bricks
display(['Building Wall - Start']);
q3 = zeros(1,7);
initialQ = [-0.3838,0 ,1.2566,0.8308,-0.5027,-1.5708,-1.2566];
for j = 1:9
    display(['Moving Brick ',num2str(j)]);
    q1 = q3;
    q2 = LinearUR3.ikcon(BrickSTR(:,:,j),initialQ);
    qMatrix = jtraj(q1,q2,100);
    q3 = LinearUR3.ikcon(BrickFTR(:,:,j),initialQ);
    qMatrix2 = jtraj(q2,q3,100);
    
    % Going to brick initial pos
    display(['Moving End Effector to Brick ',num2str(j),' initial position']);
    for i = 1:100
        pause(0.001);
        LinearUR3.animate(qMatrix(i,:))
        GripperL.base = LinearUR3.fkine(LinearUR3.getpos)*trotx(pi/2);
        GripperR.base = LinearUR3.fkine(LinearUR3.getpos)*trotz(pi)*trotx(pi/2);
        GripperL.animate(GripperL.getpos);
        GripperR.animate(GripperL.getpos);
    end
    
    % Picking up brick
    display(['Picking Up Brick ',num2str(j)]);
    for i = 1:100
        pause(0.001)
        GripperL.animate(qmatc(i,:));
        GripperR.animate(qmatc(i,:));
    end
    EndEffector = LinearUR3.fkine(LinearUR3.getpos);
    updatedPoints = [EndEffector * [v,ones(VCount,1)]']';
    Brick_h(j).Vertices = updatedPoints(:,1:3);
    drawnow();
    
    %Going to brick final pos
    display(['Moving Brick ',num2str(j),' to final position']);
    for i = 1:100
        pause(0.001);
        LinearUR3.animate(qMatrix2(i,:))
        GripperL.base = LinearUR3.fkine(LinearUR3.getpos)*trotx(pi/2);
        GripperR.base = LinearUR3.fkine(LinearUR3.getpos)*trotz(pi)*trotx(pi/2);
        GripperL.animate(GripperL.getpos);
        GripperR.animate(GripperL.getpos);
        EndEffector = LinearUR3.fkine(LinearUR3.getpos);
        updatedPoints = [EndEffector * [v,ones(VCount,1)]']';
        Brick_h(j).Vertices = updatedPoints(:,1:3);
        drawnow();
    end
    % dropping off brick
    display(['Dropping off Brick ',num2str(j)]);
    for i = 1:100
        pause(0.001)
        GripperL.animate(qmato(i,:));
        GripperR.animate(qmato(i,:));
    end
    updatedPoints = [BrickFTR(:,:,j) * [v,ones(VCount,1)]']';
    Brick_h(j).Vertices = updatedPoints(:,1:3);
    drawnow();
end

display(['Building Wall - Complete']);
%% Point Cloud
stepRads = deg2rad(50);
qlim = LinearUR3.qlim;
linearsteps = abs(qlim(1,1))/((360*2)/rad2deg(stepRads)); %change rad to linear steps


% Assign the storage required for the pointCloud as it will more computationally efficienct if we do
pointCloudeSize = prod(floor((qlim(1:7,2)-qlim(1:7,1))/stepRads + 1));
pointCloud = zeros(pointCloudeSize,3);
counter = 1;
tic

% We iterate through 6 for loops to store position of the end effector, noting q7 doesn't affect workspace volume
for q1 = qlim(1,1):linearsteps:qlim(1,2)
    for q2 = qlim(2,1):stepRads:qlim(2,2)
        for q3 = qlim(3,1):stepRads:qlim(3,2)
            for q4 = qlim(4,1):stepRads:qlim(4,2)
                for q5 = qlim(5,1):stepRads:qlim(5,2)
                    for q6 = qlim(6,1):stepRads:qlim(6,2)
                        q7=0;
                        q = [q1,q2,q3,q4,q5,q6,q7];
                        tr = LinearUR3.fkine(q); %using joint positions, find end effector transform and store the x,y,z location
                        pointCloud(counter,:) = tr(1:3,4)'; % the apostrophe is to transpose the matrix in the form the point cloud is expecting
                        counter = counter + 1;
                        if mod(counter/pointCloudeSize * 100,1) == 0 %to show messages, it outputs every 25s to give time indication of how long its gone for
                            display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                        end
                    end
                end
            end
        end
    end
end
% Create a 3D model showing where the end effector can be over all these samples.
plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
display(['Calculate and Display Point Cloud Complete']);
%% Volume 
radius = 0.5;
length = 0.8;
Volume = ((4/3)*pi*radius^3 + pi*radius^2*length)/2;
display(['Calculate Workspace Volume = ',num2str(Volume),' m^3 Complete']);