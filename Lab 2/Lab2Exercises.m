% 1.1 Start at origin and move up by 10m off the ground
tranimate(eye(4), transl([0,0,10]),'fps',25)

% 1.2 Rotate (roll) around the X axis by -30 degrees so the Y axis 
% is pointing more towards the ground than before
trStart = transl([0,0,10]);
trEnd = transl([0,0,10]) * trotx(-30* pi/180);
tranimate(trStart,trEnd,'fps',25);

% 1.3 Move in the direction of global Y to [0,2,10] - 10m to the z
trStart = trEnd;
trEnd =  transl([0,2,10]) * trotx(-30* pi/180);
tranimate(trStart,trEnd,'fps',25);

% 1.4)  Roll back to level (so the orientation is now eye(3))
trStart=trEnd;
trEnd =  transl([0,2,10]);
tranimate(trStart,trEnd,'fps',25);

% 1.5)  Rotate (pitch) around the Y axis by 30 degrees 
% so the X axis is pointing more towards the ground than before
trStart = trEnd;
trEnd = transl([0,2,10]) * troty(30* pi/180);
tranimate(trStart,trEnd,'fps',25);

%1.6) Move in the direction of global X to [2,2,10]
trStart = trEnd;
trEnd = transl([2,2,10]) * troty(30* pi/180);
tranimate(trStart,trEnd,'fps',25);

%1.7) Roll back to level (so the orientation is now %eye(3))
trStart=trEnd;
trEnd =  transl([2,2,10]);
tranimate(trStart,trEnd);

%1.8) Go to the ground so that the new position is [2,2,0]
trStart=trEnd;
trEnd =  transl([2,2,0]);
tranimate(trStart,trEnd,'fps',25);

%1.9) Encode the steps 1.1-1.8 in a %for’ loop and use the 'fps' option in ‘tranimate’ to speed up the animation


%1.10)  Use the text tool from Week 1 Lab to plot in the left hand corner the RPY and quaternion value of the orientation at each step