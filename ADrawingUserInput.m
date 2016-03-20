ur5drawinit
%% Simulation
threshold = 0.01;
startingJoints = [pi/2, pi/2, 0, pi/2, 0, pi/2];

res = vrep.simxPauseCommunication(id, true);
vrchk(vrep, res);

%% The destination plane and point
normal=[0.01;-0.99;0.01];
point=[-0.096;-0.263;0.3];

%The initial plane where the trajectory was defined
norg=[0;-1;0];

%% Getting the rotation matrix
Rot=fRot(norg,normal);
Rot=horzcat(Rot,[0;0;0]);
Rot=vertcat(Rot,[0,0,0,1]);

%% Getting the translation matrix
trans=horzcat(eye(3,3),point);
trans=vertcat(trans,[0,0,0,1]);
pause(1);

% Set the arm to its starting configuration:
for i = 1:6,
    res = vrep.simxSetJointTargetPosition(id, handles.ur5Joints(i),...
        startingJoints(i),...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end
res = vrep.simxPauseCommunication(id, false);
vrchk(vrep, res);


%% Move to the new point
x=0; %initially at origin
z=0;
Tdes=Rot*P(0.1*x,0.1*z); % rotation
Tdes=trans*Tdes; %translation

movef( id, vrep, Tdes, handles.FrameEndTarget, handles.base);
theta = ur5inv(Tdes);
targetJointss = wrapTo2Pi(theta(:,1)'+startingJoints);
for j = 1:6,
    vrep.simxSetJointTargetPosition(id, handles.ur5Joints(j),...
        targetJointss(j), ...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res);
end
copyf(id,vrep,Tdes,handles.dummy,-1);

%% Get user input:
scatter(x,z);
axis([-1 1 -1 1]);
hold on
for i=1:1:50
    [x, z, key] = ginput(1);
    scatter(x,z);
    
    if (key == 'e') % To end the drawing
        display('End of cycle.')
        break;
    else
        Tdes=Rot*P(0.1*x,0.1*z); % Rotation
        Tdes=trans*Tdes;   %Translation
        %% To create dummies and to move robot to target position
        movef( id, vrep, Tdes, handles.FrameEndTarget, handles.base);
        theta = ur5inv(Tdes);
        targetJointss = wrapTo2Pi(theta(:,1)'+startingJoints);
        for j = 1:6,
            vrep.simxSetJointTargetPosition(id, handles.ur5Joints(j),...
                targetJointss(j), ...
                vrep.simx_opmode_oneshot);
            vrchk(vrep, res);
        end
        copyf(id,vrep,Tdes,handles.dummy,-1);
    end    
end
%% Go back to starting position
for i = 1:6,
    res = vrep.simxSetJointTargetPosition(id, handles.ur5Joints(i),...
        startingJoints(i),...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end
res = vrep.simxPauseCommunication(id, false);
vrchk(vrep, res);