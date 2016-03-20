ur5drawinit

%% Simulation
threshold = 0.01;
startingJoints = [pi/2, pi/2, 0, pi/2, 0, pi/2];

res = vrep.simxPauseCommunication(id, true);
vrchk(vrep, res);
%% The destination plane and point
normal=[2;3;4];
point=[0.5;0.5;0.3];

% The initial plane where the trajectory was defined
norg=[0;-1;0];

%% Getting the rotation matrix
Rot=fRot(norg,normal);
Rot=horzcat(Rot,[0;0;0]);
Rot=vertcat(Rot,[0,0,0,1]);
pause(1);

%% Getting the translation matrix
trans=horzcat(eye(3,3),point);
trans=vertcat(trans,[0,0,0,1]);
pause(1);

%% Set the arm to its starting configuration:
for i = 1:6,
    res = vrep.simxSetJointTargetPosition(id, handles.ur5Joints(i),...
        startingJoints(i),...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end
res = vrep.simxPauseCommunication(id, false);
vrchk(vrep, res);

% Initial point at origin
a=0;
b=0;

%% Generating trajectories (8 in number):

for t=0:0.1:1
    
    x=a-(0.04*t);
    z=b+0.015;
    
Tdes=Rot*P(x,z);% rotation
Tdes=trans*Tdes;% translation
 
 % To create dummies and to move robot to target position
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
display('1over');

%%
for t=3.14:-0.314:1.57
    
    x=a-(0.01*2.5)+(0.015*cos(t));
    z=b+(0.015*sin(t))+0.015;
    
Tdes=Rot*P(x,z);% rotation
Tdes=trans*Tdes;% translation
 
 % To create dummies and to move robot to target position
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
copyf(id,vrep,Tdes,handles.dummy,-1); 
end

display('2over');
%%


for t=3.14:-0.314:0
    
    x=a+(0.01*2.5*cos(t));
    z=b+0.015+(0.01*2.5*sin(t))+0.015;
    
 Tdes=Rot*P(x,z);% rotation
Tdes=trans*Tdes;% translation
 
 % To create dummies and to move robot to target position
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
display('3over');
%%
for t=1.57:-0.314:0
    
    x=a+0.025+(0.015*cos(t));
    z=b+(0.015*sin(t))+0.015;
    

Tdes=Rot*P(x,z);% rotation
Tdes=trans*Tdes;% translation
 
 % To create dummies and to move robot to target position
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

display('4over');
%%
for t=0:0.1:1
    
    x=a+0.04-(0.04*t);
    z=b+0.015;
    

Tdes=Rot*P(x,z);% rotation
Tdes=trans*Tdes;% translation
 
 % To create dummies and to move robot to target position
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
display('5over');

%%
for t=0:0.2:1.0
    
    x=a+0.04-(0.04*t);
    z=b-(0.08*t)+0.015;
Tdes=Rot*P(x,z);% rotation
Tdes=trans*Tdes;% translation
 
 % To create dummies and to move robot to target position
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
display('6over');


%%
for t=0:0.2:1.0
    
    x=a-(0.04*t);
    z=b+(0.08*t)-0.08+0.015;
Tdes=Rot*P(x,z);% rotation
Tdes=trans*Tdes;% translation
 
 % To create dummies and to move robot to target position
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
display('7over');

%%
for t=0:0.628:6.28
    
    x=a+(0.01*cos(t));
    z=b+0.05+(0.01*sin(t))+0.015;
    

Tdes=Rot*P(x,z);% rotation
Tdes=trans*Tdes;% translation
 
 % To create dummies and to move robot to target position
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
display('8over');