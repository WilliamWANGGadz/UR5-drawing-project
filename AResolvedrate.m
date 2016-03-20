ur5drawinit
%% Simulation
threshold = 0.01;
startingJoints = [pi/2, pi/2, 0, pi/2, 0, pi/2];

res = vrep.simxPauseCommunication(id, true);
vrchk(vrep, res);

%% The destination plane and point
normal=[0.01;-0.99;0.01];
point=[-0.096;-0.263;0.3];

%The trajectory obtained in Matlab for an image
W=0.0001*[0,0;-23,47;-44,88;-64,129;-88,166;-115,193;-142,224;-180,244;-224,244;-261,227;-298,203;-322,170;-360,190;-394,210;-427,241;-475,254;-519,261;-553,254;-587,227;-604,193;-611,156;-601,115;-587,88;-574,64;-567,44;-597,54;-628,74;-658,102;-689,112;-723,125;-747,146;-771,173;-801,203;-818,237;-845,268;-859,302;-872,339;-879,387;-883,424;-876,465;-866,499;-849,533;-825,567;-798,587;-781,614;-743,642;-709,662;-686,682;-648,710;-608,727;-563,747;-512,764;-475,774;-434,795;-397,801;-377,811;-363,811;-387,798;-407,764;-411,730;-417,693;-417,645;-400,608;-383,584;-370,560;-346,543;-312,526;-271,526;-234,540;-197,543;-176,560;-142,570;-139,584;-125,597;-122,618;-115,648;-101,672;-101,706;-101,747;-95,791;-98,832;-81,835;-74,805;-71,788;-61,774;-57,744;-40,754;-6,754;7,754;28,747;38,761;48,778;68,795;72,815;79,828;89,791;89,754;85,720;96,682;102,652;113,625;133,591;153,570;181,543;218,523;248,523;289,536;333,553;367,567;388,594;408,631;408,659;408,686;398,706;388,737;388,761;371,788;357,805;374,808;408,801;425,795;466,781;493,771;527,757;564,744;595,716;625,713;656,696;690,682;721,655;741,635;765,608;799,581;826,547;846,519;856,499;877,462;887,414;880,390;880,360;870,336;856,309;843,285;829,254;802,227;778,207;758,180;724,156;697,129;673,102;646,91;615,74;595,54;571,37;558,37;561,54;578,85;592,108;592,139;592,173;581,197;561,227;541,241;507,248;469,244;449,241;418,224;391,210;364,197;344,186;330,176;327,163;306,186;286,210;265,227;242,241;218,241;191,241;164,241;140,231;116,214;106,197;82,176;72,159;58,139;48,115;38,85;24,71;14,40;4,27];

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
targetJointss = theta(:,1)'+startingJoints;
for i = 1:6
    if targetJointss(i) > pi
        targetJointss(i) = targetJointss(i) - 2*pi;
    elseif targetJointss(i) < -pi
        targetJointss(i) = targetJointss(i) + 2*pi;
    end
end
for j = 1:6,
    vrep.simxSetJointTargetPosition(id, handles.ur5Joints(j),...
        targetJointss(j), ...
        vrep.simx_opmode_oneshot);     
    vrchk(vrep, res);
end
%check if point has been reached
        cJoints = zeros(1,6);    
        reached = false;
        while ~reached
            for i = 1:6
                [returnCode,cJoints(i)] = vrep.simxGetJointPosition(id, handles.ur5Joints(i),...
                    vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
            end
            Gc = ur5fwdtrans(cJoints-startingJoints, 6);
            dJoints = cJoints - targetJointss;
            for i = 1:6
                if dJoints(i) > pi
                    dJoints(i) = dJoints(i) - 2*pi;
                elseif dJoints(i) < -pi
                    dJoints(i) = dJoints(i) + 2*pi;
                end
            end
            if max(abs(dJoints)) < threshold
                reached = true;
            end
        end
display('0over');

U=theta(:,1);  %First theta value
Xact=[0;0;0];  %Initial point in the trajectory


for t=2:2:190
    
    x=W(t,1);
    z=W(t,2);
    
Tdes=Rot*P(x,z);  %rotation
Tdes=trans*Tdes;  %translation
% m=Tdes(1:3,1:3);
Xdes=[x;0;z];    %next point
X_des=Xdes-Xact; %linear increment required at end-effector
U_v=iJack(U)*X_des;   %angular increment required at joints
U = U+U_v;   %updating thetas
Xact=Xdes;   % updating position
targetJointss = U'+startingJoints;

for i = 1:6
    if targetJointss(i) > pi
        targetJointss(i) = targetJointss(i) - 2*pi;
    elseif targetJointss(i) < -pi
        targetJointss(i) = targetJointss(i) + 2*pi;
    end
end
for j = 1:6,
    vrep.simxSetJointTargetPosition(id, handles.ur5Joints(j),...
        targetJointss(j), ...
        vrep.simx_opmode_oneshot);     
    vrchk(vrep, res);
end
%check if point has been reached
        cJoints = zeros(1,6);    
        reached = false;
        while ~reached
            for i = 1:6
                [returnCode,cJoints(i)] = vrep.simxGetJointPosition(id, handles.ur5Joints(i),...
                    vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
            end
            Gc = ur5fwdtrans(cJoints-startingJoints, 6);
            dJoints = cJoints - targetJointss;
            for i = 1:6
                if dJoints(i) > pi
                    dJoints(i) = dJoints(i) - 2*pi;
                elseif dJoints(i) < -pi
                    dJoints(i) = dJoints(i) + 2*pi;
                end
            end
            if max(abs(dJoints)) < threshold
                reached = true;
            end
        end
copyf(id,vrep,Tdes,handles.dummy,-1); 
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