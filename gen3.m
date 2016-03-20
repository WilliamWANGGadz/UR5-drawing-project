%This for TOOLFRAME JACOBIAN

function G=gen3(jointAngles)

%G- the homogeneous reperesentation of the screw of the tool frame
% jointAngles- the 6 x 1 vector of angle of rotations about the six joints

% splitting the individual angles from the vector
theta1=jointAngles(1);  
theta2=jointAngles(2);
theta3=jointAngles(3);
theta4=jointAngles(4);
theta5=jointAngles(5);
theta6=jointAngles(6);

% entering the values for the twist axis,w and point on axis,q
d1 = 0.089159;
    d2 = 0;
    d3 = 0;
    d4 = 0.10915;
    d5 = 0.09465;
    d6 = 0.0823;
    a1 = 0;
    a2 = -0.425;
    a3 = -0.39225;
    a4 = 0;
    a5 = 0;
    a6 = 0;
w6=[0,0,1];
q6=[0,0,0];
w5=[0,1,0];
q5=[0,0,-d6];
w4=[0,0,1];
q4=[0,d5,-d6];
w3=[0,0,1];
q3=[a3,d5,-d6-d4];
w2=[0,0,1];
q2=[a2+a3,d5,-d6-d4];
w1=[0,1,0];
q1=[a2+a3,d5-d1,-d6-d4];

% tool frame position and homogeneoud representation (tool frame initially
% aligned with base frame

gts = [1,0,0,a2+a3;0,0,1,d5-d1;0,-1,0,-d6-d4;0,0,0,1];

% obtaining the screws from the axis-point vectors
e1=vec2screw(w1,q1);
e2=vec2screw(w2,q2);
e3=vec2screw(w3,q3);
e4=vec2screw(w4,q4);
e5=vec2screw(w5,q5);
e6=vec2screw(w6,q6);

%converting the screws to homogeneous representation
g1=screw2mat(e1,theta1);
g2=screw2mat(e2,theta2);
g3=screw2mat(e3,theta3);
g4=screw2mat(e4,theta4);
g5=screw2mat(e5,theta5);
g6=screw2mat(e6,theta6);


%the transformed tool frame is given as
G=g6*g5*g4*g3*g2*g1*gts;

end
