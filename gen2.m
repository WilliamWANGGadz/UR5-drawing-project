%Generates the homogeneous transformation of the tool frame

function G=gen2(jointAngles)

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
w1=[0,0,1];
q1=[0,0,0];
w2=[0,-1,0];
q2=[0,0,d1];
w3=[0,-1,0];
q3=[a2,0,d1];
w4=[0,-1,0];
q4=[a2+a3,0,d1];
w5=[0,0,-1];
q5=[a2+a3,-d4,d1];
w6=[0,-1,0];
q6=[a2+a3,-d4,d1-d5];

% tool frame position and homogeneoud representation (tool frame initially
% aligned with base frame

gst = [1,0,0,a2+a3;0,0,-1,-d4-d6;0,1,0,d1-d5;0,0,0,1];

% obtaining the screws from the axis-point vectors
e1=vec2screw(w1,q1);
e2=vec2screw(w2,q2);
e3=vec2screw(w3,q3);
e4=vec2screw(w4,q4);
e5=vec2screw(w5,q5);
e6=vec2screw(w6,q6);

%converting the screws to homogeneous representation
g1=screw2mat(e1,o1);
g2=screw2mat(e2,o2);
g3=screw2mat(e3,o3);
g4=screw2mat(e4,o4);
g5=screw2mat(e5,o5);
g6=screw2mat(e6,o6);


%the transformed tool frame is given as
G=g1*g2*g3*g4*g5*g6*gst;

end
