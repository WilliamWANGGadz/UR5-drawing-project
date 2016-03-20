function OP=P(x1,z1)

%Generates a homogenous transformation of the points in the defined x-z plane

OP=[1,0,0,x1;0,0,-1,0;0,1,0,z1;0,0,0,1];

end