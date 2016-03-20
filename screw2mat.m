function MAT=screw2mat(SC,a)

% SCREW2MAT maps twist SC and parameter a to matrix representation by means of exponential mapping.
%	-SC is a 6X1 column vector 
%	-a is screw parameter 
%	-MAT is the exponential mapping of screw SC 

sSC = size(SC);
if sSC == [1 6], SC = SC'; sSC = size(SC); end

% wrong format
if sSC(1) ~= 6
    error('screw2mat:wrongsize %d rows in the SC array. It should be 6.',sSC(1));
end

rot=SC(1:3);
trans=SC(4:6);
%angular anti-symmetric matrix
A=sym(zeros(3,3));
A(1,2)=-rot(3);
A(1,3)=rot(2);
A(2,3)=-rot(1);
A(2,1)=-A(1,2);
A(3,1)=-A(1,3);
A(3,2)=-A(2,3);

A2=A*A;

%Rodrigues formula
eA=sym(eye(3))+sin(a)*A+(1-cos(a))*A2;

B=cross(rot,trans);
C=(sym(eye(3))-eA)*B;
D=a*rot*rot'*trans;
V=C+D;

MAT = sym(eye(4));
MAT(1:3,1:3)=eA;
MAT(1:3,4)=V;
