function R=fRot(v1, v2)
% R*v1=v2
% v1 and v2 should be 3x1 vectors

% 1. rotation vector
w=cross(v1,v2);
w=w/norm(w);
w_hat=GetSkew(w);
% 2. rotation angle
cos_tht=v1'*v2/norm(v1)/norm(v2);
tht=acos(cos_tht);
% 3. rotation matrix, using Rodrigues' formula
R=eye(size(v1,1))+w_hat*sin(tht)+w_hat^2*(1-cos(tht));

function x_skew=GetSkew(x)
x_skew=[0 -x(3) x(2);
 x(3) 0 -x(1);
 -x(2) x(1) 0];
