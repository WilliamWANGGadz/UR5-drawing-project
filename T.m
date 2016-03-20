function [M] = T(o,d,b,a)
%Generates the Denavit Hartenberg Transformation Matrix
%   o-theta,b-alpha
M=[cos(o),-cos(b)*sin(o),sin(b)*sin(o),a*cos(o);
    sin(o),cos(b)*cos(o),-cos(o)*sin(b),a*sin(o);
    0,sin(b),cos(b),d;
    0,0,0,1];
end

