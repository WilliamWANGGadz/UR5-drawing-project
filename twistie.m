function V=twistie(m)

% Converts skew velocities to velocity vector
    p1=m(1,4);
    p2=m(2,4);
    p3=m(3,4);
    p=[p1;p2;p3];
    w1=-m(2,3);
    w2=m(1,3);
    w3=-m(1,2);
    w=[w1;w2;w3];
    q=cross(p,w);
    
    
    V=[p1;p2;p3;w1;w2;w3];
end