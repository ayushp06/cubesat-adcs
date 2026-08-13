function q = quatMultiply(q1, q2)
% Hamilton product of two scalar-first quaternions.

% q1, q2 = [qw; qx; qy; qz]

    s1 = q1(1);
    v1 = q1(2:4);
    
    s2 = q2(1);
    v2 = q2(2:4);
    
    s = s1*s2 - dot(v1, v2);
    v = s1*v2 + s2*v1 + cross(v1, v2);
    
    q = [s; v];

end