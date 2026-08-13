function C = quatToDCM(q)
% Convert scalar-first Hamilton quaternion q_IB to direction cosine matrix C_IB.
% v_I = C_IB * v_B

    q = quatNormalize(q);
    
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    C = [
        1 - 2*(qy^2 + qz^2),  2*(qx*qy - qw*qz),      2*(qx*qz + qw*qy)
        2*(qx*qy + qw*qz),    1 - 2*(qx^2 + qz^2),    2*(qy*qz - qw*qx)
        2*(qx*qz - qw*qy),    2*(qy*qz + qw*qx),      1 - 2*(qx^2 + qy^2)
    ];

end