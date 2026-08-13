function qConj = quatConjugate(q)
% Quaternion conjugate for scalar-first quaternion.

    qConj = [
        q(1)
        -q(2)
        -q(3)
        -q(4)
    ];

end