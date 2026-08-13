function qNorm = quatNormalize(q)
% Normalize quaternion to unit magnitude.

    n = norm(q);
    
    if n < eps
        error("Quaternion magnitude is too close to zero.");
    end
    
    qNorm = q / n;

end