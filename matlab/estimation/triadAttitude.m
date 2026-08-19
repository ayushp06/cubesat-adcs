function qIB = triadAttitude(bodyVectors, inertialVectors)
% TRIADATTITUDE Exact attitude from two non-collinear vector observations.
    b1 = bodyVectors(:,1) / norm(bodyVectors(:,1));
    b2 = cross(b1, bodyVectors(:,2));
    r1 = inertialVectors(:,1) / norm(inertialVectors(:,1));
    r2 = cross(r1, inertialVectors(:,2));
    if norm(b2) < 1e-10 || norm(r2) < 1e-10
        error("triadAttitude:CollinearVectors", "TRIAD requires non-collinear vectors.");
    end
    b2 = b2/norm(b2); r2 = r2/norm(r2);
    bodyTriad = [b1,b2,cross(b1,b2)];
    inertialTriad = [r1,r2,cross(r1,r2)];
    qIB = dcmToQuat(inertialTriad * bodyTriad');
end
