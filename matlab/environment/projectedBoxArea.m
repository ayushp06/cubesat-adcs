function area = projectedBoxArea(directionB, dimensions)
% PROJECTEDBOXAREA Projected area of a rectangular box [m^2].
    directionB = directionB / norm(directionB);
    faceAreas = [dimensions(2)*dimensions(3); ...
                 dimensions(1)*dimensions(3); ...
                 dimensions(1)*dimensions(2)];
    area = dot(abs(directionB), faceAreas);
end
