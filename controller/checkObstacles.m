function obstacleDetected = checkObstacles(scans, obstacleDistance, checkLeft)
    ranges = scans.Ranges;
    angles = scans.Angles;
    anglesCheck = pi/3;
    delta = deg2rad(10);
    % Miramos en un angulo de +-pi/3 a ver si detectamos algún obstáculo
    if checkLeft
        indexes = find(angles >= -delta);
    else
        indexes = find(angles <= delta);
    end
    
%     indexes = find(angles >= -anglesCheck & angles < anglesCheck);

    rangesInFrontAngle = ranges(indexes);
    
    filteredRanges = rangesInFrontAngle(rangesInFrontAngle < obstacleDistance);

    if numel(filteredRanges) > 0
        obstacleDetected = true;
    	return
    end
        
    obstacleDetected = false;
end