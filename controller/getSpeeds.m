function [v_cmd, w_cmd] = getSpeeds(actualPose, desiredPose, sampleTime)
    w_cmd = 0;
    v_cmd = 0;

    dontRotateLimit = deg2rad(2);

    x = actualPose(1);
    y = actualPose(2);
    theta = actualPose(3);
    
    desiredX = desiredPose(1);
    desiredY = desiredPose(2);
    
    % 1. Nos fijamos cuánto hay que rotar
    desiredOrientation = atan2(desiredY - y, desiredX - x); 
    if theta < 0
        theta = theta + 2*pi;
    end
   
    rotationNeeded = desiredOrientation-theta;
        
    w_cmd = rotationNeeded/sampleTime;
    % 1.1. Si hay que rotar a más de 0.5 rad/s, lo limitamos
    if abs(w_cmd) > 0.5
        w_cmd = sign(w_cmd)*0.5;
    end
    
    if abs(rotationNeeded) < deg2rad(30)
        w_cmd = sign(w_cmd)*0.1;
    end
    % 1.2. Si la rotación es muy pequeña, no rotamos, hacemos movimiento
    % rectilineo
    if abs(rotationNeeded) < dontRotateLimit
        w_cmd = 0;
        distanceToDesiredPose = norm(desiredPose(1:2)-actualPose(1:2));
        v_cmd = distanceToDesiredPose/sampleTime;
        if v_cmd > 0.1
            v_cmd = 0.1;
        end
    end
end
