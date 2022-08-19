function [slamObj, exploredMap, slamPoses] = performSlam(slamObj, scans, initPose)
    slamObj = updateSlamObj(slamObj, scans);
    [exploredMap, slamPoses] = getExploredMapAndPoses(slamObj);
    
    % Este paso no es 100% necesario, pero sirve para la correcta
    % visualización del mapa
    [exploredMap, slamPoses] = moveMapAndPosesToReference(exploredMap, slamPoses, initPose);
end

function slamObj = updateSlamObj(slamObj, scans)
    addScan(slamObj, scans);
end

function [exploredMap, poses] = getExploredMapAndPoses(slamObj)
    [scans, poses]  = scansAndPoses(slamObj);
    exploredMap = buildMap(scans, poses, slamObj.MapResolution, slamObj.MaxLidarRange);
end

function [exploredMap, poses] = moveMapAndPosesToReference(exploredMap, poses, initPose)
    poses(end,:) = poses(end,:) + initPose(:)';
    exploredMap.GridLocationInWorld = exploredMap.GridLocationInWorld + initPose(1:2)';
%     exploredMap.XWorldLimits = [0, 6.04];
%     exploredMap.YWorldLimits = [0, 5.04];
end