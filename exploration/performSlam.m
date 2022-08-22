function [slamObj, exploredMap, slamPoses] = performSlam(slamObj, scans)
    exploredMap = 0;
    addScan(slamObj, scans);
    %slamObj = updateSlamObj(slamObj, scans);
    [~, slamPoses]  = scansAndPoses(slamObj);
    %[exploredMap, slamPoses] = getExploredMapAndPoses(slamObj);
    
    % Este paso no es 100% necesario, pero sirve para la correcta
    % visualización del mapa
    %[exploredMap, slamPoses] = moveMapAndPosesToReference(exploredMap, slamPoses);
end

function slamObj = updateSlamObj(slamObj, scans)
    addScan(slamObj, scans);
end

function [exploredMap, poses] = getExploredMapAndPoses(slamObj)
    [scans, poses]  = scansAndPoses(slamObj);
    exploredMap = buildMap(scans, poses, slamObj.MapResolution, slamObj.MaxLidarRange);
end

function [exploredMap, poses] = moveMapAndPosesToReference(exploredMap, poses)
    poses(end,:) = poses(end,:);
    exploredMap.GridLocationInWorld = exploredMap.GridLocationInWorld;
end