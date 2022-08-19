function [controller, pathWorld] = getNewPlan(controller, map, startPose, nextGoal, vigilancia)
    startCell = world2grid_tito(map, startPose);
    goalCell = world2grid_tito(map, nextGoal);
    [~, pathWorld] = planning(startCell, goalCell, map, vigilancia);
    controller = controller.setNavigationPath(pathWorld);
    controller.shouldPlan = false;
end
