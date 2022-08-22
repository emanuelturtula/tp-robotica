function [controller, pathWorld] = getNewPlanForVigilance(controller, map, startPose, nextGoal)
    startCell = world2grid_tito(map, startPose);
    goalCell = world2grid_tito(map, nextGoal);
    [~, pathWorld] = planningForVigilance(startCell, goalCell, map);
    controller = controller.setNavigationPath(pathWorld);
    controller.shouldPlan = false;
end
