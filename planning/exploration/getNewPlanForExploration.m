function [controller, pathWorld] = getNewPlanForExploration(controller, map, startPose)
    m = copy(map);
    startCell = world2grid(map, startPose(1:2));
    [~, pathWorld] = planningForExploration(startCell, m);
    controller = controller.setNavigationPath(pathWorld);
    controller.shouldPlan = false;
end
