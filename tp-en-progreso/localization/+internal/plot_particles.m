function plot_particles(particles, realPose, estimatedPose, map, particlesFig)
    Rt = 0.35/2;
    poseX = [particles(:).x]';
    poseY = [particles(:).y]';

    particlesAx = axes('Parent', particlesFig);
    particleViz = ParticleViz;
    hold(particlesAx, 'on')
    show(map, 'Parent', particlesAx);
    particleViz.handle_est = plot(particlesAx, 0, 0, 'xg');
    particleViz.handle_particles = plot(particlesAx, poseX, poseY, '.b');
    particleViz.handle_pose = plot(particlesAx, 0, 0, 'xr');
    particleViz.handle_lidar = plot(particlesAx, 0, 0, '-g');
    particleViz.handle_robot = plot(particlesAx, 0, 0, '-g');

    set(particleViz.handle_particles, 'xdata', poseX, 'ydata', poseY);
    set(particleViz.handle_est, 'xdata', estimatedPose(1), 'ydata', estimatedPose(2));
    [xc, yc] = circle_points(estimatedPose(1), estimatedPose(2), Rt, 17);
    set(particleViz.handle_robot, 'xdata', xc, 'ydata', yc);
%             [xc, yc] = circlePoints(realPose(1), realPose(2), Rt, 17);
%             set(particleViz.handle_pose, 'xdata', xc, 'ydata', yc);
    set(particleViz.handle_pose, 'xdata', realPose(1), 'ydata', realPose(2));

    len = 2*Rt;
    xp = [estimatedPose(1), estimatedPose(1) + (len*cos(estimatedPose(3)))];
    yp = [estimatedPose(2), estimatedPose(2) + (len*sin(estimatedPose(3)))];
    set(particleViz.handle_lidar, 'xdata', xp, 'ydata', yp);
    hold(particlesAx, 'off')
end

function [cx,cy] = circle_points(x,y,R,N)
    theta = linspace(-pi,pi,N);
    cx = R*cos(theta) + x;
    cy = R*sin(theta) + y;
end
