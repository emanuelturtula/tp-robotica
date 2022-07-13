function plot_state(particles, pose, particles_ax, particles_fig, particle_viz)

    if(~isvalid(particles_fig))
        return
    end

     Rt = 0.35/2;

    pose_x = zeros(numel(particles), 1);
    pose_y = zeros(numel(particles), 1);
    for i = 1:numel(particles)
        pose_x(i,1) = particles(i).pose(1);
        pose_y(i,1) = particles(i).pose(2);
    end


    hold(particles_ax, 'on')
    set(particle_viz.handle_particles, 'xdata', pose_x, 'ydata', pose_y);
    set(particle_viz.handle_pose, 'xdata', pose(1), 'ydata', pose(2));
%     set(particle_viz.handle_est, 'xdata', est_pos(1), 'ydata', est_pos(2));
    [xc, yc] = circlePoints(pose(1), pose(2), Rt, 17);
    set(particle_viz.handle_robot, 'xdata', xc, 'ydata', yc);
%     len = 2*Rt;
%     xp = [est_pos(1), est_pos(1) + (len*cos(est_pos(3)))];
%     yp = [est_pos(2), est_pos(2) + (len*sin(est_pos(3)))];
    %set(particle_viz.handle_lidar, 'xdata', xp, 'ydata', yp);
    hold(particles_ax, 'off')

%     if(print_particles)
%         filename = sprintf('plots/particles/particles_%03d.png', timestep);
%         print(particles_fig, filename, '-dpng');
%     end

end

function [cx,cy] = circlePoints(x,y,R,N)

theta = linspace(-pi,pi,N);
cx = R*cos(theta) + x;
cy = R*sin(theta) + y;

end
