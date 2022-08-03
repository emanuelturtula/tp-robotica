function plot_state(particles, pose, est_pos, timestep, particles_ax, particles_fig, print_particles, particle_viz)

if(~isvalid(particles_fig))
    return
end

Rt = 0.35/2;

pose_x = [particles(:).pose_x]';
pose_y = [particles(:).pose_y]';

hold(particles_ax, 'on')
set(particle_viz.handle_particles, 'xdata', pose_x, 'ydata', pose_y);
set(particle_viz.handle_pose, 'xdata', pose(1), 'ydata', pose(2));
set(particle_viz.handle_est, 'xdata', est_pos(1), 'ydata', est_pos(2));
[xc, yc] = internal.circlePoints(est_pos(1), est_pos(2), Rt, 17);
set(particle_viz.handle_robot, 'xdata', xc, 'ydata', yc);
len = 2*Rt;
xp = [est_pos(1), est_pos(1) + (len*cos(est_pos(3)))];
yp = [est_pos(2), est_pos(2) + (len*sin(est_pos(3)))];
set(particle_viz.handle_lidar, 'xdata', xp, 'ydata', yp);
hold(particles_ax, 'off')

if(print_particles)
    save_figure(timestep, particles_fig, 'particles');
end

end