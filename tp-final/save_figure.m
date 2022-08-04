function save_figure(timestep, fig, type)
    if rem(timestep, 2) == 0 || timestep == 1
        filename = sprintf('plots/%s/%s_%03d.png', type, type, timestep);
        print(fig, filename, '-dpng');
    end
end

