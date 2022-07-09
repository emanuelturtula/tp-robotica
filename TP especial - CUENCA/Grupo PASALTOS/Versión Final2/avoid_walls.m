function [v, w] = avoid_walls(ranges, lidar)

v = 0;
w = 0;

angle_div = 2*pi/(length(ranges) - 1);
pastel_count = 0;
pastel_idx = 0;

dist_init = 0.8;
dist_low = 0.5;
dist = dist_init;
dist_min = 0.4;
dist_delta = 0.05;

% caso cercano a la pared
if(min(ranges) < dist_min)
    pastel = pi;
    dist = dist_min;
else
    pastel = pi/2;
end
pastel_n = ceil(pastel/angle_div);
i = -floor((pastel_n - 1)/2);

zero_angle = angle_div*3;
low_angle = angle_div*8;

low_w = 0.3;
high_w = 0.8;
low_v = 0.1;
high_v = 0.3;

ranges(isnan(ranges)) = inf;
while(pastel_idx == 0 && dist >= dist_min)
    while(i < length(ranges) - 1 && pastel_count < pastel_n)
        j = mod(i - 1, length(ranges) - 1) + 1;
        x_d = norm([lidar.sensorOffset(1), lidar.sensorOffset(1)]) + ranges(j)*cos(lidar.scanAngles(j) - lidar.sensorAngleOffset);
        y_d = ranges(j)*sin(lidar.scanAngles(j) - lidar.sensorAngleOffset);
        if(norm([x_d, y_d]) >  dist)
            if(pastel_idx == 0)
                pastel_idx = i;
                pastel_count = 1;
            else
                pastel_count = pastel_count + 1;
            end
        else
            pastel_idx = 0;
        end
        i = i + 1;
    end
    dist = dist - dist_delta;
end

free_scan = mod(floor((pastel_idx + i - 1)/2) - 1, length(ranges) - 1) + 1;
free_angle = lidar.scanAngles(free_scan) - lidar.sensorAngleOffset;
free_angle = wrapToPi(free_angle);

if(abs(free_angle) < zero_angle)
    if(dist < dist_low)
        v = low_v;
    else
        v = high_v;
    end
elseif(free_angle < low_angle && free_angle > 0)
    w = low_w;
elseif(free_angle > -low_angle && free_angle < 0)
    w = -low_w;
elseif(free_angle > 0)
    w = high_w;
else
    w = -high_w;
end

end
