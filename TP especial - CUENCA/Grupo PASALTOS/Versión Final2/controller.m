function [v, w, final_point, stop_and_turn] = controller(path_grid, path_world, sim_pos, est_grid, est_angle, stop_and_turn, norm_xy_d)

Rt = 0.35/2;
delta_min = 0.075;
dist_min_angosto = Rt + 0.4;
N = 5;
M = 2;
final_point = false;

diff_sim_dist = zeros(length(path_world), 1);
for j = 1:length(path_world)
    diff_sim_dist(j) = norm(path_world(j, :) - sim_pos(1:2));
end
[~, closest_idx] = min(diff_sim_dist);

if(closest_idx == length(path_grid))
    v = 0.05;
    w = 0;
    final_point = true;
    return
end

w_matrix = [0.3, 0.7;
            0.2, 0.6;
            0.2, 0.6;
            0.2, 0.8];
w = 0;
v_base = 0.03;
v = v_base;
v_M_factor = 1.5;
v_N_factor = 4;

if(~stop_and_turn)
    v_base = 0.05;
    if(norm_xy_d/dist_min_angosto < 1.5)
        v = v_base*norm_xy_d/dist_min_angosto;
    end
    v_M_factor = 1.5;
    v_N_factor = 4;
end

path_diff = path_grid(closest_idx + 1, :) - path_grid(closest_idx, :);
[next_angle, ~] = cart2pol(path_diff(2), -path_diff(1));

sim_pos(3) = wrapToPi(sim_pos(3));
next_angle = wrapToPi(next_angle);

delta_angle = wrapToPi(sim_pos(3) - next_angle);

if((closest_idx == 1 || stop_and_turn) && abs(delta_angle) > delta_min)
    w_p = [-w_matrix(1,1)*sign(delta_angle), -w_matrix(1,2)*delta_angle/pi];
    [~, m_w] = max(abs(w_p));
    w = w_p(m_w);
    if(stop_and_turn)
        v = 0;
    end
    return
else
    if(closest_idx ~= 1)
        if(closest_idx + N + 1 <= length(path_grid))
            path_diff_N = path_grid(closest_idx + 1:closest_idx + N + 1, :) - path_grid(closest_idx:closest_idx + N, :);
            if(min(path_diff_N) == max(path_diff_N))
                [next_angle, ~] = cart2pol(path_grid(closest_idx + N + 1, 2) - est_grid(2), -(path_grid(closest_idx + N + 1, 1) - est_grid(1)));
                next_angle = wrapToPi(next_angle);
                delta_angle = wrapToPi(est_angle - next_angle);
                if(abs(delta_angle) > delta_min)
                    w_p = [-w_matrix(2,1)*sign(delta_angle), -w_matrix(2,2)*delta_angle/pi];
                    [~, m_w] = max(abs(w_p));
                    w = w_p(m_w);
                    v = v*(1 - abs(delta_angle)/pi);
                else
                    v = v*v_N_factor;
                    stop_and_turn = false;
                end
                return
            end
        end
        M = min([M, length(path_grid) - closest_idx - 1]);
        path_diff_M = path_grid(closest_idx + 1:closest_idx + M + 1, :) - path_grid(closest_idx:closest_idx + M, :);

        if(M ~= 0)
            if(min(path_diff_M) == max(path_diff_M))
                if(abs(delta_angle) > delta_min)
                    w_p = [-w_matrix(3,1)*sign(delta_angle), -w_matrix(3,2)*delta_angle/pi];
                    [~, m_w] = max(abs(w_p));
                    w = w_p(m_w);
                    v = v*(1 - abs(delta_angle)/pi);
                else
                    v = v*v_M_factor;
                    stop_and_turn = false;
                end
                return
            else
                i = 1;
                while(i < M && path_diff_M(i, 1) == path_diff_M(i + 1, 1) && path_diff_M(i, 2) == path_diff_M(i + 1, 2))
                    i = i + 1;
                end
                [next_angle, ~] = cart2pol(path_diff_M(i + 1, 2), -path_diff_M(i + 1, 1));
                next_angle = wrapToPi(next_angle);
                delta_angle = wrapToPi(sim_pos(3) - next_angle);
                if(abs(delta_angle) > delta_min)
                    w_p = [-w_matrix(4,1)*sign(delta_angle), -w_matrix(4,2)*delta_angle/pi]/i;
                    [~, m_w] = max(abs(w_p));
                    w = w_p(m_w);
                    v = v*(1 - abs(delta_angle)/pi);
                else
                    stop_and_turn = false;
                    return
                end
            end
        else
            [next_angle, ~] = cart2pol(path_diff_M(1, 2), -path_diff_M(1, 1));
            next_angle = wrapToPi(next_angle);
            delta_angle = wrapToPi(sim_pos(3) - next_angle);
            if(abs(delta_angle) > delta_min)
                w_p = [-w_matrix(4,1)*sign(delta_angle), -w_matrix(4,2)*delta_angle/pi];
                [~, m_w] = max(abs(w_p));
                w = w_p(m_w);
                v = v*(1 - abs(delta_angle)/pi);
            else
                stop_and_turn = false;
            end
        end
    end
end

end