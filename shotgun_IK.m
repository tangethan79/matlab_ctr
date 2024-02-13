function [q_l_store, q_alpha_store, fk_distance] = shotgun_IK(n, K, curvature, s_arc, c_arc, q_l_start, q_alpha_start, pose_end, max_iters)
    % this function takes in a reference starting configuration and a
    % desired end pose, then computes IK solutions for a group of points
    % near the initial configuration in configuration space
    
    % along with the resulting extension and alpha parameters, the
    % euclidean distance from the desired point is also included for all
    % guesses
    
    % parameters like interpolation and maximum distance in configuration
    % space must be tuned
    
    interp = 4; % number of points to interpolate for each tube parameter
    
    max_per_sol = max_iters/(interp*n+1);
    
    
    max_dist_l = 1; % max distance in configuration space
    max_dist_alpha = pi/8;
    
    q_l_store = zeros(1, n);
    q_alpha_store = zeros(1, n);
    fk_distance = zeros(1, 1);
    
    sol_num = 1; % counter to keep track of how many IK solutions have been calculated
    
    [q_l_out, q_alpha_out, error] = inverse_kinematics(n, K, curvature, s_arc, c_arc, q_l_start, q_alpha_start, pose_end, max_per_sol);
    q_l_store(sol_num,:) = q_l_out;
    q_alpha_store(sol_num,:) = q_alpha_out;
    fk_distance(sol_num,1) = error;
    
    sol_num = sol_num+1
    
    for i = 1:n
        diff_l = linspace(-max_dist_l,max_dist_l, interp);
        diff_alpha = linspace(-max_dist_alpha,max_dist_alpha, interp);
        % interpolate extension length 
        for j = 1:interp
            
            q_l_temp = q_l_start;
            q_l_temp(i) = q_l_temp(i) + diff_l(j);
            % adjust relevant extension length by perturbation
            [q_l_out, q_alpha_out, error] = inverse_kinematics(n, K, curvature, s_arc, c_arc, q_l_temp, q_alpha_start, pose_end, max_per_sol);
            q_l_store(sol_num,:) = q_l_out;
            q_alpha_store(sol_num,:) = q_alpha_out;
            fk_distance(sol_num,1) = error;
            
            sol_num = sol_num+1
            
            q_alpha_temp = q_alpha_start;
            q_alpha_temp(i) = q_alpha_temp(i) + diff_alpha(j);
            % adjust alpha values as well
            [q_l_out, q_alpha_out, error] = inverse_kinematics(n, K, curvature, s_arc, c_arc, q_l_start, q_alpha_temp, pose_end, max_per_sol);
            q_l_store(sol_num,:) = q_l_out;
            q_alpha_store(sol_num,:) = q_alpha_out;
            fk_distance(sol_num,1) = error;
            
            sol_num = sol_num+1
        end
    end
end