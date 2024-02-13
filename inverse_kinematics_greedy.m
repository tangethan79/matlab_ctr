function [q_l_out, q_alpha_out, error_out] = inverse_kinematics_greedy(n, K, curvature, s_arc, c_arc, q_l_start, q_alpha_start, pose_list, max_iters)
    % take as input some parameters about the robot as well as a set of 3D
    % points to track
    
    % return the set of joint angles that produce the desired path
    
    
    
    len = size(pose_list);
    len = len(2);
    [q_l_start, q_alpha_start, error_start] = inverse_kinematics(n, K, curvature, s_arc, c_arc, q_l_start, q_alpha_start, pose_list(:,1), max_iters);

    %initialize the storage variables for joint values
    
    q_l_out = [];
    q_alpha_out = [];
    error_out = [];

    for i = 1:len
        
        pose_end = pose_list(:,i);

        [q_l_new, q_alpha_new, error] = shotgun_IK(n, K, curvature, s_arc, c_arc, q_l_start, q_alpha_start, pose_end, max_iters*len);
        %find index of point with lowest positional error
        
        size(q_l_new,1)
        
        heuristic_list = [];
        for j = 1:size(q_l_new,1)
            heuristic_list = [heuristic_list edge_heuristic(q_l_start,q_alpha_start,q_l_new(j,:),q_alpha_new(j,:),error_start,error(j))];
        end
        
        [~, min_ind] = min(heuristic_list);
        q_l_start = q_l_new(min_ind,:);
        q_alpha_start = q_alpha_new(min_ind,:);
        error_start = error(min_ind);
        %update starting position for next shotgun
        q_l_out = [q_l_out; q_l_start];
        q_alpha_out = [q_alpha_out; q_alpha_start];
        error_out = [error_out error_start];
    end
    
end