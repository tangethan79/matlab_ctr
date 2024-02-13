function [q_l_out, q_alpha_out, error_out] = inverse_kinematics_graph_traversal(n, K, curvature, s_arc, c_arc, q_l_start, q_alpha_start, pose_list, max_iters)
    % take as input some parameters about the robot as well as a set of 3D
    % points to track
    
    % return the set of joint angles that produce the desired path
    
    
    
    len = size(pose_list);
    len = len(2);
    [q_l_start, q_alpha_start, ~] = inverse_kinematics(n, K, curvature, s_arc, c_arc, q_l_start, q_alpha_start, pose_list(:,1), max_iters);

    %initialize the storage variables for joint values
    q_l_store = [];
    q_alpha_store = [];
    error_store = [];
    index = 0;
    G = digraph;
    batch_old = 0;

    for i = 1:len
        
        pose_end = pose_list(:,i);

        [q_l_new, q_alpha_new, error] = shotgun_IK(n, K, curvature, s_arc, c_arc, q_l_start, q_alpha_start, pose_end, max_iters);
        %find index of point with lowest positional error
        [~, min_ind] = min(error);
        q_l_start = q_l_new(min_ind,:);
        q_alpha_start = q_alpha_new(min_ind,:);
        %update starting position for next shotgun
        
        batch_new = size(q_l_new);
        batch_new = batch_new(1);
        % calculate the size of the new batch of configurations generated
 
        q_l_store = [q_l_store; q_l_new];
        q_alpha_store = [q_alpha_store; q_alpha_new];
        error_store = [error_store error];
        %update storage variables
        if batch_old >1
            for j = 1:batch_old
                for k = 1:batch_new
                    weight = edge_heuristic(q_l_old(j,:),q_alpha_old(j,:),q_l_new(k,:),q_alpha_new(k,:),error_old(j),error(k));
                    G = addedge(G,[index+j],[index+batch_old+k],[weight]);
                end
            end
        else
            best_ind_start = min_ind;
        end
        index = index + batch_old;
        q_l_old = q_l_new;
        q_alpha_old = q_alpha_new;
        
        error_old = error;
        batch_old = batch_new;
    end
    %plot(G)
    best_ind_end = index + min_ind;
    best_path = shortestpath(G,best_ind_start,best_ind_end);
    
    q_l_out = [];
    q_alpha_out = [];
    error_out = [];
    for i = 1:len
        ind_good = best_path(i);
        q_l_out = [q_l_out; q_l_store(ind_good,:)];
        q_alpha_out = [q_alpha_out; q_alpha_store(ind_good,:)];
        error_out = [error_out error_store(ind_good)];
    end
    
end