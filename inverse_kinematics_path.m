function [q_l_store, q_alpha_store, error] = inverse_kinematics_path(n, K, curvature, s_arc, c_arc, q_l, q_alpha, pose_list, max_iters)
    % take as input some parameters about the robot as well as a set of 3D
    % points to track
    
    % return the set of joint angles that produce the desired path
    
    len = size(pose_list);
    len = len(2);
    q_l_store = zeros(len, n); %places to store resulting ik variables
    q_alpha_store = zeros(len, n);
    error = [];
    
    for i = 1:len
        i
        pose_end = pose_list(:,i);
        [q_l_out, q_alpha_out, error_out] = inverse_kinematics(n, K, curvature, s_arc, c_arc, q_l, q_alpha, pose_end, max_iters);

        q_l_store(i,:) = q_l_out;
        q_alpha_store(i,:) = q_alpha_out;
        error = [error error_out];

        q_l = q_l_out;
        q_alpha = q_alpha_out;

    end

end