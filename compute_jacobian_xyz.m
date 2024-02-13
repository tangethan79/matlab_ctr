function J = compute_jacobian_xyz(n, K, curvature, s_arc, c_arc, q_l, q_alpha)
    %this function computes the spatial velocity jacobian given some tube
    %parameters

    delta = 0.001; %tuning parameter for perturbation size
    % units of mm or degrees depending on pertured input

    Htb = compute_tip(n, K, curvature, s_arc, c_arc, q_l, q_alpha); %compute the tip to base transformation
    % compute dP/dq for each input q (these include extension and rotation
    % for each of the n tubes)
    
    pose_init = Htb(1:3,4);
    %extract unperturbed pose information in with rpy euler angle
    %representation
    
    J = zeros(3, n*2);
    for i = 1:n
        
        % calculate jacobian for perturbation in arc length
        q_l_copy = q_l;
        q_l_copy(i) = q_l_copy(i) + delta;
        H_del = compute_tip(n, K, curvature, s_arc, c_arc, q_l_copy, q_alpha);
        pose_del = H_del(1:3,4);
        J(:,i) = (pose_del-pose_init)/delta;
        
        % calculate jacobian for perturbation in rotation
        q_alpha_copy = q_alpha;
        q_alpha_copy(i) = q_alpha_copy(i) + delta;
        H_del = compute_tip(n, K, curvature, s_arc, c_arc, q_l, q_alpha_copy);
        pose_del = H_del(1:3,4);
        J(:, n + i) = (pose_del-pose_init)/delta;
    end
end