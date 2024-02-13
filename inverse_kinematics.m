function [q_l_out, q_alpha_out, error] = inverse_kinematics(n, K, curvature, s_arc, c_arc, q_l_start, q_alpha_start, pose_end, max_per_sol)
    %given an initial configuration and end pose, calculate (to a certain
    %margin) what the new inputs should be for this new pose
    
    %end_pose is a column vector of position and angular orientation
    
    step_l = 1;
    step_alpha = 1; %tunable step parameter for convergence
    %update to adaptive if necessary
    margin = 0.05; %convergence margin
    max_norm = 20;
    max_count = max_per_sol;
    
    q_l = q_l_start;
    q_alpha = q_alpha_start;
    
    Htb_init = compute_tip(n, K, curvature, s_arc, c_arc, q_l, q_alpha);
    % R = Htb_init(1:3,1:3);
    % rpy = rpy_from_dcm(R);
    % P = Htb_init(1:3,4);
    % pose_prev = [P; rpy];
    % this was an experiment with using rpy as well as positional
    % information
    
    pose_prev = Htb_init(1:3,4);
    pose_start = pose_prev;
    %calculate the initial pose given the proposed inputs
    count = 0;
    
    error = norm(pose_prev-pose_end);
    
    while (error >= margin && count < max_count)
        % jacobian_prev = compute_jacobian(n, K, curvature, s_arc, c_arc, q_l, q_alpha);
        jacobian_prev = compute_jacobian_xyz(n, K, curvature, s_arc, c_arc, q_l, q_alpha);
        %compute the jacobian at the setpoint
        j_pinv = pinv(jacobian_prev);
        %j_pinv = jacobian_prev';
        %note that the transpose also works, re-evaluate which to use if
        %necessary
        
        increment = (j_pinv*(pose_end - pose_prev));
        increment = increment';
        increment_norm = norm(increment);
        
        if increment_norm > max_norm
            increment = max_norm*(increment/increment_norm);
        end
        
        %note that increment is a column vector but the input qs are row
        %vectors, this must be transposed
        q_l = q_l + step_l*increment(1:n);
        q_alpha = q_alpha + step_alpha*increment(n+1:2*n);
        
        error_prev = error;
        error = norm(pose_prev-pose_end);
        
        if (error < 0.1 && abs(error_prev-error) < 0.01)
            break
        end
        
        Htb_init = compute_tip(n, K, curvature, s_arc, c_arc, q_l, q_alpha);
        %R = Htb_init(1:3,1:3);
        %rpy = rpy_from_dcm(R);
        %P = Htb_init(1:3,4);
        %pose_prev = [P; rpy];
        pose_prev = Htb_init(1:3,4);
        count = count+1;
    end
    
    q_l_out = q_l;
    q_alpha_out = q_alpha;
    %pose_end
    %pose_prev
    %pose_start
    %norm(pose_prev-pose_end)
    %q_l, q_alpha
    
end