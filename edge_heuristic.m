function weight = edge_heuristic(q_l1,q_alpha1,q_l2,q_alpha2,e1,e2)
    % this is an edge heuristic function that calculates how bad a
    % traversal is based on difference between parameters and how bad a
    % guess each node is
    
    % distance is calculated outside the function because the extra
    % variables are confusing

    q_weight = 1;
    dist_weight = 1;
    % weighting factors for each type of error
    
    weight = q_weight*(norm(q_l1-q_l2)+norm(q_alpha1-q_alpha2)) + dist_weight*(norm(e1)+norm(e2));
end