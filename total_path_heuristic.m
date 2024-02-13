function total_weight = total_path_heuristic(q_l,q_alpha,error)
    path_length = size(error);
    path_length = path_length(2);
    
    total_weight = 0;
    
    for i = 2:path_length
        total_weight = total_weight + edge_heuristic(q_l(i-1,:),q_alpha(i-1,:),q_l(i,:),q_alpha(i,:),error(i-1),error(i));
    end
end