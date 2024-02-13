function t = link_arcs(s_arc, c_arc, q_l, n)
    %this function returns the list of transition points sorted by arc
    %length from the base
    %assume each tube consists of 1 straight portion and 1 curved portion
    t_tubes = [0];
    for j = 1:n
        t_tubes = [t_tubes s_arc(j)+q_l(j) c_arc(j)+s_arc(j)+q_l(j)];
        %make a list of the transition points for each tube, i.e. the the
        %length of the straight portion + the extension and the length of
        %the entire tube
    end
    t_tubes = sort(t_tubes); %sort transition points by distance from 0
    t = unique(t_tubes); %elminate duplicates, i.e. places where multiple tubes have identical transition points
end