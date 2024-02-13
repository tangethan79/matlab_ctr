function [H_list, Htb, waypoints, link_num] = forward_kinematics(n, K, curvature, s_arc, c_arc, q_l, q_alpha)
    
    %this function computes intermediary transformation matrices as well as
    %interpolated waypoints for plotting

    t = link_arcs(s_arc, c_arc, q_l, n); %determine transition points in terms of arc length
    transition_mat = zeros(n, 2); %create a matrix of transition points for each tube
    for i = 1:n
        transition_mat(i,:) = [s_arc(i)+q_l(i) c_arc(i)+s_arc(i)+q_l(i)];
    end
    chi_numer = zeros(1, size(t,2)-1);
    gamma_numer = zeros(1, size(t,2)-1);
    k_denom = zeros(1, size(t,2)-1);

    for j = 1:size(k_denom,2)
        %for each link length, check the n tubes to see which contribute to
        %link shape
        %note that index k refers to the tube, j refers to the link
        for k = 1:n
            %check if curved portion contains link length
            if transition_mat(k, 2) >= t(j+1) && transition_mat(k,1) <= t(j)
                chi_numer(j) = chi_numer(j) + curvature(k) * K(k) * cos(q_alpha(k));
                gamma_numer(j) = gamma_numer(j) + curvature(k) * K(k) * sin(q_alpha(k));
                k_denom(j) = k_denom(j) + K(k);
            elseif transition_mat(k, 1) >= t(j+1) %check if straight portion contains link length
                %if only straight portion in link, no bending moment introduced
                k_denom(j) = k_denom(j) + K(k);
            end
        end
    end
    chi = zeros(size(chi_numer)); %x0 direction curvature
    gamma = zeros(size(gamma_numer)); %y0 direction curvature

    for j = 1:size(chi,2)
        chi(j) = chi_numer(j)/k_denom(j);
        gamma(j) = gamma_numer(j)/k_denom(j);
    end
    
    arc_links = zeros(size(chi)); %list of arc lengths of each link
    phi_links = zeros(size(chi));
    kappa_links = zeros(size(chi));
    prev_phi = 0;
    for j = 1:size(chi,2) %compute arc length, phi, and curvature for each link
        arc_links(j) = t(j+1)-t(j);
        phi_from_base = atan2(gamma(j), chi(j));
        phi_links(j) = phi_from_base-prev_phi;
        prev_phi = phi_from_base;
        kappa_links(j) = sqrt(gamma(j)^2 +chi(j)^2);
    end
    
    H_list = zeros(4,size(chi,2)*4);
    Htb = [1 0 0 0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1]; %keep track of final homog matrix
    H_intermediary = zeros(4,size(chi,2)*4); %keep track of intermediary for interpolation
    for j = 1:size(chi,2)
        Hj = compute_homog(kappa_links(j), arc_links(j), phi_links(j)); %compute and save relevant homographies
        H_intermediary(:,(((j-1)*4)+1):(((j-1)*4)+4)) = Htb;
        Htb = Htb*Hj;
        H_list(:,(((j-1)*4)+1):(((j-1)*4)+4)) = Hj;
    end
    
    %interpolate each link length
    waypoints = zeros(3,50*size(chi,2));
    for j = 1:size(chi,2)
        l_interp = 0:(t(j+1)-t(j))/50:t(j+1)-t(j);
        for i = 1:size(l_interp,2)
            wpi = [compute_waypoints(kappa_links(j), l_interp(i), phi_links(j)); 1];
            wpi = H_intermediary(:,(((j-1)*4)+1):(((j-1)*4)+4))* wpi;
            waypoints(:,((j-1)*50)+i) = wpi(1:3,1);
        end
        
    end
    link_num = size(chi,2);
end