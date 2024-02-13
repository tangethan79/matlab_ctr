function Htb = compute_tip(n, K, curvature, s_arc, c_arc, q_l, q_alpha)
    
    %this is a modified version of forward_kinematics that only calculates
    %the tip to base transformation for the purpose of inverse kinematic
    %jacobian calculation. It does not interpolate the waypoints and only
    %returns a single transformation

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
    
    Htb = [1 0 0 0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1]; %keep track of final homog matrix
    for j = 1:size(chi,2)
        Hj = compute_homog(kappa_links(j), arc_links(j), phi_links(j)); %compute and save relevant homographies
        Htb = Htb*Hj;
    end
    
end