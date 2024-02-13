function wp = compute_waypoints(k, l, phi) %compute intermediary points with curvature, arc length, and phi
    if k == 0 %in the case that curvature is zero (i.e. portion is straight) transform is just translation along z-axis
        wp = [0;
            0;
            l];
    else
        wp = [-(sin(phi)*(cos(k*l) - 1))/k;
            (cos(phi)*(cos(k*l) - 1))/k;
            sin(k*l)/k];
    end
end