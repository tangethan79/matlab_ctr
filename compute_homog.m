function H = compute_homog(k, l, phi) %compute homogenous transformation matrix from arc parameters
    if k == 0 %in the case that curvature is zero (i.e. portion is straight) transform is just translation along z-axis
        H = [1 0 0 0;
            0 1 0 0;
            0 0 1 l;
            0 0 0 1];
    else
        H = [cos(phi), -cos(k*l)*sin(phi),  sin(k*l)*sin(phi), -(sin(phi)*(cos(k*l) - 1))/k;
            sin(phi),  cos(k*l)*cos(phi), -sin(k*l)*cos(phi),  (cos(phi)*(cos(k*l) - 1))/k;
            0,           sin(k*l),           cos(k*l),                   sin(k*l)/k;
            0,                  0,                  0,                            1];
    end
end