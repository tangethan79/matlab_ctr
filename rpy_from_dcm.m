function rpy = rpy_from_dcm(R)
    %calculate roll pitch and yaw from a rotation matrix
    rpy = zeros(3, 1);

    % Roll.
    rpy(1) = atan2(R(3, 2), R(3, 3));

    % Pitch.
    sp = -R(3, 1);
    cp = sqrt(R(1, 1)*R(1, 1) + R(2, 1)*R(2, 1));

    if abs(cp) > 1e-15
        rpy(2) = atan2(sp, cp);
    else
        % Gimbal lock...
        rpy(2) = pi/2;

        if sp < 0
            rpy(2) = -rpy(2);
        end
    end

    % Yaw.
    rpy(3) = atan2(R(2, 1), R(1, 1));
end
