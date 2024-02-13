%tube parameters
n = 3; %number of tubes
K = [20000 100 1]; %EI of each tube
curvature = [0 0.1 0.1]; %curvature of curved segments
s_arc = [10 5 10]; %arc length of straight segment
c_arc = [0 15 15]; %arc length of curved segment

%input parameters
q_l = [0 0 0]; %arc length extension inputs, assume this is added to s_arc
q_alpha = [0 0 0]; %angular rotation inputs, assume zero lines up with z0 y0 plane
%alpha goes counterclockwise from -y_0 to x_0 (according to RHR)
%assume torsionally rigid model, alpha ~= theta

%create test circle 20 mm directly above the robot
theta = 0:0.1:2*pi;
r = 5;
z = 20*ones(size(theta));
x = r*cos(theta);
y = r*sin(theta); 

len = size(z);
points_end = [x; y; z]; %set of points to test out

scatter3(x,y,z, '*')
hold on


for i = 1:len(2)
    i
    pose_end = points_end(:,i);
    [q_l_out, q_alpha_out] = inverse_kinematics(n, K, curvature, s_arc, c_arc, q_l, q_alpha, pose_end, 50);
    [H_list, Htb, waypoints, link_num] = forward_kinematics(n, K, curvature, s_arc, c_arc, q_l_out, q_alpha_out);
    
    trans_points = zeros(3,link_num);
    for i = 1:link_num
        trans_points(:,i) = waypoints(:,50*i+1);
    end

    plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:))
    hold on
    scatter3(trans_points(1,:),trans_points(2,:),trans_points(3,:),3 , "red");
    grid on
end