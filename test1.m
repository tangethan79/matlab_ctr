%tube parameters
n = 2; %number of tubes
K = [2000 100]; %EI of each tube
curvature = [1/10 1/10]; %curvature of curved segments
s_arc = [10 15]; %arc length of straight segment
c_arc = [10 15]; %arc length of curved segment

%input parameters
q_l = [0 0]; %arc length extension inputs, assume this is added to s_arc
q_alpha = [0 pi/2]; %angular rotation inputs, assume zero lines up with z0 x0 plane
%alpha goes counterclockwise from -y_0 to x_0 (according to RHR)
%assume torsionally rigid model, alpha ~= theta
q_l2 = 0:10/50:10;
q_a2 = 0:2*pi/50:2*pi;
for j = 1:size(q_a2,2)
    q_l = [0 q_l2(j)];
    q_alpha = [0 q_a2(j)];
    [H_list, Htb, waypoints, link_num] = forward_kinematics(n, K, curvature, s_arc, c_arc, q_l, q_alpha);
    %waypoints
    
    trans_points = zeros(3,link_num);
    for i = 1:link_num
        trans_points(:,i) = waypoints(:,50*i+1);
    end

    plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:))
    hold on
    scatter3(trans_points(1,:),trans_points(2,:),trans_points(3,:),3 , "red");
    grid on
end
axis equal
xlabel('x')
ylabel('y')
zlabel('z')