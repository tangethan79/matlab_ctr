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

x = 12;
y = 12;
z = 14;

scatter3(x,y,z, 'MarkerFaceColor',[0 .75 .75])
hold on

[q_l_store, q_alpha_store, fk_distance] = shotgun_IK(n, K, curvature, s_arc, c_arc, q_l, q_alpha, [x; y; z], 300);
fk_distance

for j = 1:size(fk_distance,1)
    if fk_distance(j) < 1
        q_l_out = q_l_store(j,:);
        q_alpha_out = q_alpha_store(j,:);
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
end

xlabel('X-Axis (mm)') 
ylabel('Y-Axis (mm)') 
zlabel('Z-Axis (mm)')

text(x+0.1, y+0.1, z+0.1, 'Target Point');

pbaspect([1 1 1])