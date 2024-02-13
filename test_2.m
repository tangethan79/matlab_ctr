%tube parameters
n = 1; %number of tubes
K = [100]; %EI of each tube
curvature = [1/5]; %curvature of curved segments
s_arc = [0]; %arc length of straight segment
c_arc = [10]; %arc length of curved segment

%input parameters
q_l = [0]; %arc length extension inputs, assume this is added to s_arc
q_alpha = [0]; %angular rotation inputs, assume zero lines up with z0 x0 plane
%alpha goes counterclockwise from -y_0 to x_0 (according to RHR)
%assume torsionally rigid model, alpha ~= theta

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
axis equal
xlabel('x')
ylabel('y')
zlabel('z')

% alpha3 = 0:pi/50:2*pi;
% origins = zeros(3,size(alpha3,2));
% origins_int = zeros(3,size(alpha3,2));
% for i = 1:size(alpha3,2)
%     q_alpha = [alpha3(i) 0 0];
%     [H_list, Htb] = forward_kinematics(n, K, curvature, s_arc, c_arc, q_l, q_alpha);
%     origins(:,i) = Htb(1:3,4);
%     origins_int(:,i) = H_list(1:3,1:3)*H_list(1:3,8);
% end
% plot3(origins_int(1,:),origins_int(2,:),origins_int(3,:), origins(1,:),origins(2,:),origins(3,:))
% grid on
