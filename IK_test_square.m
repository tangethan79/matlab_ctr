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

%create test square 20 mm directly above the robot
interp = 20;
z = 20*ones([1, interp]);
x1 = linspace(-15,15, interp);
y1 = -15*ones([1, interp]);
x2 = 15*ones([1, interp]);
y2 = linspace(-15,15, interp);
x3 = linspace(15,-15, interp);
y3 = 15*ones([1, interp]);
x4 = -15*ones([1, interp]);
y4 = linspace(15,-15, interp);

z = [z z z z];
x = [x1 x2 x3 x4];
y = [y1 y2 y3 y4];

%q_l_store = zeros(len(2), n); %places to store resulting ik variables
%q_alpha_store = zeros(len(2), n);
points_end = [x; y; z]; %set of points to test out
len = size(points_end);
max_iters = 200;

[q_l_store, q_alpha_store, error] = inverse_kinematics_graph_traversal(n, K, curvature, s_arc, c_arc, q_l, q_alpha, points_end, max_iters);

%{
for i = 1:len(2)
    i
    pose_end = points_end(:,i);
    [q_l_out, q_alpha_out] = inverse_kinematics(n, K, curvature, s_arc, c_arc, q_l, q_alpha, pose_end);
    [H_list, Htb, waypoints, link_num] = forward_kinematics(n, K, curvature, s_arc, c_arc, q_l_out, q_alpha_out);
    
    q_l_store(i,:) = q_l_out;
    q_alpha_store(i,:) = q_alpha_out;
    
    q_l = q_l_out;
    q_alpha = q_alpha_out;
    
    trans_points = zeros(3,link_num);
    for j = 1:link_num
        trans_points(:,j) = waypoints(:,50*j+1);
    end
    plot3(ax1, waypoints(1,:),waypoints(2,:),waypoints(3,:))
    hold on
    scatter3(ax1, trans_points(1,:),trans_points(2,:),trans_points(3,:),3 , "red");
    grid on
end
%}

f1 = figure;
ax1 = axes('Parent',f1);

scatter3(ax1, x,y,z, '*')
hold on

len = size(q_l_store);

for i = 1:len(1)
    [H_list, Htb, waypoints, link_num] = forward_kinematics(n, K, curvature, s_arc, c_arc, q_l_store(i, :), q_alpha_store(i, :));
    trans_points = zeros(3,link_num);
    for j = 1:link_num
        trans_points(:,j) = waypoints(:,50*j+1);
    end
    plot3(ax1, waypoints(1,:),waypoints(2,:),waypoints(3,:))
    hold on
    scatter3(ax1, trans_points(1,:),trans_points(2,:),trans_points(3,:),3 , "red");
end

pbaspect([1 1 1])


grid on
min(error)
mean(error)
error
out_cost = total_path_heuristic(q_l_store, q_alpha_store, error)/interp

f2 = figure('Name','Joint Variables');
t = linspace (0,5, interp*4);

ax2 = subplot(2,1,1);
for k = 1:n
    plot(ax2, t, q_l_store(:,k),'DisplayName',['q_l' num2str(k)])
    hold on
end

ax3 = subplot(2,1,2);
for l = 1:n
    plot(ax3, t, q_alpha_store(:,l),'DisplayName',['q_{\alpha}' num2str(l)])
    hold on
end
legend(ax2)
legend(ax3)