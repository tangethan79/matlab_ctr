n = 3; %number of tubes
K = [20000 100 1]; %EI of each tube
curvature = [0 0.1 0.1]; %curvature of curved segments
s_arc = [10 5 10]; %arc length of straight segment
c_arc = [0 15 15]; %arc length of curved segment

%create test line 20 mm directly above the robot
interp = 20;
z = 20*ones([1, interp]);
x1 = linspace(-15,15, interp);
y1 = -15*ones([1, interp]);

%q_l_store = zeros(len(2), n); %places to store resulting ik variables
%q_alpha_store = zeros(len(2), n);
points_end = [x1; y1; z]; %set of points to test out
len = size(points_end);
len = len(2);


pose_end = points_end(:,1);
q_l_start = [0 0 0];
q_alpha_start = [0 0 0];

max_iters = 300;

[q_l_store, q_alpha_store, error] = inverse_kinematics_graph_traversal(n, K, curvature, s_arc, c_arc, q_l_start, q_alpha_start, points_end, max_iters);


%{
[q_l_start, q_alpha_start, error] = inverse_kinematics(n, K, curvature, s_arc, c_arc, q_l_start, q_alpha_start, pose_end);

%initialize the storage variables for joint values
q_l_store = [q_l_start];
q_alpha_store = [q_alpha_start];

for i = 1:len
    pose_end = points_end(:,i);
    
    [q_l_new, q_alpha_new, error] = shotgun_IK(n, K, curvature, s_arc, c_arc, q_l_start, q_alpha_start, pose_end);
    %find index of point with lowest positional error
    [val, min_ind] = min(error);
    q_l_start = q_l_new(min_ind,:);
    q_alpha_start = q_alpha_new(min_ind,:);
    %update starting position for next shotgun
    
    q_l_store = [q_l_store; q_l_new];
    q_alpha_store = [q_alpha_store; q_alpha_new];
    %update storage variables
end
%}


f1 = figure;
ax1 = axes('Parent',f1);

scatter3(ax1, x1,y1,z, '*')
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
t = linspace (0,5, interp);

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