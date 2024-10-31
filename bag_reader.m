%% Cleanup
clc; clear; close all

%% Plot settings
font_size = 18;
line_width_lines = 1;
line_width_markers = 2;

%% Params
wheel_radius = 0.033;
wheels_separation = 0.16;

%% Read bag and topics names
bag_name = 'bag_prof_3_robot';
path_prefix = '/home/lorenzo/Github/University/playground/logs/';
full_path = strcat(path_prefix, bag_name, '/', bag_name, '_0.db3');
bag = ros2bagreader(full_path);
all_topics = string(bag.AvailableTopics.Properties.RowNames);
t_start = bag.StartTime;
t_end = bag.EndTime;

%% Get number of robots and topics names
n_robots = -1;
topics_names = strings(0);
for i = 1:length(all_topics)
    topic = all_topics(i);

    id_str = regexp(topic, '/robot(\d+)/', 'tokens');
    id = str2double(id_str{1}{1});
    if id > n_robots
        n_robots = id;
    end

    topic_split = split(topic, '/');
     if length(topic_split) > 2
        topic_name = topic_split(3);

        if ~any(topics_names == topic_name)
            topics_names(end+1) = topic_name;
        end
    end
end

%% Get number of tags and positions
for i = 1:length(all_topics)
    topic = all_topics(i);
    if contains(topic, 'uwb')
        uwb_topic = select(bag, 'Topic', topic);
        uwb_msgs = readMessages(uwb_topic);
        n_anchors_tot = -1;
        inter_robots_distances = false;
        for k = 1:length(uwb_msgs)
            num = uwb_msgs{k}.anchor_num;
            if num > n_anchors_tot
                ids = [uwb_msgs{k}.uwbs.id];
                n_anchors_tot = uwb_msgs{k}.anchor_num;
                ids_real_anchors = ids < 100;
                n_anchors = sum(ids_real_anchors);
                gt_pos_x = [uwb_msgs{k}.uwbs.x]';
                gt_pos_y = [uwb_msgs{k}.uwbs.y]';
                pos_anchors = [gt_pos_x(ids_real_anchors) gt_pos_y(ids_real_anchors)];

                if n_anchors < n_anchors_tot
                    inter_robots_distances = true;
                end
            end
        end
        break
    end
end

%% Summary
fprintf('Duration: %.2f s\n', t_end-t_start);
fprintf('Found: %d robots, %d anchors\n', n_robots, n_anchors);
if inter_robots_distances
    fprintf('Inter-robots distances AVAILABLE\n');
else
    fprintf('Inter-robots distances NOT AVAILABLE\n');
end
fprintf('Available topics:\n');
for name = topics_names
    fprintf('\t- %s\n', name);
end
fprintf('Anchors in positions:\n');
for i = 1:length(pos_anchors)
    fprintf('\t- (%d, %d)\n', pos_anchors(i,1), pos_anchors(i,2));
end

%% Topics analysis
% Ground truth 
if any(contains(topics_names, 'ground_truth'))
    gt_topics_names = all_topics(contains(all_topics, 'ground_truth'));
    gt_data = struct('times', [], 'x', [], 'y', [], 'theta', [], 'v_lin', [], 'v_ang', []);
    figure
    for i = 1:length(gt_topics_names)
        gt_topic = select(bag, 'Topic', gt_topics_names(i));
        gt_msg = readMessages(gt_topic);
        gt_times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, gt_msg);
        gt_pos_x = cellfun(@(m) double(m.pose.pose.position.x), gt_msg);
        gt_pos_y = cellfun(@(m) double(m.pose.pose.position.y), gt_msg);

        quat_w = cellfun(@(m) m.pose.pose.orientation.w, gt_msg);
        quat_x = cellfun(@(m) m.pose.pose.orientation.x, gt_msg);
        quat_y = cellfun(@(m) m.pose.pose.orientation.y, gt_msg);
        quat_z = cellfun(@(m) m.pose.pose.orientation.z, gt_msg);
        rpy = quat2eul([quat_w, quat_x, quat_y, quat_z], 'ZYX');
        gt_data(i).theta = rpy(:, 1);

        gt_data(i).times = gt_times;
        gt_data(i).x = gt_pos_x;
        gt_data(i).y = gt_pos_y;
        gt_data(i).v_lin = cellfun(@(m) m.twist.twist.linear.x, gt_msg);
        gt_data(i).v_ang = cellfun(@(m) m.twist.twist.angular.z, gt_msg);

        plot(gt_pos_x, gt_pos_y, '--', 'LineWidth', line_width_lines, 'DisplayName', gt_topics_names(i))
        hold on
    end
    plot(pos_anchors(:, 1), pos_anchors(:, 2), 'kh', ...
        'MarkerSize', 20, 'LineWidth', line_width_markers, 'DisplayName', 'Anchors')
    title('Ground truth')
    axis equal
    xlabel('x [m]')
    ylabel('y [m]')
    grid on
    legend('Location', 'northwest', 'Interpreter', 'none')
    ax = gca;
    ax.FontSize = font_size;
end

% UWB
if any(contains(topics_names, 'uwb_tag'))
    uwb_topics_names = all_topics(contains(all_topics, 'uwb_tag'));
    uwb_data = struct('times', [], 'dist_anchors', [], 'dist_robots', []);
    for i = 1:length(uwb_topics_names)
        uwb_topic = select(bag, 'Topic', uwb_topics_names(i));
        num_msgs = uwb_topic.NumMessages;
        uwb_msg = readMessages(uwb_topic);
        uwb_data(i).times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, uwb_msg);
        for j = 1:num_msgs
            uwb_dist = nan*ones(1, n_anchors);
            uwb_dist_inter = nan*ones(1, n_anchors_tot-n_anchors+1);
            uwb_dist_inter(i) = 0.0;
            for k = 1:size(uwb_msg{j}.uwbs, 1)
                id = uwb_msg{j}.uwbs(k).id;
                if id > 100
                    uwb_dist_inter(id-100) = uwb_msg{j}.uwbs(k).dist;
                    continue
                end
                uwb_dist(id+1) = uwb_msg{j}.uwbs(k).dist;
            end
            uwb_data(i).dist_anchors = [uwb_data(i).dist_anchors; uwb_dist];
            uwb_data(i).dist_robots = [uwb_data(i).dist_robots; uwb_dist_inter];
        end
    end
end

% Joint states
if any(contains(topics_names, 'joint_states'))
    js_topics_names = all_topics(contains(all_topics, 'joint_states'));
    js_data = struct('times', [], 'angle_left', [], 'angle_right', [], 'omega_left', [], 'omega_right', []);
    for i = 1:length(js_topics_names)
        js_topic = select(bag, 'Topic', js_topics_names(i));
        js_msg = readMessages(js_topic);

        if contains(js_msg{i}.name{1}, 'right')
            idx_right = 1;
            idx_left = 2;
        else
            idx_right = 2;
            idx_left = 1;
        end

        js_data(i).times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, js_msg);
        js_data(i).angle_left = cellfun(@(m) m.position(idx_left), js_msg);
        js_data(i).angle_right = cellfun(@(m) m.position(idx_right), js_msg);
        js_data(i).omega_left = cellfun(@(m) m.velocity(idx_left), js_msg);
        js_data(i).omega_right = cellfun(@(m) m.velocity(idx_right), js_msg);
    end
end

% Odom
if any(contains(topics_names, 'odom'))
    odom_topics_names = all_topics(contains(all_topics, 'odom'));
    odom_data = struct('times', [], 'x', [], 'y', [], 'theta', [], 'v_lin', [], 'v_ang', []);
    for i = 1:length(odom_topics_names)
        odom_topic = select(bag, 'Topic', odom_topics_names(i));
        odom_msg = readMessages(odom_topic);
        
        odom_data(i).times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, odom_msg);
        odom_data(i).x = cellfun(@(m) m.pose.pose.position.x, odom_msg);
        odom_data(i).y = cellfun(@(m) m.pose.pose.position.y, odom_msg);
        odom_data(i).v_lin = cellfun(@(m) m.twist.twist.linear.x, odom_msg);
        odom_data(i).v_ang = cellfun(@(m) m.twist.twist.angular.z, odom_msg);

        quat_w = cellfun(@(m) m.pose.pose.orientation.w, odom_msg);
        quat_x = cellfun(@(m) m.pose.pose.orientation.x, odom_msg);
        quat_y = cellfun(@(m) m.pose.pose.orientation.y, odom_msg);
        quat_z = cellfun(@(m) m.pose.pose.orientation.z, odom_msg);
        rpy = quat2eul([quat_w, quat_x, quat_y, quat_z], 'ZYX');
        odom_data(i).theta = rpy(:, 1);
    end
end

%% Save workspace
roslam_data = struct();

roslam_data.wheel_radius = wheel_radius;
roslam_data.wheels_separation = wheels_separation;

roslam_data.duration = t_end-t_start;
roslam_data.num_robots = n_robots;
roslam_data.num_anchors_fixed = n_anchors;
roslam_data.num_anchors_total = n_anchors_tot;
roslam_data.inter_robots_distances = inter_robots_distances;
roslam_data.anchors_positions = pos_anchors;

roslam_data.ground_truth = gt_data;
roslam_data.robot_odometry = odom_data;
roslam_data.wheels_odometry = js_data;
roslam_data.uwb_distances = uwb_data;

save('roslam_data.mat', 'roslam_data');

%% GT vs odom
robot = 2;
gt_x = gt_data(robot).x;
gt_y = gt_data(robot).y;
odom_x = odom_data(robot).x;
odom_y = odom_data(robot).y;

x_start = gt_x(1);
y_start = gt_y(1);

figure
plot(gt_x, gt_y, 'LineWidth', line_width_lines, 'DisplayName', 'Ground truth')
hold on
plot(odom_x + x_start, odom_y + y_start, 'LineWidth', line_width_lines, 'DisplayName', 'Odom')
plot(pos_anchors(:, 1), pos_anchors(:, 2), 'kh', ...
        'MarkerSize', 20, 'LineWidth', line_width_markers, 'DisplayName', 'Anchors')
title('Ground truth VS Odom')
axis equal
xlabel('x [m]')
ylabel('y [m]')
grid on
legend('Location', 'northwest', 'Interpreter', 'none')
ax = gca;
ax.FontSize = font_size;