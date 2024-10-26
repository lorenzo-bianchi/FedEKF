%% Read bag and topics names
bag = ros2bagreader('/home/lorenzo/Github/University/playground/logs/bag_prof/bag_prof_0.db3');
topics = string(bag.AvailableTopics.Properties.RowNames);

%% Get number of robots and topics names
n_robots = -1;
topics_names = strings(0);
for i = 1:length(topics)
    topic = topics(i);

    id_str = regexp(str, '/robot(\d+)/', 'tokens');
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
for i = 1:length(topics)
    topic = topics(i);
    if contains(topic, 'uwb')
        uwb_topic = select(bag, 'Topic', topic);
        uwb_msgs = readMessages(uwb_topic);
        n_anchors = -1;
        for k = 1:length(uwb_msgs)
            num = uwb_msgs{k}.anchor_num;
            if num > n_anchors && num < 100
                n_anchors = uwb_msgs{k}.anchor_num;
                pos_anchors = [[uwb_msgs{k}.uwbs.x]' [uwb_msgs{k}.uwbs.y]'];
            end
        end
        break
    end
end

%%
uwb_topic = select(bag, 'Topic', '/robot1/uwb_tag');
msgs = readMessages(uwb_topic);