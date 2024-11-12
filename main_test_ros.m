clc; clear; close all;
load('percorsi.mat', 'percorsi');
seed = 12;

nRobot = 6;

%% PARAMETRI
data = struct();

nPassi = 1500;

nTag = 10;
nPhi = 8; % numero ipotesi angolo (si può poi variare in funzione della distanza misurata)
pruning = 1;
minZerosStartPruning = ceil(nPhi*0.6);
stepStartPruning0 = 70;         % mettere valore piccolo per evitare errori iniziali
stepStartPruning = repmat({stepStartPruning0}, 1, nRobot);
sharing = 1;
stepStartSharing = 400;
reset = 1;
resetThr = 100;

sigmaDistanza = 0.2; % std in m della misura di range
sigmaMisuraMedia = 1.0;

% ransac
numIterations = 100;
distanceThreshold = 0.1;
percentMinInliers = 0.7;

possibiliPhi = linspace(-pi+2*pi/nPhi, pi, nPhi);
sigmaPhi = 2*pi/(1.5*nPhi); %pi/(3*nPhi);

d = 0.16; % distanza presunta tra le due ruote

KR = 0.0001;
KL = 0.0001;

data.nPassi = nPassi;
data.nPhi = nPhi;
data.possibiliPhi = possibiliPhi;
data.sigmaPhi = sigmaPhi;
data.nTag = nTag;

data.sigmaDistanzaModello = sigmaDistanza;
data.sigmaPhi = sigmaPhi;
data.sigmaMisuraMedia = sigmaMisuraMedia;

data.d = d;

data.KR = KR;
data.KL = KL;

data.pruning = pruning;
data.minZerosStartPruning = minZerosStartPruning;

data.numIterations = numIterations;
data.distanceThreshold = distanceThreshold;
data.percentMinInliers = percentMinInliers;

data.resetThr = resetThr;
data.reset = reset;

x0 = [0.0, 0.0, 0.0];

rWheel = 0.033;
L = 10;

rng(seed);
cTag = 0.9*L*rand(nTag, 2) + 0.05*L;

%% INIZIALIZZAZIONE
rng(seed);

curr_robot = 1;
nRobot = 6;

startSharing = -1 * ones(1, nRobot);

ekfs = FedEkf.empty(nRobot, 0);
for robot = 1:nRobot
    x0Glob = percorsi(1, 1, robot);
    y0Glob = percorsi(1, 2, robot);

    distanze = sqrt((x0Glob-cTag(:,1)).^2+(y0Glob-cTag(:,2)).^2) + sigmaDistanza*randn;

    ekfs(robot) = FedEkf(data, robot, x0, distanze);
end

% ROS
node = ros2node('/ro_slam_matlab');
ns = strcat('/robot', num2str(curr_robot));

odom_pub = ros2publisher(node, strcat(ns, '/joint_states'), 'sensor_msgs/JointState');
odom_msg = ros2message(odom_pub);
odom_msg.name = {'right', 'left'};
odom_msg.velocity = [1.2, 2.5];

uwb_pub = ros2publisher(node, strcat(ns, '/uwb_tag'), 'ro_slam_interfaces/UwbArray');
uwb_msg = ros2message(uwb_pub);
uwb_msg.anchor_num = uint8(nTag);

shared_data_pub = ros2publisher(node, '/shared_landmarks_test', 'ro_slam_interfaces/LandmarkArray');
shared_data_msg = ros2message(shared_data_pub);

k = 0;

%%
rng(seed);
pause(1)

pause_sleep = 0.05;
stops = [];
for iter = 1:1000
    k = k + 1;
    disp(k)

    for robot = 1:nRobot
        % PREDIZIONE
        uRe = percorsi(k, 4, robot);
        uLe = percorsi(k, 5, robot);

        if robot == curr_robot
            if ismember(iter, stops)
                pause()
            end
        
            odom_msg.velocity = [uRe / rWheel, uLe / rWheel];
            send(odom_pub, odom_msg);
            pause(pause_sleep)

        end    
        ekfs(robot).prediction(uRe, uLe);

        % CORREZIONE
        if mod(k, 1) ~= 0
            continue
        end

        x = percorsi(k, 1, robot);
        y = percorsi(k, 2, robot);
        misureRange = sqrt((x-cTag(:,1)).^2+(y-cTag(:,2)).^2) + sigmaDistanza*randn;

        if robot == curr_robot
            for i = 1:nTag
                uwb_msg.uwbs(i).header = uwb_msg.header;
                uwb_msg.uwbs(i).id = int8(i-1);
                uwb_msg.uwbs(i).id_str = num2str(i-1);
                uwb_msg.uwbs(i).dist = single(misureRange(i));
                uwb_msg.uwbs(i).x = single(0);
                uwb_msg.uwbs(i).y = single(0);
                uwb_msg.uwbs(i).z = single(0);
            end
            send(uwb_pub, uwb_msg);
            pause(pause_sleep)
        end

        ekfs(robot).correction(misureRange);

        if robot == curr_robot
            disp(ekfs(robot).xHatSLAM(:, k+1)')
            disp(ekfs(robot).pesi)
        end

        if pruning && k >= stepStartPruning{robot}(end)
            ekfs(robot).pruning();
        end
    
        ekfs(robot).save_history();
    end

    % CORREZIONE con altre misure
    if sharing && k > stepStartSharing
        sharedInfoArray = struct('id', {}, 'tags', {}, 'vars', {});
        for robot = 1:nRobot
            if sum(ekfs(robot).nPhiVett) == nTag
                if startSharing(robot) == -1
                    startSharing(robot) = k;
                end
                sharedInfoArray(end+1) = ekfs(robot).data_to_share();
            end
        end

        if ~isempty(sharedInfoArray)
            for robot = 1:nRobot
                if robot == curr_robot
                    for i = 1:size(sharedInfoArray, 2)
                        shared_data_msg.id = uint8(sharedInfoArray(i).id);
                        for idx_tag = 1:nTag
                            shared_data_msg.landmarks(idx_tag).x = double(sharedInfoArray(i).tags(1, idx_tag));
                            shared_data_msg.landmarks(idx_tag).y = double(sharedInfoArray(i).tags(2, idx_tag));

                            shared_data_msg.landmarks(idx_tag).var_x =  double(sharedInfoArray(i).vars(1, idx_tag));
                            shared_data_msg.landmarks(idx_tag).var_y =  double(sharedInfoArray(i).vars(2, idx_tag));
                            shared_data_msg.landmarks(idx_tag).cov_xy = double(sharedInfoArray(i).vars(3, idx_tag));
                        end
                        send(shared_data_pub, shared_data_msg);
                        pause(pause_sleep)
                    end
                    shared_data_msg.id = uint8(100);
                    send(shared_data_pub, shared_data_msg);
                    pause(pause_sleep)
                end
    
                ekfs(robot).correction_shared(sharedInfoArray);
                if pruning && k >= stepStartPruning{robot}(end)
                    ekfs(robot).pruning();
                end
                if ekfs(robot).do_reset
                    fprintf('Robot %d resetting at t=%d\n', robot, k)
                    stepStartPruning{robot}(end+1) = stepStartPruning{robot}(end) + k;
                    ekfs(robot).reset();
                end
            end
        end
    end
end

return
