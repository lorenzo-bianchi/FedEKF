clc; clear; close all;
load('percorsi.mat', 'percorsi');
seed = 10;

%% PARAMETRI
data = struct();

nPassi = 1500;

nTag = 3;
nPhi = 4; % numero ipotesi angolo (si può poi variare in funzione della distanza misurata)
pruning = 1;
minZerosStartPruning = 1; %ceil(nPhi*0.6);
stepStartPruning = 25;         % mettere valore piccolo per evitare errori iniziali
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
data.stepStartPruning = stepStartPruning;

data.numIterations = numIterations;
data.distanceThreshold = distanceThreshold;
data.percentMinInliers = percentMinInliers;

data.resetThr = resetThr;
data.reset = reset;

x0 = [0.0, 0.0, 0.0];

robot = 1;
rWheel = 0.033;
L = 10;

rng(seed);
cTag = 0.9*L*rand(nTag, 2) + 0.05*L;

%% INIZIALIZZAZIONE
rng(seed);

x0Glob = percorsi(1, 1, robot);
y0Glob = percorsi(1, 2, robot);
distanze = sqrt((x0Glob-cTag(:,1)).^2+(y0Glob-cTag(:,2)).^2) + sigmaDistanza*randn;

ekf = FedEkf(data, 0, x0, distanze);

% ROS
node = ros2node("/ro_slam_matlab");

odom_pub = ros2publisher(node, "/robot1/joint_states", "sensor_msgs/JointState");
odom_msg = ros2message(odom_pub);
odom_msg.name = {'right', 'left'};
odom_msg.velocity = [1.2, 2.5];

uwb_pub = ros2publisher(node, "/robot1/uwb_tag", "ro_slam_interfaces/UwbArray");
uwb_msg = ros2message(uwb_pub);
uwb_msg.anchor_num = uint8(nTag);

k = 0;

if false
    %% PREDIZIONE
    k = k + 1;
    
    uRe = percorsi(k, 4, robot);
    uLe = percorsi(k, 5, robot);
    
    ekf.prediction(uRe, uLe);
    
    odom_msg.velocity = [uRe / rWheel, uLe / rWheel];
    send(odom_pub, odom_msg);
    
    %% CORREZIONE
    x = percorsi(k, 1, robot);
    y = percorsi(k, 2, robot);
    
    misureRange = sqrt((x-cTag(:,1)).^2+(y-cTag(:,2)).^2) + sigmaDistanza*randn;
    ekf.correction(misureRange);
    
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
end







%% PREDIZIONE+CORREZIONE
% odom_msg.velocity = [0.0, 0.0];
% send(odom_pub, odom_msg);
rng(seed);
pause(1)

for iter = 1:50
    % PREDIZIONE
    k = k + 1;
    
    uRe = percorsi(k, 4, robot);
    uLe = percorsi(k, 5, robot);
    
    odom_msg.velocity = [uRe / rWheel, uLe / rWheel];
    send(odom_pub, odom_msg);

    ekf.prediction(uRe, uLe);

    pause()

    
    % CORREZIONE
    x = percorsi(k, 1, robot);
    y = percorsi(k, 2, robot);
    misureRange = sqrt((x-cTag(:,1)).^2+(y-cTag(:,2)).^2) + sigmaDistanza*randn;

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

    ekf.correction(misureRange);
    disp(ekf.xHatSLAM(:, k+1)')
    disp(ekf.pesi)

    if pruning && k >= stepStartPruning
        ekf(robot).pruning();
    end

    disp(ekf.pesi)

    pause()
end

return

%%
% ROS
clc; clear; close all;
node = ros2node("/ro_slam_matlab");

misureRange = [1.234, 2.345, 3.456];
nTag = length(misureRange)

uwb_pub = ros2publisher(node, "/robot1/uwb_tag", "ro_slam_interfaces/UwbArray");
uwb_msg = ros2message(uwb_pub);
uwb_msg.anchor_num = uint8(nTag);

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