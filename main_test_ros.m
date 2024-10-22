clc; clear; close all;

%% PARAMETRI
data = struct();

nRobot = 6;

nTag = 3;

nPassi = 1500;
nPhi = 4; % numero ipotesi angolo (si può poi variare in funzione della distanza misurata)
pruning = 1;
minZerosStartPruning = ceil(nPhi*0.6);
stepStartPruning = 100;         % mettere valore piccolo per evitare errori iniziali
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

x0 = [2.2, 1.1, 1.2];
distanze = zeros(1, nTag);

%% INIZIALIZZAZIONE
ekf = FedEkf(data, 0, x0, distanze);

% ROS
node = ros2node("/ro_slam_matlab");

odom_pub = ros2publisher(node, "/robot1/joint_states", "sensor_msgs/JointState");
odom_msg = ros2message(odom_pub);
odom_msg.name = {'right', 'left'};
odom_msg.velocity = [1.2, 2.5];

uwb_pub = ros2publisher(node, "/robot1/uwb_tag", "ro_slam_interfaces/UwbArray");
uwb_msg = ros2message(uwb_pub);
uwb_msg.anchor_num = nTag;

%% PREDIZIONE
r = 0.033;
omegaR = 1.0;
omegaL = -1.5;
ekf.prediction(omegaR * r, omegaL * r);

odom_msg.velocity = [omegaR, omegaL];
send(odom_pub, odom_msg);

%% CORREZIONE
misureRange = [5.0370, 7.3865, 7.3272];%, 6.8257, 2.5218, 8.6032, 4.4099, 7.3127, 3.0873, 3.8493];
ekf.correction(misureRange);

for i = 1:length(misureRange)
   uwb_msg.uwbs(i).dist = misureRange(i); 
end
send(uwb_pub, uwb_msg);