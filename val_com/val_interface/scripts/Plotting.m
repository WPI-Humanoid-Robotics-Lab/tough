close all;
clear all;

%loading the data
ihmc_walking=load('/home/rohit/indigo_ws/src/valkyrie_control/val_interface/DataFiles/IMU_ForceWithIHMC_Walking.txt');
ihmc_notwalking=load('/home/rohit/indigo_ws/src/valkyrie_control/val_interface/DataFiles/IMU_ForceWithIHMC_Notwalking.txt');
nocontroller=load('/home/rohit/indigo_ws/src/valkyrie_control/val_interface/DataFiles/IMU_ForceWithoutController.txt');

%ihmc_walking
% IMU Acceleration
figure 1;
plot(ihmc_walking(:,1:3));
title('IHMC walking IMU Acceleration data');
hold on;
plot(ihmc_walking(:,8:10));
hold on;
plot(ihmc_walking(:,15:17));
legend('Pelvis Middle X ','Pelvis Middle Y','Pelvis Middle Z','Pelvis Rear X ','Pelvis Rear Y','Pelvis Rear Z','Left Torso X','Left Torso Y','Left Torso Z');

% IMU Orientation
figure 2;
plot(ihmc_walking(:,4:7));
title('IHMC walking IMU Orientation data');
hold on;
plot(ihmc_walking(:,11:14));
hold on;
plot(ihmc_walking(:,18:21));
legend('Pelvis Middle X ','Pelvis Middle Y','Pelvis Middle Z','Pelvis Middle W','Pelvis Rear X ','Pelvis Rear Y','Pelvis Rear Z','Pelvis Rear W','Left Torso X','Left Torso Y','Left Torso Z','Left Torso W');

% Force sensor
figure 3;
plot(ihmc_walking(:,22:24));
title('IHMC walking Force sensor data');
hold on;
plot(ihmc_walking(:,28:30));
hold on;
plot(ihmc_walking(:,34:36));
hold on;
plot(ihmc_walking(:,40:42));
legend('LeftForce X ','LeftForce Y', 'LeftForce Z','LeftForceOffset X ','LeftForceOffset Y', 'LeftForceOffset Z','RightForce X ','RightForce Y', 'RightForce Z','RightForceOffset X ','RightForceOffset Y', 'RightForceOffset Z');

% Torque sensor
figure 4;
plot(ihmc_walking(:,25:27));
title('IHMC walking Torque sensor data');
hold on;
plot(ihmc_walking(:,31:33));
hold on;
plot(ihmc_walking(:,37:39));
hold on;
plot(ihmc_walking(:,43:45));
legend('LeftTorque X ','LeftTorque Y', 'LeftTorque Z','LeftTorqueOffset X ','LeftTorqueOffset Y', 'LeftTorqueOffset Z','RightTorque X ','RightTorque Y', 'RightTorque Z','RightTorqueOffset X ','RightTorqueOffset Y', 'RightTorqueOffset Z');

% ihmc_notwalking
% IMU Acceleration
figure 5;
plot(ihmc_notwalking(:,1:3));
title('IHMC Not walking IMU Acceleration data');
hold on;
plot(ihmc_notwalking(:,8:10));
hold on;
plot(ihmc_notwalking(:,15:17));
legend('Pelvis Middle X ','Pelvis Middle Y','Pelvis Middle Z','Pelvis Rear X ','Pelvis Rear Y','Pelvis Rear Z','Left Torso X','Left Torso Y','Left Torso Z');

% IMU Orientation
figure 6;
plot(ihmc_notwalking(:,4:7));
title('IHMC Not walking IMU Orientation data');
hold on;
plot(ihmc_notwalking(:,11:14));
hold on;
plot(ihmc_notwalking(:,18:21));
legend('Pelvis Middle X ','Pelvis Middle Y','Pelvis Middle Z','Pelvis Middle W','Pelvis Rear X ','Pelvis Rear Y','Pelvis Rear Z','Pelvis Rear W','Left Torso X','Left Torso Y','Left Torso Z','Left Torso W');

% Force sensor
figure 7;
plot(ihmc_notwalking(:,22:24));
title('IHMC Not walking Force sensor data');
hold on;
plot(ihmc_notwalking(:,28:30));
hold on;
plot(ihmc_notwalking(:,34:36));
hold on;
plot(ihmc_notwalking(:,40:42));
legend('LeftForce X ','LeftForce Y', 'LeftForce Z','LeftForceOffset X ','LeftForceOffset Y', 'LeftForceOffset Z','RightForce X ','RightForce Y', 'RightForce Z','RightForceOffset X ','RightForceOffset Y', 'RightForceOffset Z');

% Torque sensor
figure 8;
plot(ihmc_notwalking(:,25:27));
title('IHMC Not walking Torque sensor data');
hold on;
plot(ihmc_notwalking(:,31:33));
hold on;
plot(ihmc_notwalking(:,37:39));
hold on;
plot(ihmc_notwalking(:,43:45));
legend('LeftTorque X ','LeftTorque Y', 'LeftTorque Z','LeftTorqueOffset X ','LeftTorqueOffset Y', 'LeftTorqueOffset Z','RightTorque X ','RightTorque Y', 'RightTorque Z','RightTorqueOffset X ','RightTorqueOffset Y', 'RightTorqueOffset Z');

% nocontrol
% IMU Acceleration
figure 9;
plot(nocontroller(:,1:3));
title('No controller IMU Acceleration data');
hold on;
plot(nocontroller(:,8:10));
hold on;
plot(nocontroller(:,15:17));
legend('Pelvis Middle X ','Pelvis Middle Y','Pelvis Middle Z','Pelvis Rear X ','Pelvis Rear Y','Pelvis Rear Z','Left Torso X','Left Torso Y','Left Torso Z');

% IMU Orientation
figure 10;
plot(nocontroller(:,4:7));
title('No controller IMU Orientation data');
hold on;
plot(nocontroller(:,11:14));
hold on;
plot(nocontroller(:,18:21));
legend('Pelvis Middle X ','Pelvis Middle Y','Pelvis Middle Z','Pelvis Middle W','Pelvis Rear X ','Pelvis Rear Y','Pelvis Rear Z','Pelvis Rear W','Left Torso X','Left Torso Y','Left Torso Z','Left Torso W');

% Force sensor
figure 11;
plot(nocontroller(:,22:24));
title('No controller Force sensor data');
hold on;
plot(nocontroller(:,28:30));
hold on;
plot(nocontroller(:,34:36));
hold on;
plot(nocontroller(:,40:42));
legend('LeftForce X ','LeftForce Y', 'LeftForce Z','LeftForceOffset X ','LeftForceOffset Y', 'LeftForceOffset Z','RightForce X ','RightForce Y', 'RightForce Z','RightForceOffset X ','RightForceOffset Y', 'RightForceOffset Z');

% Torque sensor
figure 12;
plot(nocontroller(:,25:27));
title('No controller Torque sensor data');
hold on;
plot(nocontroller(:,31:33));
hold on;
plot(nocontroller(:,37:39));
hold on;
plot(nocontroller(:,43:45));
legend('LeftTorque X ','LeftTorque Y', 'LeftTorque Z','LeftTorqueOffset X ','LeftTorqueOffset Y', 'LeftTorqueOffset Z','RightTorque X ','RightTorque Y', 'RightTorque Z','RightTorqueOffset X ','RightTorqueOffset Y', 'RightTorqueOffset Z');
