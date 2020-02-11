%% Humerus Coordinate System
% This MATLAB script reads in data from humerus points and determines the
% humeral shaft axis, center of rotation, epicondylar axis, and humeral
% plane.

[path,~,~]=fileparts(mfilename('fullpath'));
addpath(path);
%add initClasses
addpath(fullfile(path,'..','..','initClasses'));
addpath(fullfile(path,'..','..','externalLibs','xml2struct'));

%initialize
params=Parameters(fullfile(path,'..','..','parameters','parametersM20.xml'));
addpath(params.mocapToRobotLibPath);
initMocapToRobotLib();
%% Prerequisites for running the script
% The following variables must exist in the workspace before running this
% script:
%%
% * cylinder_data - a nx3 matrix containing 3D data for any number of
% points around the humeral shaft
% * sphereDataNeck - a nx3 matrix containing 3D data for any number of
% points around the anatomical neck
% * sphereDataHead - a nx3 matrix containing 3D data for any number of
% points around the humeral head
% * epicondyles - a 2x3 matrix containing 3D data for each of the
% epicondyles
% * hsCapture - a nx7 matrix containing the position and orientation of the
% hemisphere in the position where the bone was digitized. The first 4
% columns are the quaternion orientation and the last 3 columns are the
% position
% * hsFrame - a 1x6 matrix containing the frame of hemisphere. The first 3
% columns contain the position, and the last 3 the Euler angle orientation
% (XYZ extrinsic as is normal for Fanuc)
% * robotPos - a 1x6 matrix containing the pose of the robot during
% digitization. The first 3 columns contain the position, and the last 3 
% the Euler angle orientation (XYZ extrinsic as is normal for Fanuc)
%% Plot the sphere
close all
%concatenate the data so the fit is done on both sets of points
sphereAll = vertcat(sphereDataNeck,sphereDataHead);

%do the sphere fit
[Center_LSE,Radius_LSE,Residual] = spherefit(sphereAll);

%now normalize all the collected points so the humeral head center becomes
%the origin
cylinder_data_N=cylinder_data-Center_LSE';
epicondyles_N=epicondyles-Center_LSE';
sphereDataHead_N=sphereDataHead-Center_LSE';
sphereDataNeck_N=sphereDataNeck-Center_LSE';
sphereAll_N=sphereAll-Center_LSE';

%sphereDataNeck holds the points digitized around the neck
%sphereDataHead holds the points digitized around the humeral head
scatter3(sphereDataNeck_N(:, 1), sphereDataNeck_N(:, 2), sphereDataNeck_N(:, 3), 20, 'red', 'filled')
hold on
scatter3(sphereDataHead_N(:, 1), sphereDataHead_N(:, 2), sphereDataHead_N(:, 3), 20, 'blue', 'filled')

%plot the sphere
[sphere_base_X,sphere_base_Y,sphere_base_Z] = sphere(20);
hold on
surf(Radius_LSE*sphere_base_X,...
    Radius_LSE*sphere_base_Y,...
    Radius_LSE*sphere_base_Z,'faceAlpha',0.3,'Facecolor','c', 'EdgeColor','none','LineStyle','none','FaceLighting','phong')

%% Cylinder data scatter plot
%scatter plot the cylinder data
hold on
scatter3(cylinder_data_N(:,1), cylinder_data_N(:,2), cylinder_data_N(:,3), 'green', 'filled')
%% Plot epicondylar axis and humeral plane
%draw the line between the epicondyles
hold on
line(epicondyles_N(:, 1),epicondyles_N(:, 2), epicondyles_N(:, 3), ...
    'LineWidth', 4, 'Color', 'red')

text(epicondyles_N(1, 1),epicondyles_N(1, 2),epicondyles_N(1, 3),'M');
text(epicondyles_N(2, 1),epicondyles_N(2, 2),epicondyles_N(2, 3),'L');

xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal
%% Calculate humeral coordinate system and plot it
%first subtract digitization position from center
epi_mid = (epicondyles(1,:)+epicondyles(2,:))/2;
z_axis = Center_LSE'-epi_mid;
epi_diff = epicondyles(2,:)-epicondyles(1,:);
y_axis = cross(z_axis,epi_diff);
x_axis = cross(y_axis,z_axis);

x_axis = x_axis/norm(x_axis);
y_axis = y_axis/norm(y_axis);
z_axis = z_axis/norm(z_axis);
view(y_axis-x_axis)

%this is just so we can size the axes appropriately
axesLength=0.5*sqrt(dot(epi_diff,epi_diff));
line([0 x_axis(1)*axesLength],[0 x_axis(2)*axesLength],[0 x_axis(3)*axesLength],'Color','red','LineWidth',3);
line([0 y_axis(1)*axesLength],[0 y_axis(2)*axesLength],[0 y_axis(3)*axesLength],'Color','green','LineWidth',3);
line([0 z_axis(1)*axesLength],[0 z_axis(2)*axesLength],[0 z_axis(3)*axesLength],'Color','blue','LineWidth',3);

%above we have the humerus with respect to NDI, now we express it with
%respect to the hemisphere and then with respect to the default toolframe
N_R_Hu = [x_axis' y_axis' z_axis'];
T_R_He = eul2rotm(fliplr(deg2rad(hsFrame(4:6))));
N_R_He = quat2rotm(wavg_quaternion_markley(hsCapture(:,1:4),ones(size(hsCapture,1),1))');
T_R_Hu = T_R_He*N_R_He'*N_R_Hu;

%let's decompose it to find the yaw, pitch, roll
tcp_ypr = fliplr(rad2deg(rotm2eul(T_R_Hu)));

%now let's calculate the tool center point
%this is the position of the hemisphere in the optotrak frame of reference
%during digitization (we average to get a better indication of its
%position)
N_v_NtoHe=mean(hsCapture(:,5:7),1);
%vector from hemisphere to humeral head center in optotrak frame
N_v_HeToHH=Center_LSE-N_v_NtoHe';
%convert vector from hemisphere to humeral head center first to the
%hemisphere frame then to the default toolframe and add the vector from the
%default toolframe to the hemisphere in the default toolframe frame of
%reference
bone_tcp_tf = T_R_He*N_R_He'*N_v_HeToHH + hsFrame(1:3)';