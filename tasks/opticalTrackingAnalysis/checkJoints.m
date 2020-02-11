[path,~,~]=fileparts(mfilename('fullpath'));
addpath(path);
%add initClasses
addpath(fullfile(path,'..','..','initClasses'));
addpath(fullfile(path,'..','..','externalLibs','xml2struct'));

%initialize
params=Parameters(fullfile(path,'..','..','parameters','parametersM20.xml'));
addpath(params.mocapToRobotLibPath);
initMocapToRobotLib();
robotI=RobotInfo(fullfile(path,'..','..','parameters','robotM20.xml'));

%post analysis info
resultsFile=params.postAnalysisResultsFile;
dataFolder=params.postAnalysisDataFolder;
tickDefinition=params.tickDefinition;
period=params.period;
monitorNum=params.postAnalysisMonitorNum;

%read captured joints
[postAnalysisResultsFolder,resultsFileName,fileExt]=fileparts(resultsFile);
programName=extractBefore(strcat(resultsFileName,fileExt), '_sum.txt');
capturedJointsFileName=capturedJointsFile(postAnalysisResultsFolder,programName);
[capturedTime,capturedJoints]=readCapturedJoints(capturedJointsFileName,tickDefinition);
capturedJoints=rad2deg(capturedJoints);

%read desired joints
inputFile = fullfile(postAnalysisResultsFolder,strcat(programName,'.csv'));
inputTable = readtable(inputFile);
%this is a 1x1 table
c3dFile = inputTable{1,'c3dFile'};
desJointsFile=jointsFileName(dataFolder,c3dFile{1}, 1);
[desJoints]=readJointsFile(desJointsFile);
desJoints=rad2deg(desJoints);
desTime=((1:size(desJoints))-1)*period;

[capturedJoints_aligned]=alignDesCapturedJoints(desJoints,capturedJoints,capturedTime,period,200);

desVel=velocity(desTime,desJoints);
desVelPer=desVel./rad2deg(robotI.vel_limits);
desAcc=acceleration(desTime,desJoints);
capturedVel=velocity(desTime,capturedJoints_aligned);
capturedVelPer=capturedVel./rad2deg(robotI.vel_limits);
capturedAcc=acceleration(desTime,capturedJoints_aligned);

%screen info
set(0,'units','pixels');
screen_resolution = get(0,'MonitorPositions');
screen_width=screen_resolution(monitorNum,3);
screen_height=screen_resolution(monitorNum,4);
graph_width = (screen_width+120)/3;
graph_height = (screen_height-60)/2;
xOffset=screen_resolution(monitorNum,1);
yOffset=screen_resolution(monitorNum,2);

figure(2)
set(gcf,'Position',[xOffset yOffset graph_width graph_height])
for n=1:6
    subplot(2,3,n)
    plot(desTime,desVel(:,n));
    hold on
    plot(desTime,capturedVel(:,n));
    hold off
    title(sprintf('Joint %d', n));
end
sgtitle('Velocity')

figure(3)
set(gcf,'Position',[xOffset+graph_width yOffset graph_width graph_height])
for n=1:6
    subplot(2,3,n)
    plot(desTime,desVelPer(:,n));
    hold on
    plot(desTime,capturedVelPer(:,n));
    hold off
    title(sprintf('Joint %d', n));
end
sgtitle('Velocity Percentage')

figure(1)
set(gcf,'Position',[xOffset yOffset+graph_height graph_width graph_height])
for n=1:6
    subplot(2,3,n)
    plot(desTime,desJoints(:,n));
    hold on
    plot(desTime,capturedJoints_aligned(:,n));
    hold off
    title(sprintf('Joint %d', n));
end
sgtitle('Joint Angle')

figure(4)
set(gcf,'Position',[xOffset+graph_width yOffset+graph_height graph_width graph_height])
for n=1:6
    subplot(2,3,n)
    plot(desTime,desAcc(:,n));
    hold on
    plot(desTime,smoothdata(capturedAcc(:,n),'gaussian',15));
    hold off
    title(sprintf('Joint %d', n));
end
sgtitle('Acceleration')

