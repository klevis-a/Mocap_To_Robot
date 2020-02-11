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

%parameters
jointsFile=params.plotOptTrajJointsFile;
framesFile=params.plotOptTrajFramesFile;
toolFrameFile=params.plotOptTrajToolframe;
robot=robotI.robot;
base=robotI.base;
ee=robotI.ee;
velLimits=robotI.vel_limits;
period=params.period;
%read toolframe file if it exists
if isfile(toolFrameFile)
    toolframe=reshape(csvread(toolFrameFile),[4 4])';
else
    toolframe=robotI.HumerusToolframe;
end

%read joints and smooth frames files
joints=readJointsFile(jointsFile);
smoothFrames=readFramesFile(framesFile);

%compute error
[errorS,errorX,errorY]=checkErrorUsingJoints(joints,smoothFrames,...
            robot,base,ee,toolframe);
        
%compute cost
[cost,velper]=computeCostFromJoints(joints,velLimits,period);
violations=findViolations(velper);

%plot the joints
plotJoints(joints,period,velLimits);