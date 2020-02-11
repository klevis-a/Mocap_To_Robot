[path,~,~]=fileparts(mfilename('fullpath'));
addpath(path);
%add initClasses
addpath(fullfile(path,'..','..','initClasses'));
addpath(fullfile(path,'..','..','externalLibs','xml2struct'));

%initialize
params=Parameters(fullfile(path,'..','..','parameters','parametersM20.xml'));
addpath(params.mocapToRobotLibPath);
initMocapToRobotLib();

%parameters
c3dfile=params.smoothAndExportC3DFile;
exportFile=params.smoothAndExportFile;
period=params.period;
augmentNumPoints=params.augmentNumPoints;
interpNumPoints=params.interpNumPoints;
gaussInterval=params.smoothGaussInterval;

%read the file
[proximal,orientation,~,startEndIndices]=readV3DExport(c3dfile,period);
if isempty(startEndIndices)
    disp('Skipping')
    return;
end
%keep just the portion of the activity that we care about
startEndIndices=modifyStartEndIndices(startEndIndices,c3dfile,gaussInterval);

%truncate the trajectory
[proximal,orientation]=truncateTrajectory(proximal,orientation,startEndIndices);

%create frames
frames=createFrames(proximal,orientation);

[newFrames,~]=augmentTrajectory(frames,period,augmentNumPoints,interpNumPoints,gaussInterval,0);
writeSmoothedTrajectory(newFrames,period,exportFile);
