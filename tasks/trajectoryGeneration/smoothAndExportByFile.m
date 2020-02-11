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
gaussInterval=params.smoothGaussInterval;
padLength=params.smoothPadLength;
padRemoval=params.smoothPadRemoval;

%read the file
[proximal,orientation,humerusLength,startEndIndices]=readV3DExport(c3dfile,period);
%keep just the portion of the activity that we care about
[~,cd3FileName,~]=fileparts(c3dfile);
startEndIndices=modifyStartEndIndices(startEndIndices,cd3FileName,gaussInterval);
[proximal,orientation]=truncateTrajectory(proximal,orientation,startEndIndices);
%call the smoothing function
[framesSmooth,errorsP,errorsO]=smoothAndPlot(proximal,orientation,period,false,gaussInterval,padLength,padRemoval);
%write the smooth trajectory
writeSmoothedTrajectory(framesSmooth,period,exportFile);