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
file=params.smoothAndPlotFile;
period=params.period;
gaussInterval=params.smoothGaussInterval;
padLength=params.smoothPadLength;
padRemoval=params.smoothPadRemoval;

%read the file
[proximal,orientation,humerusLength,startEndIndices]=readV3DExport(file,period);
%keep just the portion of the activity that we care about
[~,cd3FileName,~]=fileparts(file);
startEndIndices=modifyStartEndIndices(startEndIndices,cd3FileName,gaussInterval);
[proximal,orientation]=truncateTrajectory(proximal,orientation,startEndIndices);
%call the smoothing function
smoothAndPlot(proximal,orientation,period,true,gaussInterval,padLength,padRemoval);