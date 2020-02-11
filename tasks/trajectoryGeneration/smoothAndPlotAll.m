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
directory=params.smoothAndPlotAllDir;
period=params.period;
gaussInterval=params.smoothGaussInterval;
padLength=params.smoothPadLength;
padRemoval=params.smoothPadRemoval;

%read the directories (each directory corresponds to a subject)
dirBrowser=DirectoryBrowser(directory);

%loop through each directory
for i=1:dirBrowser.numFolders
    %find files ending in c3d.txt
    fileBrowser=FileBrowser(dirBrowser.folderFullPath(i), '\*c3d.txt');
    for j=1:fileBrowser.numFiles
        disp(fileBrowser.file(j));
        %read the file
        [proximal,orientation,humerusLength,startEndIndices]=readV3DExport(fileBrowser.fileFullPath(j),period);
        %keep just the portion of the activity that we care about
        startEndIndices=modifyStartEndIndices(startEndIndices,fileBrowser.file(j),gaussInterval);
        [proximal,orientation]=truncateTrajectory(proximal,orientation,startEndIndices);
        %call the smoothing function
        [framesSmooth,errorsP,errorsO]=smoothAndPlot(proximal,orientation,period,true,gaussInterval,padLength,padRemoval);
        %should place breakpoint here at end to be able to view each
        %individually
    end
end