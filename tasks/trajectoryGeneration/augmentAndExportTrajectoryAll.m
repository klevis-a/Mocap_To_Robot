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
directory=params.smoothAndExportAllDir;
period=params.period;
augmentNumPoints=params.augmentNumPoints;
interpNumPoints=params.interpNumPoints;
gaussInterval=params.smoothGaussInterval;

%read the directories (each directory corresponds to a subject)
dirBrowser=DirectoryBrowser(directory);

%loop through each directory
for i=1:dirBrowser.numFolders
    %find files ending in c3d.txt
    fileBrowser=FileBrowser(dirBrowser.folderFullPath(i), '\*c3d.txt');
    for j=1:fileBrowser.numFiles
        disp(fileBrowser.file(j));
        
        %read the file
        [proximal,orientation,~,startEndIndices]=readV3DExport(fileBrowser.fileFullPath(j),period);
        if isempty(startEndIndices)
            disp('Skipping')
            continue;
        end
        %keep just the portion of the activity that we care about
        startEndIndices=modifyStartEndIndices(startEndIndices,fileBrowser.file(j),gaussInterval);

        %truncate the trajectory
        [proximal,orientation]=truncateTrajectory(proximal,orientation,startEndIndices);

        %create frames
        frames=createFrames(proximal,orientation);

        [newFrames,~]=augmentTrajectory(frames,period,augmentNumPoints,interpNumPoints,gaussInterval,0);
        writeSmoothedTrajectory(newFrames,period,smoothFramesFileName(dirBrowser.folderFullPath(i),fileBrowser.file(j)));
    end
end