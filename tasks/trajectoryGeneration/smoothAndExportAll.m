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
gaussInterval=params.smoothGaussInterval;
padLength=params.smoothPadLength;
padRemoval=params.smoothPadRemoval;

%read the directories (each directory corresponds to a subject)
dirBrowser=DirectoryBrowser(directory);

%smoothing errors
allPErrors=zeros(dirBrowser.numFolders*3,3);
allPErrorsI=zeros(dirBrowser.numFolders*3,3);
allOErrors=zeros(dirBrowser.numFolders*3,3);
allOErrorsI=zeros(dirBrowser.numFolders*3,3);
names={};

%loop through each directory
for i=1:dirBrowser.numFolders
    %find files ending in c3d.txt
    fileBrowser=FileBrowser(dirBrowser.folderFullPath(i), '\*c3d.txt');
    for j=1:fileBrowser.numFiles
        disp(fileBrowser.file(j));
        %read the file
        [proximal,orientation,humerusLength,startEndIndices]=readV3DExport(fileBrowser.fileFullPath(j),period);
        if isempty(startEndIndices)
            continue;
        end
        %keep just the portion of the activity that we care about
        startEndIndices=modifyStartEndIndices(startEndIndices,fileBrowser.file(j),gaussInterval);
        
        %check to see if the startEndIndices go beyond the start and end of
        %capture
        if startEndIndices(1)<=0 || startEndIndices(2)>size(proximal,1)
            disp('Skipping')
            continue;
        end
        [proximal,orientation]=truncateTrajectory(proximal,orientation,startEndIndices);
        %call the smoothing function
        [framesSmooth,errorsP,errorsO]=smoothAndPlot(proximal,orientation,period,false,gaussInterval,padLength,padRemoval);
        %store smoothing errors
        allPErrors((i-1)*3+j,:)=errorsP(2,:);
        allPErrorsI((i-1)*3+j,:)=errorsP(1,:);
        allOErrors((i-1)*3+j,:)=errorsO(2,:);
        allOErrorsI((i-1)*3+j,:)=errorsO(1,:);
        names{(i-1)*3+j}=fileBrowser.file(j);
        %write smoothed trajectory to files
        writeSmoothedTrajectory(framesSmooth,period,...
            smoothFramesFileName(dirBrowser.folderFullPath(i),fileBrowser.file(j)));
    end
end