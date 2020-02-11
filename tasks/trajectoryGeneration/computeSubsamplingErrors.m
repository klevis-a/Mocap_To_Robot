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
directory=params.computeSubsamplingErrorsDir;
period=params.period;
method=params.subsamplingMethodErrors;

%read the directories (each directory corresponds to a subject)
dirBrowser=DirectoryBrowser(directory);

%loop through each directory
for i=1:dirBrowser.numFolders
    %find files ending in joints.txt
    fileBrowser=FileBrowser(dirBrowser.folderFullPath(i), '\*c3d.txt');
    for j=1:fileBrowser.numFiles
        %display the current file
        disp(fileBrowser.file(j));
        %read the c3d file
        [~,~,humerusLength,~]=readV3DExport(fileBrowser.fileFullPath(j),period);
        %read frames file
        [frames,~]=readFramesFile(smoothFramesFileName(dirBrowser.folderFullPath(i),fileBrowser.file(j)));
        %compute frame differences
        [~,framesDxA] = cumFrameDiff(frames);
        framesDxA = prependIdentity(framesDxA);
        %extract the proximal, orientation, and distal components
        [proximal,distal,rotationM,~]=extractComponents(frames,framesDxA,humerusLength);
        %read the indices
        if method==0
            indices=csvread(indicesUniformFileName(dirBrowser.folderFullPath(i),fileBrowser.file(j)));
        else
            indices=csvread(indicesFileName(dirBrowser.folderFullPath(i),fileBrowser.file(j)));
        end
        %compute errors
        [proximalE,distalE,orientationE]=computeSubSamplingError(proximal,distal,rotationM,indices);
        %write out to file
        csvwrite(subsamplingErrorsFileName(dirBrowser.folderFullPath(i),fileBrowser.file(j),method), [proximalE distalE orientationE]);
    end
end