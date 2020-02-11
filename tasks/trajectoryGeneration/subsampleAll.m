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
directory=params.subsampleAllDir;
period=params.period;
subsamplePercentage=params.subsamplePerAll;

%read the directories (each directory corresponds to a subject)
dirBrowser=DirectoryBrowser(directory);

%loop through each directory
parfor (i=1:dirBrowser.numFolders,4)
    %find files ending in joints.txt
    fileBrowser=FileBrowser(dirBrowser.folderFullPath(i), '\*c3d.txt');
    for j=1:fileBrowser.numFiles
        %display the current file
        disp(fileBrowser.file(j));
        %read the c3d file
        [~,~,humerusLength,~]=readV3DExport(fileBrowser.fileFullPath(j),period);
        %read frames file
        fFile=smoothFramesFileName(dirBrowser.folderFullPath(i),fileBrowser.file(j));
        if ~isfile(fFile)
            continue;
        end
        [frames,~]=readFramesFile(fFile);
        %compute frame differences
        [~,framesDxA] = cumFrameDiff(frames);
        framesDxA = prependIdentity(framesDxA);
        %extract the proximal, orientation, and distal components
        [proximal,distal,rotationM,rotv]=extractComponents(frames,framesDxA,humerusLength);
        %subsample
        indices=SubSample(proximal,distal,rotationM,subsamplePercentage);
        %write out the content to plot
        [c3dFilePath,c3dName,ext]=fileparts(fileBrowser.fileFullPath(j));
        csvwrite(plotFileName(c3dFilePath,strcat(c3dName,ext)),[proximal distal rotv]);
        %write out the indices file
        csvwrite(indicesFileName(c3dFilePath,strcat(c3dName,ext)),indices);
    end
end