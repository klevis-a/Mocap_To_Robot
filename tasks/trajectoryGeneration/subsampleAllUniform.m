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
subsampleFrequency=params.subsampleEveryOther;

%read the directories (each directory corresponds to a subject)
dirBrowser=DirectoryBrowser(directory);

%loop through each directory
for i=1:dirBrowser.numFolders
    %find files ending in joints.txt
    fileBrowser=FileBrowser(dirBrowser.folderFullPath(i), '\*c3d.txt');
    for j=1:fileBrowser.numFiles
        %display the current file
        disp(fileBrowser.file(j));
        pIndices=csvread(indicesFileName(dirBrowser.folderFullPath(i),...
            fileBrowser.file(j)));
        indices=SubSampleUniform(pIndices(end),subsampleFrequency);
        csvwrite(indicesUniformFileName(dirBrowser.folderFullPath(i),...
            fileBrowser.file(j)), indices);
    end
end