[path,~,~]=fileparts(mfilename('fullpath'));
addpath(path);
%add initClasses
addpath(fullfile(path,'..','..','initClasses'));
addpath(fullfile(path,'..','..','externalLibs','xml2struct'));

%initialize
params=Parameters(fullfile(path,'..','..','parameters','parametersM20.xml'));

%parameters
c3dFile=params.subsampleC3DFile;
subsamplePercentage=params.subsamplePerBF;
period=params.period;

%read the c3d file
[~,~,humerusLength,~]=readV3DExport(c3dFile,period);
%read frames file
[c3dFilePath,c3dName,ext]=fileparts(c3dFile);
[frames,~]=readFramesFile(smoothFramesFileName(c3dFilePath,strcat(c3dName,ext)));
%compute frame differences
[~,framesDxA] = cumFrameDiff(frames);
framesDxA = prependIdentity(framesDxA);
%extract the proximal, orientation, and distal components
[proximal,distal,rotationM,rotv]=extractComponents(frames,framesDxA,humerusLength);
%subsample
indices=SubSample(proximal,distal,rotationM,subsamplePercentage);
%write out the content to plot
[c3dFilePath,c3dName,ext]=fileparts(c3dFile);
csvwrite(plotFileName(c3dFilePath,strcat(c3dName,ext)),[proximal distal rotv]);
%write out the indices file
csvwrite(indicesFileName(c3dFilePath,strcat(c3dName,ext)),indices);
