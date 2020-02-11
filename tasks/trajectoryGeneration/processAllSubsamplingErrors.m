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
directory=params.subsampleErrorsDir;
method=params.subsamplingMethodErrors;

%read the directories (each directory corresponds to a subject)
dirBrowser=DirectoryBrowser(directory);

%statistics functions to run over data
statsFunc={@rms, @mean, @median, @std, @(x) mad(x,0),@(x) mad(x,1), @min,...
    @max,@(x) quantile(x,0.25), @(x) quantile(x,0.75)};
numFunc=length(statsFunc);

%rms proximal,rms distal,rms orientation, max proximal,max distal, max
%orientation
errorSummary=zeros(1,numFunc,3);

counter=1;
%loop through each directory
for i=1:dirBrowser.numFolders
    %find files ending in joints.txt
    fileBrowser=FileBrowser(dirBrowser.folderFullPath(i), '\*c3d.txt');
    for j=1:fileBrowser.numFiles
        %display the current file
        disp(fileBrowser.file(j));
        [proximal,distal,orient]=readSSErrorsFile(subsamplingErrorsFileName(dirBrowser.folderFullPath(i),...
            fileBrowser.file(j),method));
        
        %remove zeros, sometimes we get an imaginary component when
        %examining orientation simply because the answer is so close to
        %zero
        proximalNoZero=proximal(proximal~=0);
        distalNoZero=distal(distal~=0);
        orientNoZero=real(orient(orient~=0));
        
        %run each statistics function over the dataset
        for n=1:numFunc
            currentFunc=statsFunc{n};
            errorSummary(counter,n,1)=currentFunc(proximalNoZero);
            errorSummary(counter,n,2)=currentFunc(distalNoZero);
            errorSummary(counter,n,3)=currentFunc(orientNoZero);
        end
        
        counter=counter+1;
    end
end
