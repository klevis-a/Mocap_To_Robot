[path,~,~]=fileparts(mfilename('fullpath'));
addpath(path);
%add initClasses
addpath(fullfile(path,'..','..','initClasses'));
addpath(fullfile(path,'..','..','externalLibs','xml2struct'));

%initialize
params=Parameters(fullfile(path,'..','..','parameters','parametersM20.xml'));
addpath(params.mocapToRobotLibPath);
initMocapToRobotLib();
robotI=RobotInfo(fullfile(path,'..','..','parameters','robotM20.xml'));

%post analysis info
resultsFolder=params.postAnalysisMainResultsFolder;
dataFolder=params.postAnalysisDataFolder;
butterworthCutoff=params.butterworthCutoff;
butterworthOrder=params.butterworthOrder;
%sampling periods
samplingPeriods.ndiSamplingPeriod=params.ndiSamplingPeriod;
samplingPeriods.mocapSamplingPeriod=params.period;
samplingPeriods.jointsSamplingPeriod=params.tickDefinition;

smoothDataFunctions.ndi=createButterworthFilter(butterworthOrder,butterworthCutoff,1/samplingPeriods.ndiSamplingPeriod);
smoothDataFunctions.joints=createButterworthFilter(butterworthOrder,butterworthCutoff,1/samplingPeriods.jointsSamplingPeriod);

%read the directories (each directory corresponds to a day of data collection)
dirBrowser=DirectoryBrowser(resultsFolder);

for i=1:dirBrowser.numFolders
    fileBrowser=FileBrowser(dirBrowser.folderFullPath(i), '\*_sum.txt');
    for j=1:fileBrowser.numFiles
        disp(fileBrowser.file(j))
        regeneratePDFFunc(fileBrowser.fileFullPath(j),dataFolder,samplingPeriods,robotI,smoothDataFunctions)
    end
end

