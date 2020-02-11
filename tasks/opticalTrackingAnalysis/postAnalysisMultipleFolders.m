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
postAnalysisMainResultsFolder=params.postAnalysisMainResultsFolder;
postAnalysisDataFolder=params.postAnalysisDataFolder;
butterworthCutoff=params.butterworthCutoff;
butterworthOrder=params.butterworthOrder;
poseCsRbs=params.postAnalysisPoseRbCs;
velCsRbs=params.postAnalysisVelRbCs;
accCsRbs=params.postAnalysisAccRbCs;
postAnalysisRepeatsAvail=params.postAnalysisRepeatsAvail;
postAnalysisPerformDtw=params.postAnalysisPerformDtw;
postAnalysisAngAccDtw=params.postAnalysisAngAccDtw;
computeStats=params.postAnalysisComputeStats;
postAnalysisOffsetEndpts=params.postAnalysisOffsetEndpts;
postAnalysisPrintDir=params.postAnalysisPrintDir;
%sampling periods
samplingPeriods.ndiSamplingPeriod=params.ndiSamplingPeriod;
samplingPeriods.mocapSamplingPeriod=params.period;
samplingPeriods.jointsSamplingPeriod=params.tickDefinition;
%plot options
plotOptions.createGraphsBool=params.postAnalysisCreateGraphs;
plotOptions.printGraphsBool=params.postAnalysisPrintGraphs;
plotOptions.postAnalysisPlotDiff=params.postAnalysisPlotDiff;
plotOptions.postAnalysisPlotPercentage=params.postAnalysisPlotPercentage;

smoothDataFunctions.ndi=createButterworthFilter(butterworthOrder,butterworthCutoff,1/samplingPeriods.ndiSamplingPeriod);
smoothDataFunctions.joints=createButterworthFilter(butterworthOrder,butterworthCutoff,1/samplingPeriods.jointsSamplingPeriod);
allFolderStats=postAnalyzeMultipleFolders(samplingPeriods,robotI,postAnalysisDataFolder,postAnalysisMainResultsFolder,poseCsRbs,velCsRbs,accCsRbs,...
    postAnalysisRepeatsAvail,postAnalysisPerformDtw,postAnalysisAngAccDtw,computeStats,plotOptions,postAnalysisPrintDir,postAnalysisOffsetEndpts,smoothDataFunctions);
allFolderStats=calcSummaryStats(allFolderStats);