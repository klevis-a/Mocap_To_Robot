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
postAnalysisResultsFolder=params.postAnalysisResultsFolder;
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
postAnalysisPrintDir=params.postAnalysisPrintDir;
postAnalysisOffsetEndpts=params.postAnalysisOffsetEndpts;
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
folderStats=postAnalyzeFolder(samplingPeriods,robotI,postAnalysisDataFolder,postAnalysisResultsFolder,poseCsRbs,velCsRbs,accCsRbs,...
    postAnalysisRepeatsAvail,postAnalysisPerformDtw,postAnalysisAngAccDtw,computeStats,plotOptions,postAnalysisPrintDir,postAnalysisOffsetEndpts,smoothDataFunctions);
folderStats=calcSummaryStats(folderStats);