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
resultsFile=params.postAnalysisResultsFile;
dataFolder=params.postAnalysisDataFolder;
butterworthCutoff=params.butterworthCutoff;
butterworthOrder=params.butterworthOrder;
poseCsRbs=params.postAnalysisPoseRbCs;
velCsRbs=params.postAnalysisVelRbCs;
accCsRbs=params.postAnalysisAccRbCs;
useDtw=params.postAnalysisPerformDtw;
monitorNum=params.postAnalysisMonitorNum;
plotDiff=params.postAnalysisPlotDiff;
plotPer=params.postAnalysisPlotPercentage;
postAnalysisAngAccDtw=params.postAnalysisAngAccDtw;
postAnalysisRepeatsAvail=params.postAnalysisRepeatsAvail;
postAnalysisOffsetEndpts=params.postAnalysisOffsetEndpts;

%sampling periods
samplingPeriods.ndiSamplingPeriod=params.ndiSamplingPeriod;
samplingPeriods.mocapSamplingPeriod=params.period;
samplingPeriods.jointsSamplingPeriod=params.tickDefinition;


% Initialization for graphs
%colors for graphs
plotStyle.leftColor=[7 114 234]./255;
plotStyle.rightColor=[234 127 7]./255;
plotStyle.triColor=[236 77 3; 3 236 77; 77 3 236]./255;

%line widths
plotStyle.desiredWidth=0.5;
plotStyle.actualWidth=2;

%line styles
plotStyle.desiredStyle='-';
plotStyle.achievedStyle=':';

%other parameters
plotStyle.yLimMult=0.9;
plotStyle.width=800;
plotStyle.height=600;

close all
if postAnalysisRepeatsAvail
    checkFileFunc=@checkFileFunctionRepeats;
    if useDtw
        checkFileFunc=@checkFileFunctionRepeatsDtw;
    end
else
    checkFileFunc=@checkFileFunction;
    if useDtw
        checkFileFunc=@checkFileFunctionDtw;
    end
end

smoothDataFunctions.ndi=createButterworthFilter(butterworthOrder,butterworthCutoff,1/samplingPeriods.ndiSamplingPeriod);
smoothDataFunctions.joints=createButterworthFilter(butterworthOrder,butterworthCutoff,1/samplingPeriods.jointsSamplingPeriod);
fileStats=checkFileFunc(plotStyle,samplingPeriods,postAnalysisAngAccDtw,robotI,dataFolder,resultsFile,poseCsRbs,velCsRbs,accCsRbs,monitorNum,plotDiff,plotPer,postAnalysisRepeatsAvail,postAnalysisOffsetEndpts,smoothDataFunctions);