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

%read parameters specific to batch processing
batchProcessPrintDir=params.batchProcessPrintDir;
batchProcessJJMocapData=params.batchProcessJJMocapData;
batchProcessJLMocapData=params.batchProcessJLMocapData;
batchProcessIRMocapData=params.batchProcessIRMocapData;
batchProcessJOMocapData=params.batchProcessJOMocapData;
batchProcessJJVerificationData=params.batchProcessJJVerificationData;
batchProcessJLVerificationData=params.batchProcessJLVerificationData;
batchProcessIRVerificationData=params.batchProcessIRVerificationData;
batchProcessJOVerificationData=params.batchProcessJOVerificationData;
batchProcessJJ=params.batchProcessJJ;
batchProcessJL=params.batchProcessJL;
batchProcessIR=params.batchProcessIR;
batchProcessJO=params.batchProcessJO;

%coordinate system and rigid bodies
poseCsRbs=params.postAnalysisPoseRbCs;
velCsRbs=params.postAnalysisVelRbCs;
accCsRbs=params.postAnalysisAccRbCs;

%post analysis options
postAnalysisOffsetEndpts=params.postAnalysisOffsetEndpts;

%sampling periods
samplingPeriods.ndiSamplingPeriod=params.ndiSamplingPeriod;
samplingPeriods.mocapSamplingPeriod=params.period;
samplingPeriods.jointsSamplingPeriod=params.tickDefinition;

%plotting options for not plotting differences
noDiffPlotOptions.createGraphsBool=0;
noDiffPlotOptions.printGraphsBool=1;
noDiffPlotOptions.postAnalysisPlotDiff=0;
noDiffPlotOptions.postAnalysisPlotPercentage=1;

%plotting options for plotting differences in percentages
diffPlotOptions.createGraphsBool=0;
diffPlotOptions.printGraphsBool=1;
diffPlotOptions.postAnalysisPlotDiff=1;
diffPlotOptions.postAnalysisPlotPercentage=1;

%smoothing functions
butterworthCutoff=params.butterworthCutoff;
butterworthOrder=params.butterworthOrder;
smoothDataFunctions.ndi=createButterworthFilter(butterworthOrder,butterworthCutoff,1/samplingPeriods.ndiSamplingPeriod);
smoothDataFunctions.joints=createButterworthFilter(butterworthOrder,butterworthCutoff,1/samplingPeriods.jointsSamplingPeriod);

%create a cell array that defines what we will print
%columns - function that finds folder name, whether to use DTW or not,
%ploting options
jobOptions = {1,{noDiffPlotOptions,diffPlotOptions},{@dtwFolder,@dtwDiffPerFolder};
              0,{noDiffPlotOptions,diffPlotOptions},{@regFolder,@regDiffPerFolder}};
numJobOptions=size(jobOptions,1);

if batchProcessJJ
    jjStats=cell(numJobOptions,1);
    jjFolder=fullfile(batchProcessPrintDir,'JJ');
    %run JJ
    for n=1:numJobOptions
        jjStats{n}=postAnalyzeMultipleFoldersBatch(samplingPeriods,robotI,batchProcessJJMocapData,batchProcessJJVerificationData,poseCsRbs,velCsRbs,accCsRbs,...
            0,jobOptions{n,1},0,1,jobOptions{n,2},cellfun(@(c) c(jjFolder),jobOptions{n,3},'UniformOutput',false),postAnalysisOffsetEndpts,smoothDataFunctions);
        jjStats{n}=calcSummaryStats(jjStats{n});
    end
end

if batchProcessJL
    %run JL single
    jlStatsSingle=cell(numJobOptions,1);
    jlSingleFolder=fullfile(batchProcessPrintDir,'JL','Single');
    for n=1:size(jobOptions,1)
        jlStatsSingle{n}=postAnalyzeMultipleFoldersBatch(samplingPeriods,robotI,batchProcessJLMocapData,batchProcessJLVerificationData,poseCsRbs,velCsRbs,accCsRbs,...
            0,jobOptions{n,1},0,1,jobOptions{n,2},cellfun(@(c) c(jlSingleFolder),jobOptions{n,3},'UniformOutput',false),postAnalysisOffsetEndpts,smoothDataFunctions);
        jlStatsSingle{n}=calcSummaryStats(jlStatsSingle{n});
    end
    
    %run JL repeats
    jlStatsRepeats=cell(numJobOptions,1);
    jlRepeatsFolder=fullfile(batchProcessPrintDir,'JL','Repeats');
    for n=1:size(jobOptions,1)
        jlStatsRepeats{n}=postAnalyzeMultipleFoldersBatch(samplingPeriods,robotI,batchProcessJLMocapData,batchProcessJLVerificationData,poseCsRbs,velCsRbs,accCsRbs,...
            10,jobOptions{n,1},0,1,jobOptions{n,2},cellfun(@(c) c(jlRepeatsFolder),jobOptions{n,3},'UniformOutput',false),postAnalysisOffsetEndpts,smoothDataFunctions);
        jlStatsRepeats{n}=calcSummaryStats(jlStatsRepeats{n});
    end
end

if batchProcessIR
    irStats=cell(numJobOptions,1);
    irFolder=fullfile(batchProcessPrintDir,'IR');
    %run IR
    for n=1:numJobOptions
        irStats{n}=postAnalyzeMultipleFoldersBatch(samplingPeriods,robotI,batchProcessIRMocapData,batchProcessIRVerificationData,poseCsRbs,velCsRbs,accCsRbs,...
            0,jobOptions{n,1},0,1,jobOptions{n,2},cellfun(@(c) c(irFolder),jobOptions{n,3},'UniformOutput',false),postAnalysisOffsetEndpts,smoothDataFunctions);
        irStats{n}=calcSummaryStats(irStats{n});
    end
end

if batchProcessJO
    joStats=cell(numJobOptions,1);
    joFolder=fullfile(batchProcessPrintDir,'JO');
    %run JO
    for n=1:numJobOptions
        joStats{n}=postAnalyzeMultipleFoldersBatch(samplingPeriods,robotI,batchProcessJOMocapData,batchProcessJOVerificationData,poseCsRbs,velCsRbs,accCsRbs,...
            0,jobOptions{n,1},0,1,jobOptions{n,2},cellfun(@(c) c(joFolder),jobOptions{n,3},'UniformOutput',false),postAnalysisOffsetEndpts,smoothDataFunctions);
        joStats{n}=calcSummaryStats(joStats{n});
    end
end

clearvars -except irStats jjStats jlStatsSingle jlStatsRepeats joStats params