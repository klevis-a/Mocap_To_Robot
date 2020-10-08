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

%post analysis options
postAnalysisOffsetEndpts=params.postAnalysisOffsetEndpts;

%sampling periods
samplingPeriods.ndiSamplingPeriod=params.ndiSamplingPeriod;
samplingPeriods.mocapSamplingPeriod=params.period;
samplingPeriods.jointsSamplingPeriod=params.tickDefinition;

%plotting options for not plotting differences
noDiffPlotOptions.createGraphsBool=0;
noDiffPlotOptions.printGraphsBool=1;

%smoothing functions
butterworthCutoff=params.butterworthCutoff;
butterworthOrder=params.butterworthOrder;
smoothDataFunctions.ndi=createButterworthFilter(butterworthOrder,butterworthCutoff,1/samplingPeriods.ndiSamplingPeriod);
smoothDataFunctions.joints=createButterworthFilter(butterworthOrder,butterworthCutoff,1/samplingPeriods.jointsSamplingPeriod);

%create a cell array that defines what we will print
%columns - function that finds folder name, whether to use DTW or not,
%ploting options
jobOptions = {1,{noDiffPlotOptions},{@dtwFolder};
              0,{noDiffPlotOptions},{@regFolder}};
numJobOptions=size(jobOptions,1);

if batchProcessJJ
    jjStats=cell(numJobOptions,1);
    jjFolder=fullfile(batchProcessPrintDir,'JJ');
    %run JJ
    for n=1:numJobOptions
        jjStats{n}=postAnalyzeKineticsMultipleFoldersBatch(samplingPeriods,robotI,batchProcessJJMocapData,batchProcessJJVerificationData,...
            0,jobOptions{n,1},1,jobOptions{n,2},cellfun(@(c) c(jjFolder),jobOptions{n,3},'UniformOutput',false),postAnalysisOffsetEndpts,smoothDataFunctions);
        jjStats{n}=calcSummaryStatsKinetics(jjStats{n});
    end
end

if batchProcessJL
    %run JL single
    jlStatsSingle=cell(numJobOptions,1);
    jlSingleFolder=fullfile(batchProcessPrintDir,'JL','Single');
    for n=1:size(jobOptions,1)
        jlStatsSingle{n}=postAnalyzeKineticsMultipleFoldersBatch(samplingPeriods,robotI,batchProcessJLMocapData,batchProcessJLVerificationData,...
            0,jobOptions{n,1},1,jobOptions{n,2},cellfun(@(c) c(jlSingleFolder),jobOptions{n,3},'UniformOutput',false),postAnalysisOffsetEndpts,smoothDataFunctions);
        jlStatsSingle{n}=calcSummaryStatsKinetics(jlStatsSingle{n});
    end
    
    %run JL repeats
    jlStatsRepeats=cell(numJobOptions,1);
    jlRepeatsFolder=fullfile(batchProcessPrintDir,'JL','Repeats');
    for n=1:size(jobOptions,1)
        jlStatsRepeats{n}=postAnalyzeKineticsMultipleFoldersBatch(samplingPeriods,robotI,batchProcessJLMocapData,batchProcessJLVerificationData,...
            10,jobOptions{n,1},1,jobOptions{n,2},cellfun(@(c) c(jlRepeatsFolder),jobOptions{n,3},'UniformOutput',false),postAnalysisOffsetEndpts,smoothDataFunctions);
        jlStatsRepeats{n}=calcSummaryStatsKinetics(jlStatsRepeats{n});
    end
end

if batchProcessIR
    irStats=cell(numJobOptions,1);
    irFolder=fullfile(batchProcessPrintDir,'IR');
    %run IR
    for n=1:numJobOptions
        irStats{n}=postAnalyzeKineticsMultipleFoldersBatch(samplingPeriods,robotI,batchProcessIRMocapData,batchProcessIRVerificationData,...
            0,jobOptions{n,1},1,jobOptions{n,2},cellfun(@(c) c(irFolder),jobOptions{n,3},'UniformOutput',false),postAnalysisOffsetEndpts,smoothDataFunctions);
        irStats{n}=calcSummaryStatsKinetics(irStats{n});
    end
end

if batchProcessJO
    joStats=cell(numJobOptions,1);
    joFolder=fullfile(batchProcessPrintDir,'JO');
    %run JO
    for n=1:numJobOptions
        joStats{n}=postAnalyzeKineticsMultipleFoldersBatch(samplingPeriods,robotI,batchProcessJOMocapData,batchProcessJOVerificationData,...
            0,jobOptions{n,1},1,jobOptions{n,2},cellfun(@(c) c(joFolder),jobOptions{n,3},'UniformOutput',false),postAnalysisOffsetEndpts,smoothDataFunctions);
        joStats{n}=calcSummaryStatsKinetics(joStats{n});
    end
end

clearvars -except irStats jjStats jlStatsSingle jlStatsRepeats joStats params