classdef Parameters    
    properties
        %----------------------------
        %
        % MocapToRobot library path
        %
        %MocapToRobot library path
        mocapToRobotLibPath
        %
        %----------------------------
        %
        % Mocap Info
        %
        %mocap period
        period
        %
        %----------------------------
        %
        % Common to all methods that plot optimized trajectories
        %
        %whether to process the second optimization or first
        processTrajOpt;
        %
        %----------------------------
        %
        % Pseudoinverse algorithm
        %
        %pseudoinverse algorithm starting joint angles
        pInvJoints0
        %pseudoinverse algorithm desired frames file
        pInvDesFramesFile
        %
        %----------------------------
        %
        % Smooth and Export
        %
        %smooth and plot all directory
        smoothAndPlotAllDir
        %smooth and export all directory
        smoothAndExportAllDir
        %smooth and plot file
        smoothAndPlotFile
        %smooth and export C3D file
        smoothAndExportC3DFile
        %smooth and export "export" file
        smoothAndExportFile
        %smoothing gauss interval
        smoothGaussInterval
        %smoothing pad length
        smoothPadLength
        %smoothing pad removal length
        smoothPadRemoval
        %number of points to amend a trajectory by
        augmentNumPoints
        %number of points to use when interpolating trajectory
        interpNumPoints
        %
        %----------------------------
        %
        % Plot and Process Optimized
        %
        %plot all optimized directory
        plotAllOptDir
        %process all optimized directory
        processAllOptDir
        %plot optimized trajectory joints files
        plotOptTrajJointsFile
        %plot optimized trajectory smooth frames file
        plotOptTrajFramesFile
        %alternate toolframes file for plotting
        plotOptTrajToolframe
        %
        %----------------------------
        %
        % Subsample
        %
        %subsample c3d file
        subsampleC3DFile
        %subsample percentage for subsample by file
        subsamplePerBF
        %directory for batch subsampling
        subsampleAllDir
        %subsample percentage for batch processing
        subsamplePerAll
        %directory for processing subsample errors
        subsampleErrorsDir
        %subsample every other
        subsampleEveryOther
        %computeSubsamplingErrorsDir
        computeSubsamplingErrorsDir
        %subsamplingMethodErrors
        subsamplingMethodErrors
        %
        %----------------------------
        %
        % Write TP program
        %
        %teach pendant program multiplier
        tpProgramMultiplier
        %J1 offset
        tpProgramJ1
        %whether to treat the J1 setting as an offset or absolute
        tpProgramJ1Offset;
        %joints file for program
        tpProgramJointsFile
        %indices file for program
        tpProgramIndicesFile
        %
        %----------------------------
        %
        % Data Processing
        %
        %folder where the axis calibration data is stored
        axisCalibrationFolder
        %J1 offset for calibration process
        axisCalibrationJ1Offset
        %process tcp along with axis calibration
        axisCalibrationTcp
        %axis calibration results
        axisCalibrationResults
        %process MoCap folder
        processMocapFolder
        %process MoCap program name
        processMocapProgramName
        %process MoCap C3D file
        processMocapC3DFile
        %mocap processing period
        ndiSamplingPeriod
        %how long is a tick
        tickDefinition
        %should we print to pdf
        processMocapPrint
        %should we just print graphs to determine a useful peak prominence
        processMocapJustGraph
        %
        %----------------------------
        %
        % Post Analysis
        %
        %post analysis results folder
        postAnalysisResultsFolder
        %post analysis results file
        postAnalysisResultsFile
        %post analysis main results folder
        postAnalysisMainResultsFolder
        %post analysis data folder
        postAnalysisDataFolder
        %post analysis create graphs or not
        postAnalysisCreateGraphs
        %post analysis print graphs or not
        postAnalysisPrintGraphs
        %post analysis compute statistics or not
        postAnalysisComputeStats
        %post analysis pose rigid bodies and coordinate systems
        postAnalysisPoseRbCs
        %post analysis velocity rigid bodies and coordinate systems
        postAnalysisVelRbCs
        %post analysis acceleration rigid bodies and coordinate systems
        postAnalysisAccRbCs
        %whether to use dtw for in post analysis
        postAnalysisPerformDtw
        %which monitor to plot on
        postAnalysisMonitorNum
        %whether to include angular acceleration in DTW
        postAnalysisAngAccDtw
        %whether there is repeat trials to use for averaging
        postAnalysisRepeatsAvail
        %where to plot differences or actual values
        postAnalysisPlotDiff
        %whether to plot percentages or not
        postAnalysisPlotPercentage
        %directory where to print graphs
        postAnalysisPrintDir
        %whether to offset the endpts when computing statistics - this may
        %be necessary because of smoothing or trajectory extension (such as
        %in the case of jogging) but may not make a big difference
        postAnalysisOffsetEndpts
        %butterworth cutoff frequency
        butterworthCutoff
        %butterworth filter order
        butterworthOrder
        %
        %----------------------------
        %
        % Batch Processing
        %
        %where to print graphs
        batchProcessPrintDir
        %directories for where original motion capture data resides
        batchProcessJJMocapData
        batchProcessJLMocapData
        batchProcessIRMocapData
        batchProcessJOMocapData
        %directories for where the verification data (NDI captures) reside
        batchProcessJJVerificationData
        batchProcessJLVerificationData
        batchProcessIRVerificationData
        batchProcessJOVerificationData
        %whether to process each activity
        batchProcessJJ
        batchProcessJL
        batchProcessIR
        batchProcessJO
    end
    
    methods
        function obj = Parameters(file)
            %read the xml initialization file
            params=xml2struct(file);
            %----------------------------
            %
            % MocapToRobot library path
            %
            %MocapToRobot library path
            obj.mocapToRobotLibPath=params.parameters.mocapToRobotLibPath.Text;
            %
            %----------------------------
            %
            % Mocap Info
            %
            %mocap period
            obj.period=str2num(params.parameters.period.Text);
            %   
            %----------------------------
            %
            % Pseudoinverse algorithm
            %
            %pseudoinverse algorithm desired frames file
            obj.pInvDesFramesFile=params.parameters.pInvDesFramesFile.Text;
            %
            %pseudoinverse algorithm starting joint angles
            obj.pInvJoints0=str2num(params.parameters.pInvJoints0.Text);
            %
            %----------------------------
            %
            % Smooth and Export
            %
            %smooth and plot all
            %
            obj.smoothAndPlotAllDir = params.parameters.smoothAndPlotAllDir.Text;
            %
            %smooth and export all
            %
            obj.smoothAndExportAllDir=params.parameters.smoothAndExportAllDir.Text;
            %
            %smooth and plot by file
            obj.smoothAndPlotFile=params.parameters.smoothAndPlotFile.Text;
            %
            %smooth and export by file
            %
            obj.smoothAndExportC3DFile=params.parameters.smoothAndExportC3DFile.Text;
            obj.smoothAndExportFile=params.parameters.smoothAndExportFile.Text;
            %smoothing gauss interval
            obj.smoothGaussInterval=str2num(params.parameters.smoothGaussInterval.Text);
            %smoothing pad length
            obj.smoothPadLength=str2num(params.parameters.smoothPadLength.Text);
            %smoothing pad removal length
            obj.smoothPadRemoval=str2num(params.parameters.smoothPadRemoval.Text);
            %number of points to amend a trajectory by
            obj.augmentNumPoints=str2num(params.parameters.augmentNumPoints.Text);
            %number of points to use when interpolating trajectory
            obj.interpNumPoints=str2num(params.parameters.interpNumPoints.Text);
            %
            %----------------------------
            %
            % Plot and Process Optimized
            %
            %plot all optimized trajectories
            %
            obj.plotAllOptDir=params.parameters.plotAllOptDir.Text;
            %
            %process all optimized trajectories
            %
            obj.processAllOptDir=params.parameters.processAllOptDir.Text;
            %
            %plot optimized trajectory by file
            %
            obj.plotOptTrajJointsFile=params.parameters.plotOptTrajJointsFile.Text;
            obj.plotOptTrajFramesFile=params.parameters.plotOptTrajFramesFile.Text;
            obj.plotOptTrajToolframe=params.parameters.plotOptTrajToolframe.Text;
            %
            %whether to process the second optimization or first
            %
            obj.processTrajOpt=str2num(params.parameters.processTrajOpt.Text);
            %
            %
            %----------------------------
            %
            % Subsample
            %
            %subsample all
            %
            obj.subsampleAllDir=params.parameters.subsampleAllDir.Text;
            %
            %batch subsample percentage
            %
            obj.subsamplePerAll=str2num(params.parameters.subsamplePerAll.Text);
            %subsample frames file
            %
            obj.subsampleC3DFile=params.parameters.subsampleC3DFile.Text;
            %
            %subsample percentage for subsample by file
            %
            obj.subsamplePerBF=str2num(params.parameters.subsamplePerBF.Text);
            %
            %uniform subsampling frequency
            obj.subsampleEveryOther=str2num(params.parameters.subsampleEveryOther.Text);
            %
            %computeSubsamplingErrorsDir
            %
            obj.computeSubsamplingErrorsDir=params.parameters.computeSubsamplingErrorsDir.Text;
            %subsamplingMethodErrors
            %
            obj.subsamplingMethodErrors=str2num(params.parameters.subsamplingMethodErrors.Text);
            %subsample errors directory
            %
            obj.subsampleErrorsDir=params.parameters.subsampleErrorsDir.Text;
            %
            %----------------------------
            %
            % Write TP program
            %
            %
            %multiplier
            obj.tpProgramMultiplier=str2num(params.parameters.tpProgramMultiplier.Text);
            %
            %J1 offset - Z rotation degree of freedom
            obj.tpProgramJ1=str2num(params.parameters.tpProgramJ1.Text);
            %
            %whether to treat the J1 setting as an offset or absolute
            obj.tpProgramJ1Offset=str2num(params.parameters.tpProgramJ1Offset.Text);
            %
            %joints file for program
            obj.tpProgramJointsFile=params.parameters.tpProgramJointsFile.Text;
            %
            %indices file for program
            obj.tpProgramIndicesFile=params.parameters.tpProgramIndicesFile.Text;
            %
            %----------------------------
            %
            % Data Processing
            %
            %folder where the axis calibration data is stored
            obj.axisCalibrationFolder=params.parameters.axisCalibrationFolder.Text;
            %J1 offset for axes calibration process
            obj.axisCalibrationJ1Offset=str2num(params.parameters.axisCalibrationJ1Offset.Text);
            %process tcp along with axis calibration
            obj.axisCalibrationTcp=str2num(params.parameters.axisCalibrationTcp.Text);
            %axis calibration results file
            obj.axisCalibrationResults=params.parameters.axisCalibrationResults.Text;
            %process MoCap folder
            obj.processMocapFolder=params.parameters.processMocapFolder.Text;
            %process MoCap program name
            obj.processMocapProgramName=params.parameters.processMocapProgramName.Text;
            %process MoCap C3D file
            obj.processMocapC3DFile=params.parameters.processMocapC3DFile.Text;
            %mocap processing period
            obj.ndiSamplingPeriod=str2num(params.parameters.ndiSamplingPeriod.Text);
            %how long is a tick
            obj.tickDefinition=str2num(params.parameters.tickDefinition.Text);
            %should we print the graphs to pdf
            obj.processMocapPrint = str2num(params.parameters.processMocapPrint.Text);
            %should we just print graphs to determine a useful peak prominence 
            obj.processMocapJustGraph = str2num(params.parameters.processMocapJustGraph.Text);
            %
            %----------------------------
            %
            % Post Analysis
            %
            %post analysis result folder
            obj.postAnalysisResultsFolder=params.parameters.postAnalysisResultsFolder.Text;
            %post analysis main results folder
            obj.postAnalysisMainResultsFolder=params.parameters.postAnalysisMainResultsFolder.Text;
            %post analysis results file
            obj.postAnalysisResultsFile=params.parameters.postAnalysisResultsFile.Text;
            %post analysis data folder
            obj.postAnalysisDataFolder=params.parameters.postAnalysisDataFolder.Text;
            %post analysis create graphs or not
            obj.postAnalysisCreateGraphs=str2num(params.parameters.postAnalysisCreateGraphs.Text);
            %post analysis print graphs or not
            obj.postAnalysisPrintGraphs=str2num(params.parameters.postAnalysisPrintGraphs.Text);
            %post analysis compute statistics or not
            obj.postAnalysisComputeStats=str2num(params.parameters.postAnalysisComputeStats.Text);
            %post analysis pose rigid bodies and coordinate systems
            postAnalysisPoseRbCsText=params.parameters.postAnalysisPoseRbCs.Text;
            postAnalysisPoseRbCsSplit=strsplit(postAnalysisPoseRbCsText,',');
            obj.postAnalysisPoseRbCs.linCS=postAnalysisPoseRbCsSplit{1};
            obj.postAnalysisPoseRbCs.linRB=postAnalysisPoseRbCsSplit{2};
            obj.postAnalysisPoseRbCs.rotCS=postAnalysisPoseRbCsSplit{3};
            obj.postAnalysisPoseRbCs.rotRB=postAnalysisPoseRbCsSplit{4};
            obj.postAnalysisPoseRbCs.eulerCS=postAnalysisPoseRbCsSplit{5};
            obj.postAnalysisPoseRbCs.eulerRB=postAnalysisPoseRbCsSplit{6};
            %post analysis velocity rigid bodies and coordinate systems
            postAnalysisVelRbCsText=params.parameters.postAnalysisVelRbCs.Text;
            postAnalysisVelRbCsSplit=strsplit(postAnalysisVelRbCsText,',');
            obj.postAnalysisVelRbCs.linCS=postAnalysisVelRbCsSplit{1};
            obj.postAnalysisVelRbCs.linRB=postAnalysisVelRbCsSplit{2};
            obj.postAnalysisVelRbCs.rotCS=postAnalysisVelRbCsSplit{3};
            obj.postAnalysisVelRbCs.rotRB=postAnalysisVelRbCsSplit{4};
            obj.postAnalysisVelRbCs.eulerCS=postAnalysisVelRbCsSplit{5};
            obj.postAnalysisVelRbCs.eulerRB=postAnalysisVelRbCsSplit{6};
            %post analysis acceleration rigid bodies and coordinate systems
            postAnalysisAccRbCsText=params.parameters.postAnalysisAccRbCs.Text;
            postAnalysisAccRbCsSplit=strsplit(postAnalysisAccRbCsText,',');
            obj.postAnalysisAccRbCs.linCS=postAnalysisAccRbCsSplit{1};
            obj.postAnalysisAccRbCs.linRB=postAnalysisAccRbCsSplit{2};
            obj.postAnalysisAccRbCs.rotCS=postAnalysisAccRbCsSplit{3};
            obj.postAnalysisAccRbCs.rotRB=postAnalysisAccRbCsSplit{4};
            obj.postAnalysisAccRbCs.eulerCS=postAnalysisAccRbCsSplit{5};
            obj.postAnalysisAccRbCs.eulerRB=postAnalysisAccRbCsSplit{6};
            %whether to use dtw for in post analysis
            obj.postAnalysisPerformDtw=str2num(params.parameters.postAnalysisPerformDtw.Text);
            %which monitor to plot on
            obj.postAnalysisMonitorNum=str2num(params.parameters.postAnalysisMonitorNum.Text);
            %whether to include angular acceleration in DTW
            obj.postAnalysisAngAccDtw=str2num(params.parameters.postAnalysisAngAccDtw.Text);
            %whether there is repeat trials to use for averaging
            obj.postAnalysisRepeatsAvail=str2num(params.parameters.postAnalysisRepeatsAvail.Text);
            %where to plot differences or actual values
            obj.postAnalysisPlotDiff=str2num(params.parameters.postAnalysisPlotDiff.Text);
            %whether to plot percentages or not
            obj.postAnalysisPlotPercentage=str2num(params.parameters.postAnalysisPlotPercentage.Text);
            %directory where to print graphs
            obj.postAnalysisPrintDir=params.parameters.postAnalysisPrintDir.Text;
            %whether to offset the endpts when computing statistics - this may
            %be necessary because of smoothing or trajectory extension (such as
            %in the case of jogging) but may not make a big difference
            obj.postAnalysisOffsetEndpts=str2num(params.parameters.postAnalysisOffsetEndpts.Text);
            %butterworth cutoff frequency
            obj.butterworthCutoff=str2num(params.parameters.butterworthCutoff.Text);
            %butterworth filter order
            obj.butterworthOrder=str2num(params.parameters.butterworthOrder.Text);
            %
            %----------------------------
            %
            % Batch Processing
            %
            %where to print graphs
            obj.batchProcessPrintDir=params.parameters.batchProcessPrintDir.Text;
            %directories for where original motion capture data resides
            obj.batchProcessJJMocapData=params.parameters.batchProcessJJMocapData.Text;
            obj.batchProcessJLMocapData=params.parameters.batchProcessJLMocapData.Text;
            obj.batchProcessIRMocapData=params.parameters.batchProcessIRMocapData.Text;
            obj.batchProcessJOMocapData=params.parameters.batchProcessJOMocapData.Text;
            %directories for where the verification data (NDI captures) reside
            obj.batchProcessJJVerificationData=params.parameters.batchProcessJJVerificationData.Text;
            obj.batchProcessJLVerificationData=params.parameters.batchProcessJLVerificationData.Text;
            obj.batchProcessIRVerificationData=params.parameters.batchProcessIRVerificationData.Text;
            obj.batchProcessJOVerificationData=params.parameters.batchProcessJOVerificationData.Text;
            %whether to process each activity
            obj.batchProcessJJ=str2num(params.parameters.batchProcessJJ.Text);
            obj.batchProcessJL=str2num(params.parameters.batchProcessJL.Text);
            obj.batchProcessIR=str2num(params.parameters.batchProcessIR.Text);
            obj.batchProcessJO=str2num(params.parameters.batchProcessJO.Text);
        end 
    end
end

