<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="20008000">
	<Property Name="NI.LV.All.SourceOnly" Type="Bool">true</Property>
	<Property Name="NI.Project.Description" Type="Str"></Property>
	<Item Name="My Computer" Type="My Computer">
		<Property Name="NI.SortType" Type="Int">3</Property>
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="autonomous paths" Type="Folder" URL="../../autonomous paths">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="autonomous plays" Type="Folder" URL="../../autonomous plays">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="calibrations and preferences" Type="Folder" URL="../../calibrations and preferences">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="documentation" Type="Folder" URL="../../documentation">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="limelight pipelines" Type="Folder" URL="../../limelight pipelines">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="Simulators" Type="Folder">
			<Item Name="ArmKinematicsSimulator.vi" Type="VI" URL="../ArmKinematicsSimulator.vi"/>
		</Item>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="vi.lib" Type="Folder">
				<Item Name="DH Kinematic Parameters.ctl" Type="VI" URL="/&lt;vilib&gt;/robotics/Kinematics/DH Kinematic Parameters.ctl"/>
				<Item Name="NI_Robotics_Link.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Link/NI_Robotics_Link.lvclass"/>
				<Item Name="NI_Matrix.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/Matrix/NI_Matrix.lvlib"/>
				<Item Name="NI_Kinematics.lvlib" Type="Library" URL="/&lt;vilib&gt;/robotics/Kinematics/NI_Kinematics.lvlib"/>
				<Item Name="NI_Robotics Utilities.lvlib" Type="Library" URL="/&lt;vilib&gt;/robotics/Utilities/NI_Robotics Utilities.lvlib"/>
				<Item Name="NI_AALBase.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/NI_AALBase.lvlib"/>
				<Item Name="NI_Robotics_Serial Robot Arm.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Robot/NI_Robotics_Serial Robot Arm.lvclass"/>
				<Item Name="NI_Robotics_3DKinematics.lvlib" Type="Library" URL="/&lt;vilib&gt;/robotics/Kinematics/Plotting/NI_Robotics_3DKinematics.lvlib"/>
				<Item Name="DH Dynamic Parameters.ctl" Type="VI" URL="/&lt;vilib&gt;/robotics/Kinematics/DH Dynamic Parameters.ctl"/>
				<Item Name="NI_AALPro.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/NI_AALPro.lvlib"/>
				<Item Name="LV3DPointTypeDef.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/miscctls.llb/LV3DPointTypeDef.ctl"/>
				<Item Name="NI_AngleManipulation.lvlib" Type="Library" URL="/&lt;vilib&gt;/Utility/AngleManip/NI_AngleManipulation.lvlib"/>
				<Item Name="NI_3D Picture Control.lvlib" Type="Library" URL="/&lt;vilib&gt;/picture/3D Picture Control/NI_3D Picture Control.lvlib"/>
				<Item Name="Color to RGB.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/colorconv.llb/Color to RGB.vi"/>
				<Item Name="Open_Create_Replace File.vi" Type="VI" URL="/&lt;vilib&gt;/_oldvers/_oldvers.llb/Open_Create_Replace File.vi"/>
				<Item Name="compatFileDialog.vi" Type="VI" URL="/&lt;vilib&gt;/_oldvers/_oldvers.llb/compatFileDialog.vi"/>
				<Item Name="compatOpenFileOperation.vi" Type="VI" URL="/&lt;vilib&gt;/_oldvers/_oldvers.llb/compatOpenFileOperation.vi"/>
				<Item Name="compatCalcOffset.vi" Type="VI" URL="/&lt;vilib&gt;/_oldvers/_oldvers.llb/compatCalcOffset.vi"/>
				<Item Name="Read Characters From File.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/file.llb/Read Characters From File.vi"/>
				<Item Name="DialogType.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/DialogType.ctl"/>
				<Item Name="Open File+.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/file.llb/Open File+.vi"/>
				<Item Name="Read File+ (string).vi" Type="VI" URL="/&lt;vilib&gt;/Utility/file.llb/Read File+ (string).vi"/>
				<Item Name="compatReadText.vi" Type="VI" URL="/&lt;vilib&gt;/_oldvers/_oldvers.llb/compatReadText.vi"/>
				<Item Name="Close File+.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/file.llb/Close File+.vi"/>
				<Item Name="Find First Error.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Find First Error.vi"/>
				<Item Name="General Error Handler.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/General Error Handler.vi"/>
				<Item Name="DialogTypeEnum.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/DialogTypeEnum.ctl"/>
				<Item Name="General Error Handler Core CORE.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/General Error Handler Core CORE.vi"/>
				<Item Name="whitespace.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/whitespace.ctl"/>
				<Item Name="Check Special Tags.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Check Special Tags.vi"/>
				<Item Name="TagReturnType.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/TagReturnType.ctl"/>
				<Item Name="Set String Value.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Set String Value.vi"/>
				<Item Name="GetRTHostConnectedProp.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/GetRTHostConnectedProp.vi"/>
				<Item Name="Error Code Database.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Error Code Database.vi"/>
				<Item Name="Trim Whitespace.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Trim Whitespace.vi"/>
				<Item Name="Format Message String.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Format Message String.vi"/>
				<Item Name="Find Tag.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Find Tag.vi"/>
				<Item Name="Search and Replace Pattern.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Search and Replace Pattern.vi"/>
				<Item Name="Set Bold Text.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Set Bold Text.vi"/>
				<Item Name="Details Display Dialog.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Details Display Dialog.vi"/>
				<Item Name="ErrWarn.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/ErrWarn.ctl"/>
				<Item Name="eventvkey.ctl" Type="VI" URL="/&lt;vilib&gt;/event_ctls.llb/eventvkey.ctl"/>
				<Item Name="Clear Errors.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Clear Errors.vi"/>
				<Item Name="Not Found Dialog.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Not Found Dialog.vi"/>
				<Item Name="Three Button Dialog.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Three Button Dialog.vi"/>
				<Item Name="Three Button Dialog CORE.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Three Button Dialog CORE.vi"/>
				<Item Name="LVRectTypeDef.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/miscctls.llb/LVRectTypeDef.ctl"/>
				<Item Name="Longest Line Length in Pixels.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Longest Line Length in Pixels.vi"/>
				<Item Name="Convert property node font to graphics font.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Convert property node font to graphics font.vi"/>
				<Item Name="Get Text Rect.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Get Text Rect.vi"/>
				<Item Name="Get String Text Bounds.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Get String Text Bounds.vi"/>
				<Item Name="LVBoundsTypeDef.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/miscctls.llb/LVBoundsTypeDef.ctl"/>
				<Item Name="BuildHelpPath.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/BuildHelpPath.vi"/>
				<Item Name="GetHelpDir.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/GetHelpDir.vi"/>
				<Item Name="LVSceneTextAlignment.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/miscctls.llb/LVSceneTextAlignment.ctl"/>
				<Item Name="NI_Gmath.lvlib" Type="Library" URL="/&lt;vilib&gt;/gmath/NI_Gmath.lvlib"/>
				<Item Name="Is Value Changed.vim" Type="VI" URL="/&lt;vilib&gt;/Utility/Is Value Changed.vim"/>
				<Item Name="NI_Robotics_Analytical Serial Arm Class.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/Generic/NI_Robotics_Analytical Serial Arm Class.lvclass"/>
				<Item Name="NI_Robotics_5R Type 1 Serial Arm Class.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/5R/Type 1/NI_Robotics_5R Type 1 Serial Arm Class.lvclass"/>
				<Item Name="NI_AAL_Angle.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/NI_AAL_Angle.lvlib"/>
				<Item Name="NI_Robotics_6R Type 2 Serial Arm Class.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/6R/Type 2/NI_Robotics_6R Type 2 Serial Arm Class.lvclass"/>
				<Item Name="NI_Robotics_6R Type 3 Serial Arm Class.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/6R/Type 3/NI_Robotics_6R Type 3 Serial Arm Class.lvclass"/>
				<Item Name="NI_Robotics_AnalyticKinematics.lvlib" Type="Library" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/NI_Robotics_AnalyticKinematics.lvlib"/>
				<Item Name="NI_Robotics_SCARA Serial Arm Class.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/SCARA/NI_Robotics_SCARA Serial Arm Class.lvclass"/>
				<Item Name="NI_Robotics_6R Type 1 Serial Arm Class.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/6R/Type 1/NI_Robotics_6R Type 1 Serial Arm Class.lvclass"/>
				<Item Name="WPI_DriverStationRobotMode2.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationRobotMode2.ctl"/>
				<Item Name="WPI_DriverStationAllianceInfo.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationAllianceInfo.ctl"/>
			</Item>
			<Item Name="lvanlys.dll" Type="Document" URL="/&lt;resource&gt;/lvanlys.dll"/>
			<Item Name="ArmInverseKinematics.vi" Type="VI" URL="../ArmInverseKinematics.vi"/>
			<Item Name="GlobalMiscellaneous.vi" Type="VI" URL="../GlobalMiscellaneous.vi"/>
			<Item Name="GlobalDrivebase.vi" Type="VI" URL="../GlobalDrivebase.vi"/>
			<Item Name="GlobalArm.vi" Type="VI" URL="../GlobalArm.vi"/>
			<Item Name="SetRobotProfile.vi" Type="VI" URL="../SetRobotProfile.vi"/>
			<Item Name="InitArmKinematics.vi" Type="VI" URL="../InitArmKinematics.vi"/>
			<Item Name="ArmGoalJointAngleCalculations.vi" Type="VI" URL="../ArmGoalJointAngleCalculations.vi"/>
			<Item Name="GlobalGripper.vi" Type="VI" URL="../GlobalGripper.vi"/>
			<Item Name="ArmFeedForwardCalculations.vi" Type="VI" URL="../ArmFeedForwardCalculations.vi"/>
			<Item Name="GlobalDriverStationData.vi" Type="VI" URL="../GlobalDriverStationData.vi"/>
			<Item Name="GlobalPoseAndTargeting.vi" Type="VI" URL="../GlobalPoseAndTargeting.vi"/>
		</Item>
		<Item Name="Build Specifications" Type="Build"/>
	</Item>
	<Item Name="Target" Type="RT roboRIO">
		<Property Name="alias.name" Type="Str">Target</Property>
		<Property Name="alias.value" Type="Str">10.42.65.2</Property>
		<Property Name="CCSymbols" Type="Str">OS,Linux;CPU,ARM;DeviceCode,7AAE;TARGET_TYPE,RT;</Property>
		<Property Name="crio.ControllerPID" Type="Str">7AAE</Property>
		<Property Name="crio.family" Type="Str">ARMLinux</Property>
		<Property Name="host.ResponsivenessCheckEnabled" Type="Bool">true</Property>
		<Property Name="host.ResponsivenessCheckPingDelay" Type="UInt">5000</Property>
		<Property Name="host.ResponsivenessCheckPingTimeout" Type="UInt">1000</Property>
		<Property Name="host.TargetCPUID" Type="UInt">8</Property>
		<Property Name="host.TargetOSID" Type="UInt">8</Property>
		<Property Name="target.cleanupVisa" Type="Bool">false</Property>
		<Property Name="target.DoNotReboot" Type="Bool">true</Property>
		<Property Name="target.FPProtocolGlobals_ControlTimeLimit" Type="Int">300</Property>
		<Property Name="target.getDefault-&gt;WebServer.Port" Type="Int">80</Property>
		<Property Name="target.getDefault-&gt;WebServer.Timeout" Type="Int">60</Property>
		<Property Name="target.IOScan.Faults" Type="Str"></Property>
		<Property Name="target.IOScan.NetVarPeriod" Type="UInt">100</Property>
		<Property Name="target.IOScan.NetWatchdogEnabled" Type="Bool">false</Property>
		<Property Name="target.IOScan.Period" Type="UInt">10000</Property>
		<Property Name="target.IOScan.PowerupMode" Type="UInt">0</Property>
		<Property Name="target.IOScan.Priority" Type="UInt">0</Property>
		<Property Name="target.IOScan.ReportModeConflict" Type="Bool">true</Property>
		<Property Name="target.IsRemotePanelSupported" Type="Bool">true</Property>
		<Property Name="target.RTCPULoadMonitoringEnabled" Type="Bool">true</Property>
		<Property Name="target.RTDebugWebServerHTTPPort" Type="Int">8001</Property>
		<Property Name="target.RTTarget.ApplicationPath" Type="Path">/home/lvuser/natinst/bin/startup.rtexe</Property>
		<Property Name="target.RTTarget.EnableFileSharing" Type="Bool">true</Property>
		<Property Name="target.RTTarget.IPAccess" Type="Str">+*</Property>
		<Property Name="target.RTTarget.LaunchAppAtBoot" Type="Bool">true</Property>
		<Property Name="target.RTTarget.VIPath" Type="Path">/home/lvuser/natinst/bin</Property>
		<Property Name="target.server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="target.server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="target.server.tcp.access" Type="Str">+*</Property>
		<Property Name="target.server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="target.server.tcp.paranoid" Type="Bool">true</Property>
		<Property Name="target.server.tcp.port" Type="Int">3363</Property>
		<Property Name="target.server.tcp.serviceName" Type="Str">Main Application Instance/VI Server</Property>
		<Property Name="target.server.tcp.serviceName.default" Type="Str">Main Application Instance/VI Server</Property>
		<Property Name="target.server.vi.access" Type="Str">+*</Property>
		<Property Name="target.server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="target.server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="target.WebServer.Config" Type="Str">Listen 8000

NI.ServerName default
DocumentRoot "$LVSERVER_DOCROOT"
TypesConfig "$LVSERVER_CONFIGROOT/mime.types"
DirectoryIndex index.htm
WorkerLimit 10
InactivityTimeout 60

LoadModulePath "$LVSERVER_MODULEPATHS"
LoadModule LVAuth lvauthmodule
LoadModule LVRFP lvrfpmodule

#
# Pipeline Definition
#

SetConnector netConnector

AddHandler LVAuth
AddHandler LVRFP

AddHandler fileHandler ""

AddOutputFilter chunkFilter


</Property>
		<Property Name="target.WebServer.Enabled" Type="Bool">false</Property>
		<Property Name="target.WebServer.LogEnabled" Type="Bool">false</Property>
		<Property Name="target.WebServer.LogPath" Type="Path">/c/ni-rt/system/www/www.log</Property>
		<Property Name="target.WebServer.Port" Type="Int">80</Property>
		<Property Name="target.WebServer.RootPath" Type="Path">/c/ni-rt/system/www</Property>
		<Property Name="target.WebServer.TcpAccess" Type="Str">c+*</Property>
		<Property Name="target.WebServer.Timeout" Type="Int">60</Property>
		<Property Name="target.WebServer.ViAccess" Type="Str">+*</Property>
		<Property Name="target.webservices.SecurityAPIKey" Type="Str">PqVr/ifkAQh+lVrdPIykXlFvg12GhhQFR8H9cUhphgg=:pTe9HRlQuMfJxAG6QCGq7UvoUpJzAzWGKy5SbZ+roSU=</Property>
		<Property Name="target.webservices.ValidTimestampWindow" Type="Int">15</Property>
		<Item Name="Autonomous" Type="Folder">
			<Item Name="Commands" Type="Folder">
				<Item Name="DriveAngleH.vi" Type="VI" URL="../DriveAngleH.vi"/>
				<Item Name="DriveBalance.vi" Type="VI" URL="../DriveBalance.vi"/>
				<Item Name="DriveDistanceX.vi" Type="VI" URL="../DriveDistanceX.vi"/>
				<Item Name="DriveDistanceY.vi" Type="VI" URL="../DriveDistanceY.vi"/>
				<Item Name="DriveEnd.vi" Type="VI" URL="../DriveEnd.vi"/>
				<Item Name="DriveOver.vi" Type="VI" URL="../DriveOver.vi"/>
				<Item Name="DrivePath.vi" Type="VI" URL="../DrivePath.vi"/>
				<Item Name="DriveSetRobotPosition.vi" Type="VI" URL="../DriveSetRobotPosition.vi"/>
				<Item Name="DriveWaitForButton.vi" Type="VI" URL="../DriveWaitForButton.vi"/>
				<Item Name="DriveWaitForManip.vi" Type="VI" URL="../DriveWaitForManip.vi"/>
				<Item Name="DriveWaitForTime.vi" Type="VI" URL="../DriveWaitForTime.vi"/>
				<Item Name="ManipChangeScoringPosition.vi" Type="VI" URL="../ManipChangeScoringPosition.vi"/>
				<Item Name="ManipChangeState.vi" Type="VI" URL="../ManipChangeState.vi"/>
				<Item Name="ManipDropPiece.vi" Type="VI" URL="../ManipDropPiece.vi"/>
				<Item Name="ManipEnd.vi" Type="VI" URL="../ManipEnd.vi"/>
				<Item Name="ManipHumanPlayerSignal.vi" Type="VI" URL="../ManipHumanPlayerSignal.vi"/>
				<Item Name="ManipWaitForArm.vi" Type="VI" URL="../ManipWaitForArm.vi"/>
				<Item Name="ManipWaitForButton.vi" Type="VI" URL="../ManipWaitForButton.vi"/>
				<Item Name="ManipWaitForDistance.vi" Type="VI" URL="../ManipWaitForDistance.vi"/>
				<Item Name="ManipWaitForDrive.vi" Type="VI" URL="../ManipWaitForDrive.vi"/>
				<Item Name="ManipWaitForTime.vi" Type="VI" URL="../ManipWaitForTime.vi"/>
			</Item>
			<Item Name="Path Following" Type="Folder">
				<Item Name="AutoFeedForwardConversion.vi" Type="VI" URL="../AutoFeedForwardConversion.vi"/>
				<Item Name="PathSearch.vi" Type="VI" URL="../PathSearch.vi"/>
			</Item>
			<Item Name="AutoInit.vi" Type="VI" URL="../AutoInit.vi"/>
			<Item Name="AutoParameterStringToArray.vi" Type="VI" URL="../AutoParameterStringToArray.vi"/>
			<Item Name="AutoPlayLoad.vi" Type="VI" URL="../AutoPlayLoad.vi"/>
			<Item Name="AutoPlayParse.vi" Type="VI" URL="../AutoPlayParse.vi"/>
			<Item Name="AutoSequencer.vi" Type="VI" URL="../AutoSequencer.vi"/>
			<Item Name="GlobalAutonomous.vi" Type="VI" URL="../GlobalAutonomous.vi"/>
		</Item>
		<Item Name="Communications" Type="Folder">
			<Item Name="DriverStationGetCommands.vi" Type="VI" URL="../DriverStationGetCommands.vi"/>
			<Item Name="DriverStationGetData.vi" Type="VI" URL="../DriverStationGetData.vi"/>
			<Item Name="DriverStationSendData.vi" Type="VI" URL="../DriverStationSendData.vi"/>
			<Item Name="GlobalDriverStationCommands.vi" Type="VI" URL="../GlobalDriverStationCommands.vi"/>
			<Item Name="GlobalDriverStationData.vi" Type="VI" URL="../GlobalDriverStationData.vi"/>
			<Item Name="InitDriverControl.vi" Type="VI" URL="../InitDriverControl.vi"/>
		</Item>
		<Item Name="FIRST" Type="Folder">
			<Item Name="Autonomous Independent.vi" Type="VI" URL="../Autonomous Independent.vi"/>
			<Item Name="Test.vi" Type="VI" URL="../Test.vi"/>
		</Item>
		<Item Name="Initialization" Type="Folder">
			<Item Name="Begin.vi" Type="VI" URL="../Begin.vi"/>
			<Item Name="Finish.vi" Type="VI" URL="../Finish.vi"/>
			<Item Name="SetGlobalVariables.vi" Type="VI" URL="../SetGlobalVariables.vi"/>
			<Item Name="SetRobotProfile.vi" Type="VI" URL="../SetRobotProfile.vi"/>
		</Item>
		<Item Name="Sensors" Type="Folder">
			<Item Name="Evergreen" Type="Folder">
				<Item Name="DetectPressureAnomalies.vi" Type="VI" URL="../DetectPressureAnomalies.vi"/>
				<Item Name="MeasureBaselineCurrents.vi" Type="VI" URL="../MeasureBaselineCurrents.vi"/>
			</Item>
			<Item Name="Pose Estimation" Type="Folder">
				<Item Name="Odometry" Type="Folder">
					<Item Name="CalculateRobotVelocityAcceleration.vi" Type="VI" URL="../CalculateRobotVelocityAcceleration.vi"/>
					<Item Name="ConvertFalconDriveEncoders.vi" Type="VI" URL="../ConvertFalconDriveEncoders.vi"/>
					<Item Name="GetShifterState.vi" Type="VI" URL="../GetShifterState.vi"/>
					<Item Name="GetSwerveAbsoluteAzimuth.vi" Type="VI" URL="../GetSwerveAbsoluteAzimuth.vi"/>
					<Item Name="GetSwerveEncoders.vi" Type="VI" URL="../GetSwerveEncoders.vi"/>
					<Item Name="GetSwerveRawAzimuth.vi" Type="VI" URL="../GetSwerveRawAzimuth.vi"/>
					<Item Name="SwerveModuleOdometry.vi" Type="VI" URL="../SwerveModuleOdometry.vi"/>
				</Item>
				<Item Name="EstimateTargetingInformation.vi" Type="VI" URL="../EstimateTargetingInformation.vi"/>
				<Item Name="FusePose.vi" Type="VI" URL="../FusePose.vi"/>
				<Item Name="GetAprilTags.vi" Type="VI" URL="../GetAprilTags.vi"/>
				<Item Name="GetPigeonIMU.vi" Type="VI" URL="../GetPigeonIMU.vi"/>
				<Item Name="GlobalPoseAndTargeting.vi" Type="VI" URL="../GlobalPoseAndTargeting.vi"/>
				<Item Name="ModePose.vi" Type="VI" URL="../ModePose.vi"/>
				<Item Name="PoseAndTargeting.vi" Type="VI" URL="../PoseAndTargeting.vi"/>
				<Item Name="VisionPose.vi" Type="VI" URL="../VisionPose.vi"/>
			</Item>
			<Item Name="GetHighPrioritySensors.vi" Type="VI" URL="../GetHighPrioritySensors.vi"/>
			<Item Name="GetLowPrioritySensors.vi" Type="VI" URL="../GetLowPrioritySensors.vi"/>
		</Item>
		<Item Name="Subsystems" Type="Folder">
			<Property Name="NI.SortType" Type="Int">0</Property>
			<Item Name="Arm" Type="Folder">
				<Item Name="Kinematics" Type="Folder">
					<Item Name="ArmEndEffectorPosition.vi" Type="VI" URL="../ArmEndEffectorPosition.vi"/>
					<Item Name="ArmFeedForwardCalculations.vi" Type="VI" URL="../ArmFeedForwardCalculations.vi"/>
					<Item Name="ArmGoalJointAngleCalculations.vi" Type="VI" URL="../ArmGoalJointAngleCalculations.vi"/>
					<Item Name="ArmInverseKinematics.vi" Type="VI" URL="../ArmInverseKinematics.vi"/>
					<Item Name="InitArmKinematics.vi" Type="VI" URL="../InitArmKinematics.vi"/>
				</Item>
				<Item Name="Arm Position Check.vi" Type="VI" URL="../Arm Position Check.vi"/>
				<Item Name="ArmSequencer.vi" Type="VI" URL="../ArmSequencer.vi"/>
				<Item Name="GlobalArm.vi" Type="VI" URL="../GlobalArm.vi"/>
				<Item Name="InitArm.vi" Type="VI" URL="../InitArm.vi"/>
				<Item Name="ModeArmScoring.vi" Type="VI" URL="../ModeArmScoring.vi"/>
				<Item Name="OutputArm.vi" Type="VI" URL="../OutputArm.vi"/>
				<Item Name="StateArm.vi" Type="VI" URL="../StateArm.vi"/>
			</Item>
			<Item Name="Climber" Type="Folder">
				<Item Name="GlobalClimber.vi" Type="VI" URL="../GlobalClimber.vi"/>
				<Item Name="InitClimber.vi" Type="VI" URL="../InitClimber.vi"/>
				<Item Name="OutputClimber.vi" Type="VI" URL="../OutputClimber.vi"/>
				<Item Name="StateClimber.vi" Type="VI" URL="../StateClimber.vi"/>
			</Item>
			<Item Name="Drivebase" Type="Folder">
				<Item Name="Swerve Toolbox" Type="Folder">
					<Item Name="FieldOrientedTransform.vi" Type="VI" URL="../FieldOrientedTransform.vi"/>
					<Item Name="SwerveAntiDrift.vi" Type="VI" URL="../SwerveAntiDrift.vi"/>
					<Item Name="SwerveChooseBestAngle.vi" Type="VI" URL="../SwerveChooseBestAngle.vi"/>
					<Item Name="SwerveChooseCenter.vi" Type="VI" URL="../SwerveChooseCenter.vi"/>
					<Item Name="SwerveJoystickRotationScaling.vi" Type="VI" URL="../SwerveJoystickRotationScaling.vi"/>
					<Item Name="SwerveJoystickStrafeScaling.vi" Type="VI" URL="../SwerveJoystickStrafeScaling.vi"/>
					<Item Name="SwerveKinematics.vi" Type="VI" URL="../SwerveKinematics.vi"/>
					<Item Name="SwerveLatchGoalAngle.vi" Type="VI" URL="../SwerveLatchGoalAngle.vi"/>
					<Item Name="SwerveModifyDriveCommand.vi" Type="VI" URL="../SwerveModifyDriveCommand.vi"/>
					<Item Name="SwerveUnwrapGoalAngle.vi" Type="VI" URL="../SwerveUnwrapGoalAngle.vi"/>
				</Item>
				<Item Name="ConfigureDriverProfiles.vi" Type="VI" URL="../ConfigureDriverProfiles.vi"/>
				<Item Name="DecideWhenToShift.vi" Type="VI" URL="../DecideWhenToShift.vi"/>
				<Item Name="DetermineSpeedCap.vi" Type="VI" URL="../DetermineSpeedCap.vi"/>
				<Item Name="DriveCommandToMotorPower.vi" Type="VI" URL="../DriveCommandToMotorPower.vi"/>
				<Item Name="GlobalDrivebase.vi" Type="VI" URL="../GlobalDrivebase.vi"/>
				<Item Name="InitDrivebase.vi" Type="VI" URL="../InitDrivebase.vi"/>
				<Item Name="ModeDrivebase.vi" Type="VI" URL="../ModeDrivebase.vi"/>
				<Item Name="ModeFieldOriented.vi" Type="VI" URL="../ModeFieldOriented.vi"/>
				<Item Name="OutputSwerveAzimuth.vi" Type="VI" URL="../OutputSwerveAzimuth.vi"/>
				<Item Name="OutputSwerveDrive.vi" Type="VI" URL="../OutputSwerveDrive.vi"/>
				<Item Name="StateDrivebase.vi" Type="VI" URL="../StateDrivebase.vi"/>
				<Item Name="StutterShifterState.vi" Type="VI" URL="../StutterShifterState.vi"/>
			</Item>
			<Item Name="Gripper" Type="Folder">
				<Item Name="GlobalGripper.vi" Type="VI" URL="../GlobalGripper.vi"/>
				<Item Name="InitGripper.vi" Type="VI" URL="../InitGripper.vi"/>
				<Item Name="OutputGripper.vi" Type="VI" URL="../OutputGripper.vi"/>
				<Item Name="StateGripper.vi" Type="VI" URL="../StateGripper.vi"/>
			</Item>
			<Item Name="Hopper" Type="Folder">
				<Item Name="GlobalHopper.vi" Type="VI" URL="../GlobalHopper.vi"/>
				<Item Name="InitHopper.vi" Type="VI" URL="../InitHopper.vi"/>
				<Item Name="OutputHopper.vi" Type="VI" URL="../OutputHopper.vi"/>
				<Item Name="StateHopper.vi" Type="VI" URL="../StateHopper.vi"/>
			</Item>
			<Item Name="Intake" Type="Folder">
				<Item Name="GlobalIntake.vi" Type="VI" URL="../GlobalIntake.vi"/>
				<Item Name="InitIntake.vi" Type="VI" URL="../InitIntake.vi"/>
				<Item Name="OutputIntake.vi" Type="VI" URL="../OutputIntake.vi"/>
				<Item Name="StateIntake.vi" Type="VI" URL="../StateIntake.vi"/>
			</Item>
			<Item Name="LEDs" Type="Folder">
				<Item Name="LEDDebugColorSensors.vi" Type="VI" URL="../LEDDebugColorSensors.vi"/>
				<Item Name="LEDDebugColorSwerve.vi" Type="VI" URL="../LEDDebugColorSwerve.vi"/>
				<Item Name="ModeLED.vi" Type="VI" URL="../ModeLED.vi"/>
				<Item Name="OutputLED.vi" Type="VI" URL="../OutputLED.vi"/>
				<Item Name="StateLED.vi" Type="VI" URL="../StateLED.vi"/>
			</Item>
			<Item Name="LimeLight" Type="Folder">
				<Item Name="OutputLimeLight.vi" Type="VI" URL="../OutputLimeLight.vi"/>
				<Item Name="StateLimeLight.vi" Type="VI" URL="../StateLimeLight.vi"/>
			</Item>
			<Item Name="Miscellaneous" Type="Folder">
				<Item Name="GlobalMiscellaneous.vi" Type="VI" URL="../GlobalMiscellaneous.vi"/>
				<Item Name="InitMiscellaneous.vi" Type="VI" URL="../InitMiscellaneous.vi"/>
				<Item Name="OutputCompressor.vi" Type="VI" URL="../OutputCompressor.vi"/>
				<Item Name="StateAlarmManagement.vi" Type="VI" URL="../StateAlarmManagement.vi"/>
				<Item Name="StateHapticEngine.vi" Type="VI" URL="../StateHapticEngine.vi"/>
				<Item Name="StatePowerManagement.vi" Type="VI" URL="../StatePowerManagement.vi"/>
			</Item>
			<Item Name="MasterStateController.vi" Type="VI" URL="../MasterStateController.vi"/>
		</Item>
		<Item Name="Utility" Type="Folder">
			<Property Name="NI.SortType" Type="Int">0</Property>
			<Item Name="Actuator Interlocks" Type="Folder">
				<Item Name="GenerateSinusoidalOutput.vi" Type="VI" URL="../GenerateSinusoidalOutput.vi"/>
				<Item Name="GenerateSquarewaveOutput.vi" Type="VI" URL="../GenerateSquarewaveOutput.vi"/>
				<Item Name="TAI_Motor.vi" Type="VI" URL="../TAI_Motor.vi"/>
				<Item Name="TAI_Servo.vi" Type="VI" URL="../TAI_Servo.vi"/>
				<Item Name="TAI_Solenoid.vi" Type="VI" URL="../TAI_Solenoid.vi"/>
				<Item Name="TestActuatorInterlock.vi" Type="VI" URL="../TestActuatorInterlock.vi"/>
			</Item>
			<Item Name="Actuator Wrappers" Type="Folder">
				<Item Name="DoubleSolenoidCommandState.vi" Type="VI" URL="../DoubleSolenoidCommandState.vi"/>
				<Item Name="PDHSwitchCommandState.vi" Type="VI" URL="../PDHSwitchCommandState.vi"/>
				<Item Name="TalonFXCommandPosition.vi" Type="VI" URL="../TalonFXCommandPosition.vi"/>
				<Item Name="TalonFXConfigMotionMagic.vi" Type="VI" URL="../TalonFXConfigMotionMagic.vi"/>
				<Item Name="TalonFXConfigPID.vi" Type="VI" URL="../TalonFXConfigPID.vi"/>
				<Item Name="TalonFXReadEncoder.vi" Type="VI" URL="../TalonFXReadEncoder.vi"/>
				<Item Name="TalonFXSetCoast.vi" Type="VI" URL="../TalonFXSetCoast.vi"/>
				<Item Name="TalonFXZeroEncoder.vi" Type="VI" URL="../TalonFXZeroEncoder.vi"/>
			</Item>
			<Item Name="Array Helpers" Type="Folder">
				<Item Name="BuildRollingArray.vim" Type="VI" URL="../BuildRollingArray.vim"/>
				<Item Name="SearchTimestampArray.vi" Type="VI" URL="../SearchTimestampArray.vi"/>
			</Item>
			<Item Name="Communication Helpers" Type="Folder">
				<Item Name="ReducedBWSendToNT.vim" Type="VI" URL="../ReducedBWSendToNT.vim"/>
				<Item Name="SendMessageToConsole.vi" Type="VI" URL="../SendMessageToConsole.vi"/>
			</Item>
			<Item Name="Control Algorithms" Type="Folder">
				<Item Name="CustomPID_v4.0.vi" Type="VI" URL="../CustomPID_v4.0.vi"/>
				<Item Name="CustomPID_v4.1.vi" Type="VI" URL="../CustomPID_v4.1.vi"/>
				<Item Name="LimitAcceleration.vi" Type="VI" URL="../LimitAcceleration.vi"/>
			</Item>
			<Item Name="Control Modifiers" Type="Folder">
				<Item Name="EdgeTrigger.vi" Type="VI" URL="../EdgeTrigger.vi"/>
				<Item Name="Latch.vim" Type="VI" URL="../Latch.vim"/>
				<Item Name="LongpressButton.vi" Type="VI" URL="../LongpressButton.vi"/>
				<Item Name="PulseOutput.vi" Type="VI" URL="../PulseOutput.vi"/>
				<Item Name="StickyButton.vi" Type="VI" URL="../StickyButton.vi"/>
				<Item Name="Toggle.vi" Type="VI" URL="../Toggle.vi"/>
			</Item>
			<Item Name="Error Helpers" Type="Folder">
				<Item Name="ErrorLatch.vi" Type="VI" URL="../ErrorLatch.vi"/>
				<Item Name="MergeErrorMessages.vi" Type="VI" URL="../MergeErrorMessages.vi"/>
			</Item>
			<Item Name="File Helpers" Type="Folder">
				<Item Name="BumpSetpoint.vi" Type="VI" URL="../BumpSetpoint.vi"/>
				<Item Name="ConfigureSetpoints.vi" Type="VI" URL="../ConfigureSetpoints.vi"/>
				<Item Name="ConfigureVariable.vi" Type="VI" URL="../ConfigureVariable.vi"/>
				<Item Name="ReadCSV.vi" Type="VI" URL="../ReadCSV.vi"/>
				<Item Name="ReadXML.vi" Type="VI" URL="../ReadXML.vi"/>
			</Item>
			<Item Name="Joysticks" Type="Folder">
				<Item Name="GenericJoystickCalibrate.vi" Type="VI" URL="../GenericJoystickCalibrate.vi"/>
				<Item Name="GenericJoystickCoerce.vi" Type="VI" URL="../GenericJoystickCoerce.vi"/>
				<Item Name="GenericJoystickDeadband.vi" Type="VI" URL="../GenericJoystickDeadband.vi"/>
			</Item>
			<Item Name="Parser" Type="Folder">
				<Item Name="SafeParser.vi" Type="VI" URL="../SafeParser.vi"/>
				<Item Name="SafeParser_Float.vi" Type="VI" URL="../SafeParser_Float.vi"/>
				<Item Name="SafeParser_Integer.vi" Type="VI" URL="../SafeParser_Integer.vi"/>
				<Item Name="SafeParser_String.vi" Type="VI" URL="../SafeParser_String.vi"/>
			</Item>
			<Item Name="Ref Num Helpers" Type="Folder">
				<Property Name="NI.SortType" Type="Int">0</Property>
				<Item Name="GetRefNumFast.vi" Type="VI" URL="../GetRefNumFast.vi"/>
				<Item Name="GRNF_AI.vi" Type="VI" URL="../GRNF_AI.vi"/>
				<Item Name="GRNF_ColorSensor.vi" Type="VI" URL="../GRNF_ColorSensor.vi"/>
				<Item Name="GRNF_Compressor.vi" Type="VI" URL="../GRNF_Compressor.vi"/>
				<Item Name="GRNF_CTREMotor.vi" Type="VI" URL="../GRNF_CTREMotor.vi"/>
				<Item Name="GRNF_DI.vi" Type="VI" URL="../GRNF_DI.vi"/>
				<Item Name="GRNF_DIC.vi" Type="VI" URL="../GRNF_DIC.vi"/>
				<Item Name="GRNF_GenericMotor.vi" Type="VI" URL="../GRNF_GenericMotor.vi"/>
				<Item Name="GRNF_GenericSolenoid.vi" Type="VI" URL="../GRNF_GenericSolenoid.vi"/>
				<Item Name="GRNF_Joystick.vi" Type="VI" URL="../GRNF_Joystick.vi"/>
				<Item Name="GRNF_POT.vi" Type="VI" URL="../GRNF_POT.vi"/>
				<Item Name="GRNF_Servo.vi" Type="VI" URL="../GRNF_Servo.vi"/>
			</Item>
			<Item Name="Sensor Helpers" Type="Folder">
				<Item Name="AngularDistance.vi" Type="VI" URL="../AngularDistance.vi"/>
				<Item Name="ConvertPotentiometerToDegrees.vi" Type="VI" URL="../ConvertPotentiometerToDegrees.vi"/>
				<Item Name="InterpolateCalibrationValue.vi" Type="VI" URL="../InterpolateCalibrationValue.vi"/>
			</Item>
			<Item Name="Timing" Type="Folder">
				<Item Name="ElapsedTimes.vi" Type="VI" URL="../ElapsedTimes.vi"/>
				<Item Name="Timer.vi" Type="VI" URL="../Timer.vi"/>
				<Item Name="TimeThreshold.vi" Type="VI" URL="../TimeThreshold.vi"/>
			</Item>
			<Item Name="BlinkLight.vi" Type="VI" URL="../BlinkLight.vi"/>
			<Item Name="DesyncInitCAN.vi" Type="VI" URL="../DesyncInitCAN.vi"/>
			<Item Name="RGBtoGRB.vi" Type="VI" URL="../RGBtoGRB.vi"/>
			<Item Name="StateSelectionEnum.vim" Type="VI" URL="../StateSelectionEnum.vim"/>
			<Item Name="TriggerUpdate.vim" Type="VI" URL="../TriggerUpdate.vim"/>
		</Item>
		<Item Name="MasterRobotModeController.vi" Type="VI" URL="../MasterRobotModeController.vi"/>
		<Item Name="Periodic Tasks.vi" Type="VI" URL="../Periodic Tasks.vi"/>
		<Item Name="Robot Main.vi" Type="VI" URL="../Robot Main.vi"/>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="vi.lib" Type="Folder">
				<Item Name="Acquire Semaphore.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/semaphor.llb/Acquire Semaphore.vi"/>
				<Item Name="AddNamedSemaphorePrefix.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/semaphor.llb/AddNamedSemaphorePrefix.vi"/>
				<Item Name="AI Accum proto.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/AI/AI Accum proto.ctl"/>
				<Item Name="AI Data proto.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/AI/AI Data proto.ctl"/>
				<Item Name="AI proto.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/AI/AI proto.ctl"/>
				<Item Name="AI Update Ops.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/AI/AI Update Ops.ctl"/>
				<Item Name="AnalogInput_PanelSim_Global.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/AI/AnalogInput_PanelSim_Global.vi"/>
				<Item Name="Clear Errors.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Clear Errors.vi"/>
				<Item Name="Close File+.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/file.llb/Close File+.vi"/>
				<Item Name="Color Sensor V3 Add Color Match.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Public/Color Sensor V3 Add Color Match.vi"/>
				<Item Name="Color Sensor V3 Check Device ID.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Private/Color Sensor V3 Check Device ID.vi"/>
				<Item Name="Color Sensor V3 Close.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Public/Color Sensor V3 Close.vi"/>
				<Item Name="Color Sensor V3 Color Match FGV.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Private/Color Sensor V3 Color Match FGV.vi"/>
				<Item Name="Color Sensor V3 Color Match.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/Typedef/Color Sensor V3 Color Match.ctl"/>
				<Item Name="Color Sensor V3 Config Color Sensor.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Public/Color Sensor V3 Config Color Sensor.vi"/>
				<Item Name="Color Sensor V3 Config Prox Sensor.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Public/Color Sensor V3 Config Prox Sensor.vi"/>
				<Item Name="Color Sensor V3 Config ProxSensor LED.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Public/Color Sensor V3 Config ProxSensor LED.vi"/>
				<Item Name="Color Sensor V3 DevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/Typedef/Color Sensor V3 DevRef.ctl"/>
				<Item Name="Color Sensor V3 FGV Command.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/Typedef/Color Sensor V3 FGV Command.ctl"/>
				<Item Name="Color Sensor V3 GainFactor.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/Typedef/Color Sensor V3 GainFactor.ctl"/>
				<Item Name="Color Sensor V3 Get Color.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Public/Color Sensor V3 Get Color.vi"/>
				<Item Name="Color Sensor V3 Get Proximity.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Public/Color Sensor V3 Get Proximity.vi"/>
				<Item Name="Color Sensor V3 Initialize.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Private/Color Sensor V3 Initialize.vi"/>
				<Item Name="Color Sensor V3 LEDCurrent.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/Typedef/Color Sensor V3 LEDCurrent.ctl"/>
				<Item Name="Color Sensor V3 LEDPulseFreq.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/Typedef/Color Sensor V3 LEDPulseFreq.ctl"/>
				<Item Name="Color Sensor V3 Match Color.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Public/Color Sensor V3 Match Color.vi"/>
				<Item Name="Color Sensor V3 MeasurementRate.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/Typedef/Color Sensor V3 MeasurementRate.ctl"/>
				<Item Name="Color Sensor V3 Open.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Public/Color Sensor V3 Open.vi"/>
				<Item Name="Color Sensor V3 ProximityRate.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/Typedef/Color Sensor V3 ProximityRate.ctl"/>
				<Item Name="Color Sensor V3 ProxResolution.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/Typedef/Color Sensor V3 ProxResolution.ctl"/>
				<Item Name="Color Sensor V3 Read8.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Private/Color Sensor V3 Read8.vi"/>
				<Item Name="Color Sensor V3 RefNum Registry Get.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Public/Color Sensor V3 RefNum Registry Get.vi"/>
				<Item Name="Color Sensor V3 RefNum Registry Set.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Public/Color Sensor V3 RefNum Registry Set.vi"/>
				<Item Name="Color Sensor V3 Registers.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/Typedef/Color Sensor V3 Registers.ctl"/>
				<Item Name="Color Sensor V3 Resolution.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/Typedef/Color Sensor V3 Resolution.ctl"/>
				<Item Name="Color Sensor V3 To 20 Bit.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Private/Color Sensor V3 To 20 Bit.vi"/>
				<Item Name="Color Sensor V3 Write8.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/REV Robotics/REVLib/SubVI/Private/Color Sensor V3 Write8.vi"/>
				<Item Name="Color to RGB.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/colorconv.llb/Color to RGB.vi"/>
				<Item Name="compatCalcOffset.vi" Type="VI" URL="/&lt;vilib&gt;/_oldvers/_oldvers.llb/compatCalcOffset.vi"/>
				<Item Name="compatFileDialog.vi" Type="VI" URL="/&lt;vilib&gt;/_oldvers/_oldvers.llb/compatFileDialog.vi"/>
				<Item Name="compatOpenFileOperation.vi" Type="VI" URL="/&lt;vilib&gt;/_oldvers/_oldvers.llb/compatOpenFileOperation.vi"/>
				<Item Name="compatReadText.vi" Type="VI" URL="/&lt;vilib&gt;/_oldvers/_oldvers.llb/compatReadText.vi"/>
				<Item Name="CTRE_ErrorHandle.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Error/CTRE_ErrorHandle.vi"/>
				<Item Name="CTRE_LibraryCall_ErrorHandle.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Error/CTRE_LibraryCall_ErrorHandle.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_ConfigClosedLoopConstants.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Closed Loop/CTRE_Phoenix_MotorControl_ConfigClosedLoopConstants.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_ConfigClosedLoopRamp.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Output Config/CTRE_Phoenix_MotorControl_ConfigClosedLoopRamp.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_ConfigMotionMagic.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Closed Loop/CTRE_Phoenix_MotorControl_ConfigMotionMagic.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_ConfigNeutralDeadband.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Output Config/CTRE_Phoenix_MotorControl_ConfigNeutralDeadband.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_ConfigOpenLoopRamp.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Output Config/CTRE_Phoenix_MotorControl_ConfigOpenLoopRamp.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_ConfigSelectedFeedbackSensor_TalonFX.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Sensor/CTRE_Phoenix_MotorControl_ConfigSelectedFeedbackSensor_TalonFX.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_ConfigSetParameter.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Parameter/CTRE_Phoenix_MotorControl_ConfigSetParameter.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_ConfigSoftLimitEnables.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Limit Switch/CTRE_Phoenix_MotorControl_ConfigSoftLimitEnables.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_ConfigSoftLimitThresholds.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Limit Switch/CTRE_Phoenix_MotorControl_ConfigSoftLimitThresholds.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_ConfigStatorCurrentLimit.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Current Limit/CTRE_Phoenix_MotorControl_ConfigStatorCurrentLimit.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_ConfigSupplyCurrentLimit.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Current Limit/CTRE_Phoenix_MotorControl_ConfigSupplyCurrentLimit.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_ConfigVoltageCompensation.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Output Config/CTRE_Phoenix_MotorControl_ConfigVoltageCompensation.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_ControlMode.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_ControlMode.ctl"/>
				<Item Name="CTRE_Phoenix_MotorControl_DevRefData.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_DevRefData.ctl"/>
				<Item Name="CTRE_Phoenix_MotorControl_EnableVoltageCompensation.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Output Config/CTRE_Phoenix_MotorControl_EnableVoltageCompensation.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_FactoryDefault.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_FactoryDefault.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_FeedbackDevice_TalonFX.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_FeedbackDevice_TalonFX.ctl"/>
				<Item Name="CTRE_Phoenix_MotorControl_GetDevRefData.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_GetDevRefData.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_NeutralMode.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_NeutralMode.ctl"/>
				<Item Name="CTRE_Phoenix_MotorControl_OverrideLimitSwitchesEnable.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Limit Switch/CTRE_Phoenix_MotorControl_OverrideLimitSwitchesEnable.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_RefNumRegistryGet.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_RefNumRegistryGet.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_RefNumRegistrySet.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_RefNumRegistrySet.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_SelectProfile Slot.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Closed Loop/CTRE_Phoenix_MotorControl_SelectProfile Slot.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_Set.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_Set.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_SetInverted_TalonFX.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_SetInverted_TalonFX.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_SetNeutralMode.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_SetNeutralMode.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_SetSelectedSensorPosition.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Sensor/CTRE_Phoenix_MotorControl_SetSelectedSensorPosition.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_SetSensorPhase.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Sensor/CTRE_Phoenix_MotorControl_SetSensorPhase.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_SetStatusFramePeriod_Enhanced.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Frame Rate/CTRE_Phoenix_MotorControl_SetStatusFramePeriod_Enhanced.vi"/>
				<Item Name="CTRE_Phoenix_MotorControl_StatorCurrentLimitConfiguration.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_StatorCurrentLimitConfiguration.ctl"/>
				<Item Name="CTRE_Phoenix_MotorControl_StatusFrameEnhanced.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_StatusFrameEnhanced.ctl"/>
				<Item Name="CTRE_Phoenix_MotorControl_SupplyCurrentLimitConfiguration.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_SupplyCurrentLimitConfiguration.ctl"/>
				<Item Name="CTRE_Phoenix_MotorControl_TalonFXInvertType.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_TalonFXInvertType.ctl"/>
				<Item Name="CTRE_Phoenix_MotorControl_UpdateRefNum.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_UpdateRefNum.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_ClearStickyFaults.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Errors and Faults/CTRE_Phoenix_MotorController_ClearStickyFaults.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_Get_TalonFX.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorController_Get_TalonFX.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetBusVoltage.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetBusVoltage.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetClosedLoopData.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetClosedLoopData.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetClosedLoopError.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetClosedLoopError.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetClosedLoopTarget.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetClosedLoopTarget.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetDeviceID.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetDeviceID.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetErrorDerivative.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetErrorDerivative.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetFirmwareVersion.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetFirmwareVersion.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetGeneralStatus.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetGeneralStatus.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetHasResetOccurred.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetHasResetOccurred.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetIntegralAccumulator.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetIntegralAccumulator.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetLimitSwitchStates.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetLimitSwitchStates.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetPercentOutput.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetPercentOutput.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetSelectedSensorData.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetSelectedSensorData.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetSelectedSensorPosition.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetSelectedSensorPosition.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetSelectedSensorVelocity.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetSelectedSensorVelocity.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetSupplyAndStatorCurrent_Enhanced.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetSupplyAndStatorCurrent_Enhanced.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetTemperature.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetTemperature.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_GetVoltage.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetVoltage.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_IntegratedSensorData_TalonFX.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_IntegratedSensorData_TalonFX.vi"/>
				<Item Name="CTRE_Phoenix_MotorController_SetParameter.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_SetParameter.vi"/>
				<Item Name="CTRE_Phoenix_Params.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Controls/CTRE_Phoenix_Params.ctl"/>
				<Item Name="CTRE_Phoenix_Pigeon2_Faults.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon2/controls/CTRE_Phoenix_Pigeon2_Faults.ctl"/>
				<Item Name="CTRE_Phoenix_Pigeon2_GetFaults.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon2/CTRE_Phoenix_Pigeon2_GetFaults.vi"/>
				<Item Name="CTRE_Phoenix_Pigeon2_GetGravityVector.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon2/CTRE_Phoenix_Pigeon2_GetGravityVector.vi"/>
				<Item Name="CTRE_Phoenix_Pigeon2_GetStatus.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon2/CTRE_Phoenix_Pigeon2_GetStatus.vi"/>
				<Item Name="CTRE_Phoenix_Pigeon2_GetStickyFaults.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon2/CTRE_Phoenix_Pigeon2_GetStickyFaults.vi"/>
				<Item Name="CTRE_Phoenix_Pigeon2_Open.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon2/CTRE_Phoenix_Pigeon2_Open.vi"/>
				<Item Name="CTRE_Phoenix_TalonFX_Open.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Talon FX/CTRE_Phoenix_TalonFX_Open.vi"/>
				<Item Name="CTRE_PigeonIMU_CalibrationMode.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_CalibrationMode.ctl"/>
				<Item Name="CTRE_PigeonIMU_ConnectionType.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_ConnectionType.ctl"/>
				<Item Name="CTRE_PigeonIMU_DeviceRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_DeviceRef.ctl"/>
				<Item Name="CTRE_PigeonIMU_Get6DQuaternion.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_Get6DQuaternion.vi"/>
				<Item Name="CTRE_PigeonIMU_GetAccelerometerAngles.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_GetAccelerometerAngles.vi"/>
				<Item Name="CTRE_PigeonIMU_GetAccumGyro.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_GetAccumGyro.vi"/>
				<Item Name="CTRE_PigeonIMU_GetBiasedAccelerometer.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_GetBiasedAccelerometer.vi"/>
				<Item Name="CTRE_PigeonIMU_GetBiasedMagnetometer.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_GetBiasedMagnetometer.vi"/>
				<Item Name="CTRE_PigeonIMU_GetCompass.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_GetCompass.vi"/>
				<Item Name="CTRE_PigeonIMU_GetData.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_GetData.vi"/>
				<Item Name="CTRE_PigeonIMU_GetFusedHeading.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_GetFusedHeading.vi"/>
				<Item Name="CTRE_PigeonIMU_GetRawGyro.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_GetRawGyro.vi"/>
				<Item Name="CTRE_PigeonIMU_GetRawMagnetometer.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_GetRawMagnetometer.vi"/>
				<Item Name="CTRE_PigeonIMU_GetStartupStatus.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_GetStartupStatus.vi"/>
				<Item Name="CTRE_PigeonIMU_GetTempStateUpTime.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_GetTempStateUpTime.vi"/>
				<Item Name="CTRE_PigeonIMU_GetYPR.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_GetYPR.vi"/>
				<Item Name="CTRE_PigeonIMU_PigeonState.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_PigeonState.ctl"/>
				<Item Name="CTRE_PigeonIMU_RefNumRegistryGet.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_RefNumRegistryGet.vi"/>
				<Item Name="CTRE_PigeonIMU_RefNumRegistrySet.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Pigeon IMU/CTRE_PigeonIMU_RefNumRegistrySet.vi"/>
				<Item Name="DH Dynamic Parameters.ctl" Type="VI" URL="/&lt;vilib&gt;/robotics/Kinematics/DH Dynamic Parameters.ctl"/>
				<Item Name="DH Kinematic Parameters.ctl" Type="VI" URL="/&lt;vilib&gt;/robotics/Kinematics/DH Kinematic Parameters.ctl"/>
				<Item Name="DIO data proto.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/DIO/DIO data proto.ctl"/>
				<Item Name="DIO proto.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/DIO/DIO proto.ctl"/>
				<Item Name="DIO Update Ops.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/DIO/DIO Update Ops.ctl"/>
				<Item Name="DIO_PanelSim_Global.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/DIO/DIO_PanelSim_Global.vi"/>
				<Item Name="dPWM Data proto.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/dPWM/dPWM Data proto.ctl"/>
				<Item Name="dPWM proto.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/dPWM/dPWM proto.ctl"/>
				<Item Name="dPWM Update Ops.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/dPWM/dPWM Update Ops.ctl"/>
				<Item Name="dPWM_PanelSim_Global.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/dPWM/dPWM_PanelSim_Global.vi"/>
				<Item Name="DriverLib.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Controls/DriverLib.ctl"/>
				<Item Name="Error Cluster From Error Code.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Error Cluster From Error Code.vi"/>
				<Item Name="Find First Error.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Find First Error.vi"/>
				<Item Name="FindCloseTagByName.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/xml.llb/FindCloseTagByName.vi"/>
				<Item Name="FindElement.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/xml.llb/FindElement.vi"/>
				<Item Name="FindElementStartByName.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/xml.llb/FindElementStartByName.vi"/>
				<Item Name="FindEmptyElement.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/xml.llb/FindEmptyElement.vi"/>
				<Item Name="FindFirstTag.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/xml.llb/FindFirstTag.vi"/>
				<Item Name="FindMatchingCloseTag.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/xml.llb/FindMatchingCloseTag.vi"/>
				<Item Name="FPGA_AIAverageBits.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIAverageBits.vi"/>
				<Item Name="FPGA_AIChannel.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIChannel.ctl"/>
				<Item Name="FPGA_AIDevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIDevRef.ctl"/>
				<Item Name="FPGA_AIERRInvalidAccumulatorChannel.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIERRInvalidAccumulatorChannel.vi"/>
				<Item Name="FPGA_AIERRInvalidAnalogChannel.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIERRInvalidAnalogChannel.vi"/>
				<Item Name="FPGA_AIOpen.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIOpen.vi"/>
				<Item Name="FPGA_AIOversampleBits.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIOversampleBits.vi"/>
				<Item Name="FPGA_AIReadAveraged.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIReadAveraged.vi"/>
				<Item Name="FPGA_AIReadLSBWeight.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIReadLSBWeight.vi"/>
				<Item Name="FPGA_AIReadOversampleBits.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIReadOversampleBits.vi"/>
				<Item Name="FPGA_AIWriteAccumulatorReset.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIWriteAccumulatorReset.vi"/>
				<Item Name="FPGA_AIWriteAverageBits.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIWriteAverageBits.vi"/>
				<Item Name="FPGA_AIWriteConfig.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIWriteConfig.vi"/>
				<Item Name="FPGA_AIWriteOversampleBits.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIWriteOversampleBits.vi"/>
				<Item Name="FPGA_AIWriteScanList.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_AIWriteScanList.vi"/>
				<Item Name="FPGA_CounterCtrSystemIndex.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/Counter/FPGA_CounterCtrSystemIndex.ctl"/>
				<Item Name="FPGA_CounterERRInvalidResource.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/Counter/FPGA_CounterERRInvalidResource.vi"/>
				<Item Name="FPGA_DIOChannel.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOChannel.ctl"/>
				<Item Name="FPGA_DIODevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIODevRef.ctl"/>
				<Item Name="FPGA_DIOERRInvalidPWMChannel.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOERRInvalidPWMChannel.vi"/>
				<Item Name="FPGA_DIOOpen.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOOpen.vi"/>
				<Item Name="FPGA_DIOPWMChannel.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOPWMChannel.ctl"/>
				<Item Name="FPGA_DIOReadDI.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOReadDI.vi"/>
				<Item Name="FPGA_DIOReadDO_PWMDutyCycle.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOReadDO_PWMDutyCycle.vi"/>
				<Item Name="FPGA_DIOReadLoopTiming.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOReadLoopTiming.vi"/>
				<Item Name="FPGA_DIOReadMXPSpecialFunction.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOReadMXPSpecialFunction.vi"/>
				<Item Name="FPGA_DIOReadOutputEnable.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOReadOutputEnable.vi"/>
				<Item Name="FPGA_DIOReadPWMValue.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOReadPWMValue.vi"/>
				<Item Name="FPGA_DIOWriteDO_PWMConfig.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOWriteDO_PWMConfig.vi"/>
				<Item Name="FPGA_DIOWriteFilterSelect.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOWriteFilterSelect.vi"/>
				<Item Name="FPGA_DIOWriteMXPSpecialFunction.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOWriteMXPSpecialFunction.vi"/>
				<Item Name="FPGA_DIOWriteOutputEnable.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOWriteOutputEnable.vi"/>
				<Item Name="FPGA_DIOWritePWMMinHigh.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOWritePWMMinHigh.vi"/>
				<Item Name="FPGA_DIOWritePWMPeriod.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOWritePWMPeriod.vi"/>
				<Item Name="FPGA_DIOWritePWMPeriodScale.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOWritePWMPeriodScale.vi"/>
				<Item Name="FPGA_DIOWritePWMValue.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOWritePWMValue.vi"/>
				<Item Name="FPGA_DIOWriteSlowValueRelayFwd.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOWriteSlowValueRelayFwd.vi"/>
				<Item Name="FPGA_DIOWriteSlowValueRelayRev.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOWriteSlowValueRelayRev.vi"/>
				<Item Name="FPGA_LED_Configure.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/LED/FPGA_LED_Configure.vi"/>
				<Item Name="FPGA_LED_ERRInvalidPWMChannel.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/LED/FPGA_LED_ERRInvalidPWMChannel.vi"/>
				<Item Name="FPGA_LED_ERRMismatchedStringLength.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/LED/FPGA_LED_ERRMismatchedStringLength.vi"/>
				<Item Name="FPGA_LED_LoadString.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/LED/FPGA_LED_LoadString.vi"/>
				<Item Name="FPGA_LED_PackU32ArrayToU64Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/LED/FPGA_LED_PackU32ArrayToU64Array.vi"/>
				<Item Name="FPGA_LED_Start.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/LED/FPGA_LED_Start.vi"/>
				<Item Name="FPGA_MapDIOChannelEnumToModuleLine.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_MapDIOChannelEnumToModuleLine.vi"/>
				<Item Name="FPGA_NIFPGAInterfaceFPGAResourceConstant.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/FPGA_NIFPGAInterfaceFPGAResourceConstant.vi"/>
				<Item Name="FPGA_PowerStatus.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/Power/FPGA_PowerStatus.ctl"/>
				<Item Name="FPGA_ReadAI.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/AI/FPGA_ReadAI.vi"/>
				<Item Name="FPGA_SPI_Write_EnableDIO.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/SPI/FPGA_SPI_Write_EnableDIO.vi"/>
				<Item Name="FPGA_SystemAsync VI Agent.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/System/FPGA_SystemAsync VI Agent.vi"/>
				<Item Name="FPGA_SystemAsynch VI Registration.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/System/FPGA_SystemAsynch VI Registration.vi"/>
				<Item Name="FPGA_SystemERRWrongVersion.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/System/FPGA_SystemERRWrongVersion.vi"/>
				<Item Name="FPGA_SystemFPGA Ref Global.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/System/FPGA_SystemFPGA Ref Global.vi"/>
				<Item Name="FPGA_SystemFRC FPGA Ref.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/System/FPGA_SystemFRC FPGA Ref.ctl"/>
				<Item Name="FPGA_SystemGet.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/System/FPGA_SystemGet.vi"/>
				<Item Name="FPGA_SystemOpen.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/System/FPGA_SystemOpen.vi"/>
				<Item Name="FPGA_SystemStart Async Agent.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/System/FPGA_SystemStart Async Agent.vi"/>
				<Item Name="FPGA_UtilitiesRead LocalTime.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/Utilities/FPGA_UtilitiesRead LocalTime.vi"/>
				<Item Name="GetNamedSemaphorePrefix.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/semaphor.llb/GetNamedSemaphorePrefix.vi"/>
				<Item Name="GetSystemError.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/System/GetSystemError.vi"/>
				<Item Name="i2c_bus.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/i2clib/i2c_bus.ctl"/>
				<Item Name="i2clib_close.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/i2clib/i2clib_close.vi"/>
				<Item Name="i2clib_open.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/i2clib/i2clib_open.vi"/>
				<Item Name="i2clib_read.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/i2clib/i2clib_read.vi"/>
				<Item Name="i2clib_write.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/i2clib/i2clib_write.vi"/>
				<Item Name="i2clib_writeread.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/i2clib/i2clib_writeread.vi"/>
				<Item Name="Is Value Changed.vim" Type="VI" URL="/&lt;vilib&gt;/Utility/Is Value Changed.vim"/>
				<Item Name="Joystick data proto.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/Joystick/Joystick data proto.ctl"/>
				<Item Name="Joystick proto.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/Joystick/Joystick proto.ctl"/>
				<Item Name="Joystick Update Ops.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/Joystick/Joystick Update Ops.ctl"/>
				<Item Name="Joystick_PanelSim_Global.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/Joystick/Joystick_PanelSim_Global.vi"/>
				<Item Name="NetComm_AllianceStation.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_AllianceStation.ctl"/>
				<Item Name="NetComm_CAN_GetStatus.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_CAN_GetStatus.vi"/>
				<Item Name="NetComm_CAN_Receive.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_CAN_Receive.vi"/>
				<Item Name="NetComm_CAN_Send.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_CAN_Send.vi"/>
				<Item Name="NetComm_ControlWord.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_ControlWord.ctl"/>
				<Item Name="NetComm_getAllianceStation.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_getAllianceStation.vi"/>
				<Item Name="NetComm_getControlWord.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_getControlWord.vi"/>
				<Item Name="NetComm_getFPGAFileName.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_getFPGAFileName.vi"/>
				<Item Name="NetComm_getJoystickAxes.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_getJoystickAxes.vi"/>
				<Item Name="NetComm_getJoystickButtons.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_getJoystickButtons.vi"/>
				<Item Name="NetComm_getJoystickPOVs.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_getJoystickPOVs.vi"/>
				<Item Name="NetComm_getMatchInfo.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_getMatchInfo.vi"/>
				<Item Name="NetComm_getMatchTime.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_getMatchTime.vi"/>
				<Item Name="NetComm_MatchType.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_MatchType.ctl"/>
				<Item Name="NetComm_ObserveUserProgramStarting.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_ObserveUserProgramStarting.vi"/>
				<Item Name="NetComm_SendError.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_SendError.vi"/>
				<Item Name="NetComm_SendMessage.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_SendMessage.vi"/>
				<Item Name="NetComm_SetNewDataOccurrenceReference.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_SetNewDataOccurrenceReference.vi"/>
				<Item Name="NetComm_UnloadCPPStartupProgram.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_UnloadCPPStartupProgram.vi"/>
				<Item Name="NetComm_UsageReport_report.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_UsageReport_report.vi"/>
				<Item Name="NetComm_UsageReport_ResourceType.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_UsageReport_ResourceType.ctl"/>
				<Item Name="NI_AAL_Angle.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/NI_AAL_Angle.lvlib"/>
				<Item Name="NI_AALBase.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/NI_AALBase.lvlib"/>
				<Item Name="NI_AALPro.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/NI_AALPro.lvlib"/>
				<Item Name="NI_Gmath.lvlib" Type="Library" URL="/&lt;vilib&gt;/gmath/NI_Gmath.lvlib"/>
				<Item Name="NI_Kinematics.lvlib" Type="Library" URL="/&lt;vilib&gt;/robotics/Kinematics/NI_Kinematics.lvlib"/>
				<Item Name="NI_Matrix.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/Matrix/NI_Matrix.lvlib"/>
				<Item Name="NI_Robotics Utilities.lvlib" Type="Library" URL="/&lt;vilib&gt;/robotics/Utilities/NI_Robotics Utilities.lvlib"/>
				<Item Name="NI_Robotics_3DKinematics.lvlib" Type="Library" URL="/&lt;vilib&gt;/robotics/Kinematics/Plotting/NI_Robotics_3DKinematics.lvlib"/>
				<Item Name="NI_Robotics_5R Type 1 Serial Arm Class.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/5R/Type 1/NI_Robotics_5R Type 1 Serial Arm Class.lvclass"/>
				<Item Name="NI_Robotics_6R Type 1 Serial Arm Class.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/6R/Type 1/NI_Robotics_6R Type 1 Serial Arm Class.lvclass"/>
				<Item Name="NI_Robotics_6R Type 2 Serial Arm Class.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/6R/Type 2/NI_Robotics_6R Type 2 Serial Arm Class.lvclass"/>
				<Item Name="NI_Robotics_6R Type 3 Serial Arm Class.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/6R/Type 3/NI_Robotics_6R Type 3 Serial Arm Class.lvclass"/>
				<Item Name="NI_Robotics_Analytical Serial Arm Class.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/Generic/NI_Robotics_Analytical Serial Arm Class.lvclass"/>
				<Item Name="NI_Robotics_AnalyticKinematics.lvlib" Type="Library" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/NI_Robotics_AnalyticKinematics.lvlib"/>
				<Item Name="NI_Robotics_Link.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Link/NI_Robotics_Link.lvclass"/>
				<Item Name="NI_Robotics_SCARA Serial Arm Class.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Analytical/SCARA/NI_Robotics_SCARA Serial Arm Class.lvclass"/>
				<Item Name="NI_Robotics_Serial Robot Arm.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/robotics/Kinematics/Robot/NI_Robotics_Serial Robot Arm.lvclass"/>
				<Item Name="NI_Vision_Development_Module.lvlib" Type="Library" URL="/&lt;vilib&gt;/vision/NI_Vision_Development_Module.lvlib"/>
				<Item Name="Normalize End Of Line.vi" Type="VI" URL="/&lt;vilib&gt;/AdvancedString/Normalize End Of Line.vi"/>
				<Item Name="Not A Semaphore.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/semaphor.llb/Not A Semaphore.vi"/>
				<Item Name="NT Add NullSubscriber.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Add NullSubscriber.vi"/>
				<Item Name="NT Check Errors.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Check Errors.vi"/>
				<Item Name="NT Datatype.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Datatype.ctl"/>
				<Item Name="NT Get PublishHandle.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Get PublishHandle.vi"/>
				<Item Name="NT Get SubscribeHandle.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Get SubscribeHandle.vi"/>
				<Item Name="NT Get Topic.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Get Topic.vi"/>
				<Item Name="NT Globals.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Globals.vi"/>
				<Item Name="NT PublishOptions.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT PublishOptions.ctl"/>
				<Item Name="NT Read Boolean Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Boolean Array.vi"/>
				<Item Name="NT Read Boolean.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Boolean.vi"/>
				<Item Name="NT Read Number.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Number.vi"/>
				<Item Name="NT Read Numeric Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Numeric Array.vi"/>
				<Item Name="NT Read Raw.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Raw.vi"/>
				<Item Name="NT Read String Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read String Array.vi"/>
				<Item Name="NT Read String.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read String.vi"/>
				<Item Name="NT Read Value.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Value.vi"/>
				<Item Name="NT Read Variant.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Variant.vi"/>
				<Item Name="NT Server.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Server.vi"/>
				<Item Name="NT SubscribeOptions.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT SubscribeOptions.ctl"/>
				<Item Name="NT Write Boolean Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Boolean Array.vi"/>
				<Item Name="NT Write Boolean.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Boolean.vi"/>
				<Item Name="NT Write Number.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Number.vi"/>
				<Item Name="NT Write Numeric Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Numeric Array.vi"/>
				<Item Name="NT Write Raw.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Raw.vi"/>
				<Item Name="NT Write String Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write String Array.vi"/>
				<Item Name="NT Write String.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write String.vi"/>
				<Item Name="NT Write Value.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Value.vi"/>
				<Item Name="NT Write Variant.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Variant.vi"/>
				<Item Name="NT_Create Actual Table Name.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_Create Actual Table Name.vi"/>
				<Item Name="NT_CreateInstance.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_CreateInstance.vi"/>
				<Item Name="NT_LL_Read Boolean Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Read Boolean Array.vi"/>
				<Item Name="NT_LL_Read Boolean.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Read Boolean.vi"/>
				<Item Name="NT_LL_Read Number.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Read Number.vi"/>
				<Item Name="NT_LL_Read Numeric Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Read Numeric Array.vi"/>
				<Item Name="NT_LL_Read Raw.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Read Raw.vi"/>
				<Item Name="NT_LL_Read String Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Read String Array.vi"/>
				<Item Name="NT_LL_Read String.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Read String.vi"/>
				<Item Name="NT_LL_Write Boolean Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Write Boolean Array.vi"/>
				<Item Name="NT_LL_Write Boolean.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Write Boolean.vi"/>
				<Item Name="NT_LL_Write Number.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Write Number.vi"/>
				<Item Name="NT_LL_Write Numeric Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Write Numeric Array.vi"/>
				<Item Name="NT_LL_Write Raw.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Write Raw.vi"/>
				<Item Name="NT_LL_Write String Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Write String Array.vi"/>
				<Item Name="NT_LL_Write String.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT_LL_Write String.vi"/>
				<Item Name="Obtain Semaphore Reference.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/semaphor.llb/Obtain Semaphore Reference.vi"/>
				<Item Name="Open File+.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/file.llb/Open File+.vi"/>
				<Item Name="Open_Create_Replace File.vi" Type="VI" URL="/&lt;vilib&gt;/_oldvers/_oldvers.llb/Open_Create_Replace File.vi"/>
				<Item Name="ParseXMLFragments.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/xml.llb/ParseXMLFragments.vi"/>
				<Item Name="PWM data proto.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/PWM/PWM data proto.ctl"/>
				<Item Name="PWM proto.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/PWM/PWM proto.ctl"/>
				<Item Name="PWM Update Ops.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/PWM/PWM Update Ops.ctl"/>
				<Item Name="PWM_PanelSim_Global.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/Protocol/Devices/PWM/PWM_PanelSim_Global.vi"/>
				<Item Name="Read Delimited Spreadsheet (DBL).vi" Type="VI" URL="/&lt;vilib&gt;/Utility/file.llb/Read Delimited Spreadsheet (DBL).vi"/>
				<Item Name="Read Delimited Spreadsheet (I64).vi" Type="VI" URL="/&lt;vilib&gt;/Utility/file.llb/Read Delimited Spreadsheet (I64).vi"/>
				<Item Name="Read Delimited Spreadsheet (string).vi" Type="VI" URL="/&lt;vilib&gt;/Utility/file.llb/Read Delimited Spreadsheet (string).vi"/>
				<Item Name="Read Delimited Spreadsheet.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/file.llb/Read Delimited Spreadsheet.vi"/>
				<Item Name="Read File+ (string).vi" Type="VI" URL="/&lt;vilib&gt;/Utility/file.llb/Read File+ (string).vi"/>
				<Item Name="Read From XML File(array).vi" Type="VI" URL="/&lt;vilib&gt;/Utility/xml.llb/Read From XML File(array).vi"/>
				<Item Name="Read From XML File(string).vi" Type="VI" URL="/&lt;vilib&gt;/Utility/xml.llb/Read From XML File(string).vi"/>
				<Item Name="Read From XML File.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/xml.llb/Read From XML File.vi"/>
				<Item Name="Read Lines From File (with error IO).vi" Type="VI" URL="/&lt;vilib&gt;/Utility/file.llb/Read Lines From File (with error IO).vi"/>
				<Item Name="Refnum Registry Operation.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Utilities/Refnum Registry Operation.ctl"/>
				<Item Name="Release Semaphore.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/semaphor.llb/Release Semaphore.vi"/>
				<Item Name="RGB to Color.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/colorconv.llb/RGB to Color.vi"/>
				<Item Name="roboRIO_FPGA_2023_23.0.0.lvbitx" Type="Document" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/roboRIO_FPGA_2023_23.0.0.lvbitx"/>
				<Item Name="Semaphore RefNum" Type="VI" URL="/&lt;vilib&gt;/Utility/semaphor.llb/Semaphore RefNum"/>
				<Item Name="Semaphore Refnum Core.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/semaphor.llb/Semaphore Refnum Core.ctl"/>
				<Item Name="Space Constant.vi" Type="VI" URL="/&lt;vilib&gt;/dlg_ctls.llb/Space Constant.vi"/>
				<Item Name="spi_bus.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/spilib/spi_bus.ctl"/>
				<Item Name="System Exec.vi" Type="VI" URL="/&lt;vilib&gt;/Platform/system.llb/System Exec.vi"/>
				<Item Name="Trim Whitespace.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Trim Whitespace.vi"/>
				<Item Name="Validate Semaphore Size.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/semaphor.llb/Validate Semaphore Size.vi"/>
				<Item Name="whitespace.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/whitespace.ctl"/>
				<Item Name="WPI_AnalogChannelChannelCache.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogChannel/WPI_AnalogChannelChannelCache.vi"/>
				<Item Name="WPI_AnalogChannelChannelCacheOp.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogChannel/WPI_AnalogChannelChannelCacheOp.ctl"/>
				<Item Name="WPI_AnalogChannelClose.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogChannel/WPI_AnalogChannelClose.vi"/>
				<Item Name="WPI_AnalogChannelDevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogChannel/WPI_AnalogChannelDevRef.ctl"/>
				<Item Name="WPI_AnalogChannelERRAcquireInvalidChannel.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogChannel/WPI_AnalogChannelERRAcquireInvalidChannel.vi"/>
				<Item Name="WPI_AnalogChannelERRChannelAllocated.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogChannel/WPI_AnalogChannelERRChannelAllocated.vi"/>
				<Item Name="WPI_AnalogChannelERRScanlistInvalid.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogChannel/WPI_AnalogChannelERRScanlistInvalid.vi"/>
				<Item Name="WPI_AnalogChannelGetAverageVoltage.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogChannel/WPI_AnalogChannelGetAverageVoltage.vi"/>
				<Item Name="WPI_AnalogChannelOpen.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogChannel/WPI_AnalogChannelOpen.vi"/>
				<Item Name="WPI_AnalogChannelRefNum Registry Get.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogChannel/WPI_AnalogChannelRefNum Registry Get.vi"/>
				<Item Name="WPI_AnalogChannelSetAveraging.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogChannel/WPI_AnalogChannelSetAveraging.vi"/>
				<Item Name="WPI_AnalogChannelSetSampleRate.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogChannel/WPI_AnalogChannelSetSampleRate.vi"/>
				<Item Name="WPI_AnalogChannelWARNSampleRateTooFast.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogChannel/WPI_AnalogChannelWARNSampleRateTooFast.vi"/>
				<Item Name="WPI_AnalogOutputChannel.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/AnalogOutput/WPI_AnalogOutputChannel.ctl"/>
				<Item Name="WPI_CAN_ConvertPSI.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCH/WPI_CAN_ConvertPSI.vi"/>
				<Item Name="WPI_CAN_ConvertVToPSI.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCH/WPI_CAN_ConvertVToPSI.vi"/>
				<Item Name="WPI_CAN_GetStatus.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/WPI_CAN_GetStatus.vi"/>
				<Item Name="WPI_CAN_PCHCommands.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCH/WPI_CAN_PCHCommands.ctl"/>
				<Item Name="WPI_CAN_PCHGetAll.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCH/WPI_CAN_PCHGetAll.ctl"/>
				<Item Name="WPI_CAN_PCHGlobal.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCH/WPI_CAN_PCHGlobal.vi"/>
				<Item Name="WPI_CAN_PCHGlobalOps.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCH/WPI_CAN_PCHGlobalOps.ctl"/>
				<Item Name="WPI_CAN_PCHMessageIDs.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCH/WPI_CAN_PCHMessageIDs.ctl"/>
				<Item Name="WPI_CAN_PCHReceive.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCH/WPI_CAN_PCHReceive.vi"/>
				<Item Name="WPI_CAN_PCHSend.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCH/WPI_CAN_PCHSend.vi"/>
				<Item Name="WPI_CAN_PCMGlobal.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCM/WPI_CAN_PCMGlobal.vi"/>
				<Item Name="WPI_CAN_PCMGlobalOps.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCM/WPI_CAN_PCMGlobalOps.ctl"/>
				<Item Name="WPI_CAN_PCMMessageIDs.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCM/WPI_CAN_PCMMessageIDs.ctl"/>
				<Item Name="WPI_CAN_PCMReceive.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCM/WPI_CAN_PCMReceive.vi"/>
				<Item Name="WPI_CAN_PCMSend.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PCM/WPI_CAN_PCMSend.vi"/>
				<Item Name="WPI_CAN_PDHCommands.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDH/WPI_CAN_PDHCommands.ctl"/>
				<Item Name="WPI_CAN_PDHDecode4Currents.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDH/WPI_CAN_PDHDecode4Currents.vi"/>
				<Item Name="WPI_CAN_PDHGetAll.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDH/WPI_CAN_PDHGetAll.ctl"/>
				<Item Name="WPI_CAN_PDHGlobal.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDH/WPI_CAN_PDHGlobal.vi"/>
				<Item Name="WPI_CAN_PDHGlobalOps.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDH/WPI_CAN_PDHGlobalOps.ctl"/>
				<Item Name="WPI_CAN_PDHMessageIDs.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDH/WPI_CAN_PDHMessageIDs.ctl"/>
				<Item Name="WPI_CAN_PDHReceive.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDH/WPI_CAN_PDHReceive.vi"/>
				<Item Name="WPI_CAN_PDHSend.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDH/WPI_CAN_PDHSend.vi"/>
				<Item Name="WPI_CAN_PDPCommands.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDP/WPI_CAN_PDPCommands.ctl"/>
				<Item Name="WPI_CAN_PDPDecode4Currents.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDP/WPI_CAN_PDPDecode4Currents.vi"/>
				<Item Name="WPI_CAN_PDPGetAll.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDP/WPI_CAN_PDPGetAll.ctl"/>
				<Item Name="WPI_CAN_PDPGlobal.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDP/WPI_CAN_PDPGlobal.vi"/>
				<Item Name="WPI_CAN_PDPGlobalOps.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDP/WPI_CAN_PDPGlobalOps.ctl"/>
				<Item Name="WPI_CAN_PDPMessageIDs.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDP/WPI_CAN_PDPMessageIDs.ctl"/>
				<Item Name="WPI_CAN_PDPReceive.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDP/WPI_CAN_PDPReceive.vi"/>
				<Item Name="WPI_CAN_PDPSend.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/CAN/PDP/WPI_CAN_PDPSend.vi"/>
				<Item Name="WPI_CompressorDevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Compressor/WPI_CompressorDevRef.ctl"/>
				<Item Name="WPI_CompressorFeedbackMode.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Compressor/WPI_CompressorFeedbackMode.ctl"/>
				<Item Name="WPI_CompressorGetEnableState.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Compressor/WPI_CompressorGetEnableState.vi"/>
				<Item Name="WPI_CompressorOpen.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Compressor/WPI_CompressorOpen.vi"/>
				<Item Name="WPI_CompressorRefNum Registry Get.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Compressor/WPI_CompressorRefNum Registry Get.vi"/>
				<Item Name="WPI_CompressorRefNum Registry Set.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Compressor/WPI_CompressorRefNum Registry Set.vi"/>
				<Item Name="WPI_CompressorStart.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Compressor/WPI_CompressorStart.vi"/>
				<Item Name="WPI_CompressorStop.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Compressor/WPI_CompressorStop.vi"/>
				<Item Name="WPI_CounterDevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Counter/WPI_CounterDevRef.ctl"/>
				<Item Name="WPI_CounterRefNum Registry Get.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Counter/WPI_CounterRefNum Registry Get.vi"/>
				<Item Name="WPI_CTRE_MotorController_Callback_Disable.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/WPI Callback VIs/WPI_CTRE_MotorController_Callback_Disable.vi"/>
				<Item Name="WPI_CTRE_MotorController_Callback_Enable.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/WPI Callback VIs/WPI_CTRE_MotorController_Callback_Enable.vi"/>
				<Item Name="WPI_CTRE_MotorController_Callback_SetOutput.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/ThirdParty/Phoenix-LabVIEW/Motor Controller/WPI Callback VIs/WPI_CTRE_MotorController_Callback_SetOutput.vi"/>
				<Item Name="WPI_DefaultPWMConstants.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_DefaultPWMConstants.vi"/>
				<Item Name="WPI_DigitalInputClose.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalInput/WPI_DigitalInputClose.vi"/>
				<Item Name="WPI_DigitalInputDevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalInput/WPI_DigitalInputDevRef.ctl"/>
				<Item Name="WPI_DigitalInputGetValue.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalInput/WPI_DigitalInputGetValue.vi"/>
				<Item Name="WPI_DigitalInputOpen.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalInput/WPI_DigitalInputOpen.vi"/>
				<Item Name="WPI_DigitalInputRefNum Registry Get.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalInput/WPI_DigitalInputRefNum Registry Get.vi"/>
				<Item Name="WPI_DigitalInputRefNum Registry Set.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalInput/WPI_DigitalInputRefNum Registry Set.vi"/>
				<Item Name="WPI_DigitalInputToDigitalModule.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalInput/WPI_DigitalInputToDigitalModule.vi"/>
				<Item Name="WPI_DigitalModuleDeviceRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalModule/WPI_DigitalModuleDeviceRef.ctl"/>
				<Item Name="WPI_DigitalModuleDIOAllocator.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalModule/WPI_DigitalModuleDIOAllocator.vi"/>
				<Item Name="WPI_DigitalModuleDIOCache.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalModule/WPI_DigitalModuleDIOCache.vi"/>
				<Item Name="WPI_DigitalModuleDIOCacheOp.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalModule/WPI_DigitalModuleDIOCacheOp.ctl"/>
				<Item Name="WPI_DigitalModuleERRDIOAlreadyAllocated.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalModule/WPI_DigitalModuleERRDIOAlreadyAllocated.vi"/>
				<Item Name="WPI_DigitalModuleERRInvalidDIOIdx.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalModule/WPI_DigitalModuleERRInvalidDIOIdx.vi"/>
				<Item Name="WPI_DigitalModuleERRSetOnUnallocatedDIO.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalModule/WPI_DigitalModuleERRSetOnUnallocatedDIO.vi"/>
				<Item Name="WPI_DigitalModuleGetDIO.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalModule/WPI_DigitalModuleGetDIO.vi"/>
				<Item Name="WPI_DigitalModuleOpen.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalModule/WPI_DigitalModuleOpen.vi"/>
				<Item Name="WPI_DigitalOutputClose.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalOutput/WPI_DigitalOutputClose.vi"/>
				<Item Name="WPI_DigitalOutputDevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalOutput/WPI_DigitalOutputDevRef.ctl"/>
				<Item Name="WPI_DigitalOutputDisablePWM.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalOutput/WPI_DigitalOutputDisablePWM.vi"/>
				<Item Name="WPI_DigitalOutputGetPWM.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalOutput/WPI_DigitalOutputGetPWM.vi"/>
				<Item Name="WPI_DigitalOutputPWMAllocator.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalOutput/WPI_DigitalOutputPWMAllocator.vi"/>
				<Item Name="WPI_DigitalOutputPWMRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalOutput/WPI_DigitalOutputPWMRef.ctl"/>
				<Item Name="WPI_DisableFilter.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalInput/Filter/WPI_DisableFilter.vi"/>
				<Item Name="WPI_DriverStationAllianceInfo.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationAllianceInfo.ctl"/>
				<Item Name="WPI_DriverStationCreate Lib Version File.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationCreate Lib Version File.vi"/>
				<Item Name="WPI_DriverStationDerivedRobotMode.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationDerivedRobotMode.ctl"/>
				<Item Name="WPI_DriverStationGet Alliance Info.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationGet Alliance Info.vi"/>
				<Item Name="WPI_DriverStationGet Robot Mode.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationGet Robot Mode.vi"/>
				<Item Name="WPI_DriverStationGetModeAndStatusInternal.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationGetModeAndStatusInternal.vi"/>
				<Item Name="WPI_DriverStationGetRemainingMatchTime.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationGetRemainingMatchTime.vi"/>
				<Item Name="WPI_DriverStationMatch Info.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationMatch Info.ctl"/>
				<Item Name="WPI_DriverStationPositionInfo.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationPositionInfo.ctl"/>
				<Item Name="WPI_DriverStationReport Robot Code State.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationReport Robot Code State.vi"/>
				<Item Name="WPI_DriverStationRobotMode2.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationRobotMode2.ctl"/>
				<Item Name="WPI_DriverStationStart Communication.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationStart Communication.vi"/>
				<Item Name="WPI_DriverStationStart VI Asynchronous.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationStart VI Asynchronous.vi"/>
				<Item Name="WPI_DriverStationStartStopVI.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationStartStopVI.vi"/>
				<Item Name="WPI_DriverStationStop VI Asynchronous.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationStop VI Asynchronous.vi"/>
				<Item Name="WPI_ERRAcquireInvalidPWMChannelIdx.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_ERRAcquireInvalidPWMChannelIdx.vi"/>
				<Item Name="WPI_ERRAllocAnInvalidSolenoidIdx.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_ERRAllocAnInvalidSolenoidIdx.vi"/>
				<Item Name="WPI_ERRFilterReserved.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalInput/Filter/WPI_ERRFilterReserved.vi"/>
				<Item Name="WPI_ERRPwmChannelAllocated.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_ERRPwmChannelAllocated.vi"/>
				<Item Name="WPI_ERRSolenoidAllocation.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_ERRSolenoidAllocation.vi"/>
				<Item Name="WPI_ERRSolenoidNotAllocated.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_ERRSolenoidNotAllocated.vi"/>
				<Item Name="WPI_GetSetVariantRefNum.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Utilities/WPI_GetSetVariantRefNum.vi"/>
				<Item Name="WPI_I2C_ERRI2CAlreadyAllocated.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/I2C/WPI_I2C_ERRI2CAlreadyAllocated.vi"/>
				<Item Name="WPI_I2C_ERRInvalidI2C_Index.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/I2C/WPI_I2C_ERRInvalidI2C_Index.vi"/>
				<Item Name="WPI_I2CBusCache.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/I2C/WPI_I2CBusCache.vi"/>
				<Item Name="WPI_I2CClose.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/I2C/WPI_I2CClose.vi"/>
				<Item Name="WPI_I2CDevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/I2C/WPI_I2CDevRef.ctl"/>
				<Item Name="WPI_I2COpen.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/I2C/WPI_I2COpen.vi"/>
				<Item Name="WPI_I2CRead.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/I2C/WPI_I2CRead.vi"/>
				<Item Name="WPI_I2CWrite.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/I2C/WPI_I2CWrite.vi"/>
				<Item Name="WPI_JoystickDeviceEnum.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Joystick/WPI_JoystickDeviceEnum.ctl"/>
				<Item Name="WPI_JoystickDevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Joystick/WPI_JoystickDevRef.ctl"/>
				<Item Name="WPI_JoystickGetValues.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Joystick/WPI_JoystickGetValues.vi"/>
				<Item Name="WPI_JoystickOpen.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Joystick/WPI_JoystickOpen.vi"/>
				<Item Name="WPI_JoystickRefNum Registry Get.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Joystick/WPI_JoystickRefNum Registry Get.vi"/>
				<Item Name="WPI_JoystickRefNum Registry Set.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Joystick/WPI_JoystickRefNum Registry Set.vi"/>
				<Item Name="WPI_JoystickSetOutputs.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Joystick/WPI_JoystickSetOutputs.vi"/>
				<Item Name="WPI_LED_Open.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/LED/WPI_LED_Open.vi"/>
				<Item Name="WPI_LED_Write.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/LED/WPI_LED_Write.vi"/>
				<Item Name="WPI_MotorControlClose.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlClose.vi"/>
				<Item Name="WPI_MotorControlDeviceRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlDeviceRef.ctl"/>
				<Item Name="WPI_MotorControlDisable.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlDisable.vi"/>
				<Item Name="WPI_MotorControlDisableRefType.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlDisableRefType.ctl"/>
				<Item Name="WPI_MotorControlDoesDevRefExist.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlDoesDevRefExist.vi"/>
				<Item Name="WPI_MotorControlGetOutput.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlGetOutput.vi"/>
				<Item Name="WPI_MotorControlGetOutputRefType.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlGetOutputRefType.ctl"/>
				<Item Name="WPI_MotorControlRefNum Compare.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlRefNum Compare.vi"/>
				<Item Name="WPI_MotorControlRefNum Registry Get.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlRefNum Registry Get.vi"/>
				<Item Name="WPI_MotorControlRefNum Registry Read Name.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlRefNum Registry Read Name.vi"/>
				<Item Name="WPI_MotorControlSafetyCheck.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlSafetyCheck.vi"/>
				<Item Name="WPI_MotorControlSafetyError.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlSafetyError.vi"/>
				<Item Name="WPI_MotorControlSafetyUpdate.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlSafetyUpdate.vi"/>
				<Item Name="WPI_MotorControlScaleFromPWM.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlScaleFromPWM.vi"/>
				<Item Name="WPI_MotorControlToPWM.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlToPWM.vi"/>
				<Item Name="WPI_MotorControlTrackCANSemaphores.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlTrackCANSemaphores.vi"/>
				<Item Name="WPI_MotorControlType.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlType.ctl"/>
				<Item Name="WPI_MXP DIO Channel to MXP Pin.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MXP/WPI_MXP DIO Channel to MXP Pin.vi"/>
				<Item Name="WPI_MXP General IO to MXP Pin.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MXP/WPI_MXP General IO to MXP Pin.vi"/>
				<Item Name="WPI_MXP I2C Bus to first MXP Pin.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MXP/WPI_MXP I2C Bus to first MXP Pin.vi"/>
				<Item Name="WPI_MXP PWM Channel to MXP Pin.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MXP/WPI_MXP PWM Channel to MXP Pin.vi"/>
				<Item Name="WPI_MXP Shared MXP Pin.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MXP/WPI_MXP Shared MXP Pin.ctl"/>
				<Item Name="WPI_MXP SPI Bus to first MXP Pin.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MXP/WPI_MXP SPI Bus to first MXP Pin.vi"/>
				<Item Name="WPI_MXP_ChannelCache.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MXP/WPI_MXP_ChannelCache.vi"/>
				<Item Name="WPI_MXP_Grab Mutex.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MXP/WPI_MXP_Grab Mutex.vi"/>
				<Item Name="WPI_PotentiometerClose.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Potentiometer/WPI_PotentiometerClose.vi"/>
				<Item Name="WPI_PotentiometerDevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Potentiometer/WPI_PotentiometerDevRef.ctl"/>
				<Item Name="WPI_PotentiometerGetNormalized.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Potentiometer/WPI_PotentiometerGetNormalized.vi"/>
				<Item Name="WPI_PotentiometerOpen.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Potentiometer/WPI_PotentiometerOpen.vi"/>
				<Item Name="WPI_PotentiometerRefNum Registry Get.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Potentiometer/WPI_PotentiometerRefNum Registry Get.vi"/>
				<Item Name="WPI_PotentiometerRefNum Registry Set.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Potentiometer/WPI_PotentiometerRefNum Registry Set.vi"/>
				<Item Name="WPI_Power CAN Device Type.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power CAN Device Type.ctl"/>
				<Item Name="WPI_Power Discovered CAN Devices.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power Discovered CAN Devices.vi"/>
				<Item Name="WPI_Power Enable PD Switched Power.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power Enable PD Switched Power.vi"/>
				<Item Name="WPI_Power Get 3Volt Rail Voltage and Current.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power Get 3Volt Rail Voltage and Current.vi"/>
				<Item Name="WPI_Power Get 5Volt Rail Voltage and Current.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power Get 5Volt Rail Voltage and Current.vi"/>
				<Item Name="WPI_Power Get 6Volt Rail Voltage and Current.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power Get 6Volt Rail Voltage and Current.vi"/>
				<Item Name="WPI_Power Get PD Currents.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power Get PD Currents.vi"/>
				<Item Name="WPI_Power Get PD Faults.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power Get PD Faults.vi"/>
				<Item Name="WPI_Power Get PD Voltage.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power Get PD Voltage.vi"/>
				<Item Name="WPI_Power Get Pneumatic Faults.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power Get Pneumatic Faults.vi"/>
				<Item Name="WPI_Power Get Rail Voltage and Current.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power Get Rail Voltage and Current.vi"/>
				<Item Name="WPI_Power Get roboRIO Rail Faults.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power Get roboRIO Rail Faults.vi"/>
				<Item Name="WPI_Power Pneumatics Type Cache.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power Pneumatics Type Cache.vi"/>
				<Item Name="WPI_Power PowerDist Type Cache.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_Power PowerDist Type Cache.vi"/>
				<Item Name="WPI_PowerERRSwitchNotSupported.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Power/WPI_PowerERRSwitchNotSupported.vi"/>
				<Item Name="WPI_PWMChannelCache.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMChannelCache.vi"/>
				<Item Name="WPI_PWMChannelCacheOp.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMChannelCacheOp.ctl"/>
				<Item Name="WPI_PWMClose.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMClose.vi"/>
				<Item Name="WPI_PWMDeadband.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMDeadband.ctl"/>
				<Item Name="WPI_PWMDeviceRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMDeviceRef.ctl"/>
				<Item Name="WPI_PWMDoesDevRefExist.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMDoesDevRefExist.vi"/>
				<Item Name="WPI_PWMERRSetOnUnallocatedChannel.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMERRSetOnUnallocatedChannel.vi"/>
				<Item Name="WPI_PWMGetRangeScaleFactors.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMGetRangeScaleFactors.vi"/>
				<Item Name="WPI_PWMGetValue.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMGetValue.vi"/>
				<Item Name="WPI_PWMOpen.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMOpen.vi"/>
				<Item Name="WPI_PWMPeriodMultiplier.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMPeriodMultiplier.ctl"/>
				<Item Name="WPI_PWMRefNum Registry Get.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMRefNum Registry Get.vi"/>
				<Item Name="WPI_PWMRefNum Registry Read Name.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMRefNum Registry Read Name.vi"/>
				<Item Name="WPI_PWMRefNum Registry Set.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMRefNum Registry Set.vi"/>
				<Item Name="WPI_PWMSafetyCheck.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMSafetyCheck.vi"/>
				<Item Name="WPI_PWMSafetyError.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMSafetyError.vi"/>
				<Item Name="WPI_PWMSafetyUpdate.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMSafetyUpdate.vi"/>
				<Item Name="WPI_PWMSetValue.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMSetValue.vi"/>
				<Item Name="WPI_ReserveFilterResource.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalInput/Filter/WPI_ReserveFilterResource.vi"/>
				<Item Name="WPI_SafetyOutputCheckMenu.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/SafetyOutput/WPI_SafetyOutputCheckMenu.ctl"/>
				<Item Name="WPI_SafetyOutputEngine.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/SafetyOutput/WPI_SafetyOutputEngine.vi"/>
				<Item Name="WPI_SafetyOutputVIRefnumList.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/SafetyOutput/WPI_SafetyOutputVIRefnumList.vi"/>
				<Item Name="WPI_ServoDeviceRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/Servo/WPI_ServoDeviceRef.ctl"/>
				<Item Name="WPI_ServoRefNum Registry Get.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/Servo/WPI_ServoRefNum Registry Get.vi"/>
				<Item Name="WPI_SolenoidCache.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidCache.vi"/>
				<Item Name="WPI_SolenoidCacheOp.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidCacheOp.ctl"/>
				<Item Name="WPI_SolenoidChannel.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidChannel.ctl"/>
				<Item Name="WPI_SolenoidClose.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidClose.vi"/>
				<Item Name="WPI_SolenoidDevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidDevRef.ctl"/>
				<Item Name="WPI_SolenoidDoesDevRefExist.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidDoesDevRefExist.vi"/>
				<Item Name="WPI_SolenoidGet.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidGet.vi"/>
				<Item Name="WPI_SolenoidGetBool.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/Deprecated/WPI_SolenoidGetBool.vi"/>
				<Item Name="WPI_SolenoidGetEnum.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidGetEnum.vi"/>
				<Item Name="WPI_SolenoidOpen.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidOpen.vi"/>
				<Item Name="WPI_SolenoidOpenDouble.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidOpenDouble.vi"/>
				<Item Name="WPI_SolenoidOpenSingle.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidOpenSingle.vi"/>
				<Item Name="WPI_SolenoidRefNum Registry Get.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidRefNum Registry Get.vi"/>
				<Item Name="WPI_SolenoidRefNum Registry Read Name.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidRefNum Registry Read Name.vi"/>
				<Item Name="WPI_SolenoidRefNum Registry Set.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidRefNum Registry Set.vi"/>
				<Item Name="WPI_SolenoidSafetyCheck.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidSafetyCheck.vi"/>
				<Item Name="WPI_SolenoidSafetyError.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidSafetyError.vi"/>
				<Item Name="WPI_SolenoidSafetyUpdate.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidSafetyUpdate.vi"/>
				<Item Name="WPI_SolenoidSet.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidSet.vi"/>
				<Item Name="WPI_SolenoidSetBool.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/Deprecated/WPI_SolenoidSetBool.vi"/>
				<Item Name="WPI_SolenoidSetEnum.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidSetEnum.vi"/>
				<Item Name="WPI_SolenoidValue.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidValue.ctl"/>
				<Item Name="WPI_SPIBusCacheOp.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/SPI/WPI_SPIBusCacheOp.ctl"/>
				<Item Name="WPI_SwitchDevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Switch/WPI_SwitchDevRef.ctl"/>
				<Item Name="WPI_SwitchRefNum Registry Get.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Switch/WPI_SwitchRefNum Registry Get.vi"/>
				<Item Name="WPI_UtilitiesERRGetRefNum.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Utilities/WPI_UtilitiesERRGetRefNum.vi"/>
				<Item Name="WPI_UtilitiesFRC Build Error.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Utilities/WPI_UtilitiesFRC Build Error.vi"/>
				<Item Name="WPI_UtilitiesFRC FPGATime.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Utilities/WPI_UtilitiesFRC FPGATime.vi"/>
				<Item Name="WPI_UtilitiesFRC SendMessageToConsole.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Utilities/WPI_UtilitiesFRC SendMessageToConsole.vi"/>
				<Item Name="WPI_UtilitiesFRCAutomaticValuePublisher.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Utilities/WPI_UtilitiesFRCAutomaticValuePublisher.vi"/>
				<Item Name="WPI_UtilitiesTimebaseConstants.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Utilities/WPI_UtilitiesTimebaseConstants.vi"/>
				<Item Name="Write to XML File(array).vi" Type="VI" URL="/&lt;vilib&gt;/Utility/xml.llb/Write to XML File(array).vi"/>
				<Item Name="Write to XML File(string).vi" Type="VI" URL="/&lt;vilib&gt;/Utility/xml.llb/Write to XML File(string).vi"/>
				<Item Name="Write to XML File.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/xml.llb/Write to XML File.vi"/>
			</Item>
			<Item Name="FRC_NetworkCommunication.dll" Type="Document" URL="FRC_NetworkCommunication.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
			<Item Name="FRC_NetworkTablesLV.dll" Type="Document" URL="FRC_NetworkTablesLV.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
			<Item Name="hmblv.dll" Type="Document" URL="hmblv.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
			<Item Name="lvanlys.dll" Type="Document" URL="/&lt;resource&gt;/lvanlys.dll"/>
			<Item Name="NiFpgaLv.dll" Type="Document" URL="NiFpgaLv.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
			<Item Name="nivision.dll" Type="Document" URL="nivision.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
			<Item Name="ntcoreffi.dll" Type="Document" URL="ntcoreffi.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
		</Item>
		<Item Name="Build Specifications" Type="Build">
			<Item Name="FRC Robot Boot-up Deployment" Type="{69A947D5-514E-4E75-818E-69657C0547D8}">
				<Property Name="App_INI_aliasGUID" Type="Str">{7AAF6F56-ED86-4686-A01D-90E0BB9C8086}</Property>
				<Property Name="App_INI_GUID" Type="Str">{A18A8773-40AE-4BF9-9DC4-FD6E80D0C49E}</Property>
				<Property Name="App_serverConfig.httpPort" Type="Int">8002</Property>
				<Property Name="App_serverType" Type="Int">0</Property>
				<Property Name="App_winsec.description" Type="Str">http://www.NI.com</Property>
				<Property Name="Bld_autoIncrement" Type="Bool">true</Property>
				<Property Name="Bld_buildCacheID" Type="Str">{8C1C8770-D686-4931-835E-3A364C203F20}</Property>
				<Property Name="Bld_buildSpecDescription" Type="Str">Build Robot Main.vi into an EXE that will run at startup on the roboRIO</Property>
				<Property Name="Bld_buildSpecName" Type="Str">FRC Robot Boot-up Deployment</Property>
				<Property Name="Bld_excludeLibraryItems" Type="Bool">true</Property>
				<Property Name="Bld_excludePolymorphicVIs" Type="Bool">true</Property>
				<Property Name="Bld_localDestDir" Type="Path">/C/Git Checkouts/Robot Builds/2023</Property>
				<Property Name="Bld_previewCacheID" Type="Str">{6782B190-04E1-4A41-93AB-3F357B35791E}</Property>
				<Property Name="Bld_targetDestDir" Type="Path">/home/lvuser/natinst/bin</Property>
				<Property Name="Bld_version.build" Type="Int">21</Property>
				<Property Name="Bld_version.major" Type="Int">1</Property>
				<Property Name="Destination[0].destName" Type="Str">startup.rtexe</Property>
				<Property Name="Destination[0].path" Type="Path">/home/lvuser/natinst/bin/startup.rtexe</Property>
				<Property Name="Destination[0].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[0].preserveHierarchy" Type="Bool">true</Property>
				<Property Name="Destination[0].type" Type="Str">App</Property>
				<Property Name="Destination[1].destName" Type="Str">Support Directory</Property>
				<Property Name="Destination[1].path" Type="Path">/home/lvuser/natinst/bin/data</Property>
				<Property Name="Destination[1].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="DestinationCount" Type="Int">2</Property>
				<Property Name="Source[0].itemID" Type="Str">{21C2D0A2-46E5-4471-9CC6-B62844B36A97}</Property>
				<Property Name="Source[0].type" Type="Str">Container</Property>
				<Property Name="Source[1].destinationIndex" Type="Int">0</Property>
				<Property Name="Source[1].itemID" Type="Ref">/Target/Robot Main.vi</Property>
				<Property Name="Source[1].sourceInclusion" Type="Str">TopLevel</Property>
				<Property Name="Source[1].type" Type="Str">VI</Property>
				<Property Name="SourceCount" Type="Int">2</Property>
				<Property Name="TgtF_companyName" Type="Str">NI</Property>
				<Property Name="TgtF_fileDescription" Type="Str">FRC Robot Boot-up Deployment</Property>
				<Property Name="TgtF_internalName" Type="Str">FRC Robot Boot-up Deployment</Property>
				<Property Name="TgtF_legalCopyright" Type="Str">Copyright © 2013 NI</Property>
				<Property Name="TgtF_productName" Type="Str">FRC Robot Boot-up Deployment</Property>
				<Property Name="TgtF_targetfileGUID" Type="Str">{8C9344B8-7ED9-4E5F-92AE-3A9F547EFD4A}</Property>
				<Property Name="TgtF_targetfileName" Type="Str">startup.rtexe</Property>
			</Item>
		</Item>
	</Item>
</Project>
