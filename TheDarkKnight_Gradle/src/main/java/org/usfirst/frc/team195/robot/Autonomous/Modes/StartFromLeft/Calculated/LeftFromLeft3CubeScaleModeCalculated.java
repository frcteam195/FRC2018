package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromLeft.Calculated;

import org.usfirst.frc.team195.robot.Actions.AutomatedActions;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetArmRotationAction;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetIntakeAction;
import org.usfirst.frc.team195.robot.Actions.DrivePathAction;
import org.usfirst.frc.team195.robot.Actions.Framework.ParallelAction;
import org.usfirst.frc.team195.robot.Actions.Framework.SeriesAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitForPathMarkerAction;
import org.usfirst.frc.team195.robot.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.Left3CubeScale.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ArmPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class LeftFromLeft3CubeScaleModeCalculated extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new Left3CubeScaleStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
														                          AutomatedActions.PreparePlaceCubeCalculatedScaleHeight(ElevatorPosition.OVER_THE_BACK_MID_MORE, ArmPosition.BACK))))));

		runAction(AutomatedActions.OutakeCubeExtraFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCube(),
												   new SetIntakeAction(IntakeControl.INTAKE_IN),
												   new SeriesAction(Arrays.asList(new WaitAction(0.5),
																				  new DrivePathAction(new Left3CubeScaleStep2()))))));

		runAction(AutomatedActions.GrabCube());

		runAction(new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.VERTICAL),
												   new DrivePathAction(new Left3CubeScaleStep3()),
												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
														 						  AutomatedActions.PreparePlaceCubeCalculatedScaleHeight(ElevatorPosition.OVER_THE_BACK_MID_MORE, ArmPosition.BACK))))));

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());
		runAction(new WaitAction(0.2));

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCube(),
												   new SetIntakeAction(IntakeControl.INTAKE_IN),
												   new SeriesAction(Arrays.asList(new WaitAction(0.5),
														   						  new DrivePathAction(new Left3CubeScaleStep4()))))));

		runAction(AutomatedActions.GrabCube());

		runAction(new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.VERTICAL),
												   new DrivePathAction(new Left3CubeScaleStep5()),
												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
														   						  AutomatedActions.PreparePlaceCubeCalculatedScaleHeight(ElevatorPosition.OVER_THE_BACK_MID_MORE, ArmPosition.BACK))))));

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCube(),
												   new SetIntakeAction(IntakeControl.INTAKE_IN),
												   new DrivePathAction(new Left3CubeScaleStep6()))));

		runAction(AutomatedActions.GrabCube());

		runAction(new WaitAction(15));
	}
}
