package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromRight.LeftRight;

import org.usfirst.frc.team195.robot.Actions.AutomatedActions;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetArmRotationAction;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetElevatorHeightAction;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetIntakeAction;
import org.usfirst.frc.team195.robot.Actions.DrivePathAction;
import org.usfirst.frc.team195.robot.Actions.Framework.ParallelAction;
import org.usfirst.frc.team195.robot.Actions.Framework.SeriesAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitForPathMarkerAction;
import org.usfirst.frc.team195.robot.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team195.robot.Actions.TurnToHeadingAction;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftRight_2cube.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ArmPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class LeftRightFromRightMode_2cube extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new LeftRightFromRightStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
												       AutomatedActions.PreparePlaceCubeOnScaleOverBack())))));

		runAction(AutomatedActions.OutakeCubeMidSpeed());
		runAction(AutomatedActions.StopIntake());

////		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new LeftRightFromRightStep2()),
////												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePickupCube"),
////													   AutomatedActions.PreparePickupCube(),
////												       new SetIntakeAction(IntakeControl.INTAKE_IN))))));
//
//		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new LeftRightFromRightStep2()),
//												   AutomatedActions.PreparePickupCube(),
//												   new SetIntakeAction(IntakeControl.INTAKE_IN))));
//
//		runAction(AutomatedActions.ClampIntake());
//		runAction(new WaitAction(0.1));
//		runAction(AutomatedActions.StopIntake());
//
//		runAction(new DrivePathAction(new LeftRightFromRightStep3()));
//
//		runAction(AutomatedActions.LiftArmTo90());
//
//		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new LeftRightFromRightStep4()),
//												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
//													   new SetElevatorHeightAction(ElevatorPosition.LOW))))));
//
//		//runAction(new DrivePathAction(new LeftRightFromRightStep5Final()));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new LeftRightFromRightTestStep()),
												   AutomatedActions.SetRestingPosition(),
												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePickupCube"),
													   				AutomatedActions.PreparePickupCube(),
														   			new SetIntakeAction(IntakeControl.INTAKE_IN))))));

//		runAction(new DrivePathAction(new LeftRightFromRightTestStep2()));
//		runAction(new DrivePathAction(new LeftRightFromRightTestStep3()));

		runAction(AutomatedActions.ClampIntake());
		runAction(new WaitAction(0.1));
		runAction(AutomatedActions.StopIntake());

		runAction(new SetArmRotationAction(ArmPosition.VERTICAL));
//		runAction(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPosition.SWITCH),
//											   new SetArmRotationAction(ArmPosition.VERTICAL))));

		runAction(new TurnToHeadingAction(160));

		//runAction(new DrivePathAction(new LeftRightFromRightTestStep4()));

//		runAction(new WaitAction(.5));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new LeftRightFromRightTestStep5()),
												    new SetArmRotationAction(ArmPosition.SWITCH))));

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

		runAction(AutomatedActions.SetRestingPosition());

		runAction(new WaitAction(15));
	}
}