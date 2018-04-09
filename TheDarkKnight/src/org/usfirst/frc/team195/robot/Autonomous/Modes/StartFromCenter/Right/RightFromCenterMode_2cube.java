package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromCenter.Right;

import org.usfirst.frc.team195.robot.Actions.AutomatedActions;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetArmRotationAction;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetElevatorHeightAction;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetIntakeAction;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetIntakeClampAction;
import org.usfirst.frc.team195.robot.Actions.DrivePathAction;
import org.usfirst.frc.team195.robot.Actions.Framework.ParallelAction;
import org.usfirst.frc.team195.robot.Actions.Framework.SeriesAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitForPathMarkerAction;
import org.usfirst.frc.team195.robot.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromCenter.Right2Cube.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ArmPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class RightFromCenterMode_2cube extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new RightFromCenterStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));

		//runAction(new DrivePathAction(pathContainer));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
												   new SetElevatorHeightAction(ElevatorPosition.SWITCH),
												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("ArmDown"),
																				  new SetArmRotationAction(ArmPosition.LOW))))));

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

//		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightFromCenterStep2()),
//												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePickupCube"),
//													   AutomatedActions.PreparePickupCube())))));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightFromCenterStep2()),
												   AutomatedActions.PreparePickupCube())));

		runAction(new SetIntakeClampAction(true));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightFromCenterStep3()),
												   new SetIntakeAction(IntakeControl.INTAKE_IN))));

		runAction(new WaitAction(0.1));
		runAction(AutomatedActions.ClampIntake());
		runAction(new WaitAction(0.1));
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightFromCenterStep4()),
												   AutomatedActions.LiftArmTo90())));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightFromCenterStep5Final()),
												   new SetElevatorHeightAction(ElevatorPosition.SWITCH),
												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("ArmDown"),
																				  new SetArmRotationAction(ArmPosition.LOW))))));

		runAction(AutomatedActions.OutakeCubeMidSpeed());
		runAction(AutomatedActions.StopIntake());

		runAction(AutomatedActions.SetRestingPosition());

		runAction(new WaitAction(15));
	}
}
