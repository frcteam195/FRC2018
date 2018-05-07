package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromCenter.Right;

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
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromCenter.Right3Cube.*;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ArmPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class RightFromCenterMode_3cube extends AutoModeBase {

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

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightFromCenterStep3()),
				new SetIntakeAction(IntakeControl.INTAKE_IN))));

		runAction(AutomatedActions.GrabCube());

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightFromCenterStep4()),
				AutomatedActions.LiftArmTo90())));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightFromCenterStep5()),
				new SetElevatorHeightAction(ElevatorPosition.SWITCH),
				new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("ArmDown"),
						new SetArmRotationAction(ArmPosition.LOW))))));

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightFromCenterStep6()),
												   AutomatedActions.PreparePickupCube())));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightFromCenterStep7()),
				new SetIntakeAction(IntakeControl.INTAKE_IN))));


		runAction(AutomatedActions.GrabCube());

		runAction(new DrivePathAction(new RightFromCenterStep8()));

		//runAction(new DrivePathAction(new RightFromCenterStep9()));

		runAction(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPosition.SWITCH),
												   new SetArmRotationAction(ArmPosition.LOW),
												   new DrivePathAction(new RightFromCenterStep9Alt()))));

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new WaitAction(15));
	}
}
