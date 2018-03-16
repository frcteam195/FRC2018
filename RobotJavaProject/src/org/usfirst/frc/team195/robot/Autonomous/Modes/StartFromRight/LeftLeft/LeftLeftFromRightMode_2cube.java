package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromRight.LeftLeft;

import org.usfirst.frc.team195.robot.Actions.AutomatedActions;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetIntakeAction;
import org.usfirst.frc.team195.robot.Actions.DrivePathAction;
import org.usfirst.frc.team195.robot.Actions.Framework.ParallelAction;
import org.usfirst.frc.team195.robot.Actions.Framework.SeriesAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitForPathMarkerAction;
import org.usfirst.frc.team195.robot.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftLeft_2cube.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class LeftLeftFromRightMode_2cube extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new LeftLeftFromRight_2cubeStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
														   AutomatedActions.PreparePlaceCubeOnScaleOverBackMid())))));

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new LeftLeftFromRight_2cubeStep2()),
												   new SetIntakeAction(IntakeControl.INTAKE_IN),
												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePickupCube"),
													   AutomatedActions.PreparePickupCube())))));

		runAction(AutomatedActions.ClampIntake());
		runAction(new WaitAction(0.1));
		runAction(AutomatedActions.StopIntake());

		runAction(new SeriesAction(Arrays.asList(AutomatedActions.PreparePlaceCubeOnSwitchElevator(),
												 new DrivePathAction(new LeftLeftFromRight_2cubeStep3()))));

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

		runAction(AutomatedActions.LiftArmTo90());
		runAction(new WaitAction(0.4));
		runAction(AutomatedActions.ElevetorTo0());

		runAction(new WaitAction(15));
	}
}