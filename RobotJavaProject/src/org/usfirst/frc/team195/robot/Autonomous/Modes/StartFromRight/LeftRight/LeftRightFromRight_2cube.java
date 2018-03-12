package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromRight.LeftRight;

import org.usfirst.frc.team195.robot.Actions.AutomatedActions;
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
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftRight_2cube.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class LeftRightFromRight_2cube extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new LeftRightFromRightStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
												       AutomatedActions.PreparePlaceCubeOnScaleOverBack())))));

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new LeftRightFromRightStep2()),
												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePickupCube"),
													   AutomatedActions.PreparePickupCube(),
												       new SetIntakeAction(IntakeControl.INTAKE_IN))))));

		runAction(AutomatedActions.ClampIntake());
		runAction(new WaitAction(0.1));
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new LeftRightFromRightStep3()),
												   new SetElevatorHeightAction(ElevatorPosition.SWITCH))));

		runAction(new DrivePathAction(new LeftRightFromRightStep4Final()));

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new WaitAction(15));
	}
}