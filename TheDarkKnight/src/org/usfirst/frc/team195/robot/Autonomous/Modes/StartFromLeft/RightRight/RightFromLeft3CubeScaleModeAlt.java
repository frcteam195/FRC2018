package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromLeft.RightRight;

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
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.Left3CubeScale.*;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.Right3CubeScale.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class RightFromLeft3CubeScaleModeAlt extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new Right3CubeScaleStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
				new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
						AutomatedActions.PrepareShootCubeOverBack())))));

		runAction(AutomatedActions.OutakeCubeExtraFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCube(),
				new SetIntakeAction(IntakeControl.INTAKE_IN),
				new DrivePathAction(new Right3CubeScaleStep2()))));

		runAction(AutomatedActions.GrabCube());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PrepareShootCubeOverBackLowHighArm(),
				new DrivePathAction(new Right3CubeScaleStep3()))));

		runAction(AutomatedActions.OutakeCubeExtraFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCube(),
				new SetIntakeAction(IntakeControl.INTAKE_IN),
				new DrivePathAction(new Right3CubeScaleStep4()))));

		runAction(AutomatedActions.GrabCube());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PrepareShootCubeOverBackLowHighArm(),
				new DrivePathAction(new Right3CubeScaleStep5()))));

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

		runAction(AutomatedActions.SetRestingPosition());

		runAction(new WaitAction(15));
	}
}
