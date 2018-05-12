package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromRight.LeftFromRight;

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
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftFromRight3CubeScale.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class LeftFromRight3CubeScaleModeAlt extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new LeftFromRight3CubeScaleStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
				new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
						AutomatedActions.PrepareShootCubeOverBack())))));

		runAction(AutomatedActions.OutakeCubeExtraFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCube(),
				new SetIntakeAction(IntakeControl.INTAKE_IN),
				new DrivePathAction(new LeftFromRight3CubeScaleStep2()))));

		runAction(AutomatedActions.GrabCube());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PrepareShootCubeOverBack(),
				new DrivePathAction(new LeftFromRight3CubeScaleStep3()))));

		runAction(new WaitAction(0.150));
		runAction(AutomatedActions.OutakeCubeExtraFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCube(),
				new SetIntakeAction(IntakeControl.INTAKE_IN),
				new DrivePathAction(new LeftFromRight3CubeScaleStep4()))));

		runAction(AutomatedActions.GrabCube());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PrepareShootCubeOverBack(),
				new DrivePathAction(new LeftFromRight3CubeScaleStep5()))));

		runAction(AutomatedActions.OutakeCubeExtraFast());
		runAction(AutomatedActions.StopIntake());

		runAction(AutomatedActions.SetRestingPosition());

		runAction(new WaitAction(15));
	}
}
