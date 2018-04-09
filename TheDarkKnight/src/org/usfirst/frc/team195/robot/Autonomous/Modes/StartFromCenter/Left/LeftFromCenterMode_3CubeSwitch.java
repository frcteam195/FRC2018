package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromCenter.Left;

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
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromCenter.LeftFromCenter3CubeSwitch.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class LeftFromCenterMode_3CubeSwitch extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new LeftFromCenter3CubeSwitchStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
				new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
						AutomatedActions.PreparePlaceCubeOnSwitchOverBackLow())))));

		runAction(AutomatedActions.OutakeCubeMidSpeed());
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCubeSlow(),
				new SetIntakeAction(IntakeControl.INTAKE_IN),
				new DrivePathAction(new LeftFromCenter3CubeSwitchStep2()))));

		runAction(new SeriesAction(Arrays.asList(AutomatedActions.ClampIntake(),
				new WaitAction(0.4))));

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.StopIntake(),
				AutomatedActions.PreparePlaceCubeOnSwitchOverBackLow(),
				new DrivePathAction(new LeftFromCenter3CubeSwitchStep3()))));

		runAction(AutomatedActions.OutakeCubeMidSpeed());
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCubeSlow(),
				new SetIntakeAction(IntakeControl.INTAKE_IN),
				new DrivePathAction(new LeftFromCenter3CubeSwitchStep4()))));

		runAction(new SeriesAction(Arrays.asList(AutomatedActions.ClampIntake(),
				new WaitAction(0.4))));

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.StopIntake(),
				AutomatedActions.PreparePlaceCubeOnSwitchOverBackLow(),
				new DrivePathAction(new LeftFromCenter3CubeSwitchStep5()))));

		runAction(AutomatedActions.OutakeCubeMidSpeed());
		runAction(AutomatedActions.StopIntake());

		runAction(AutomatedActions.SetRestingPosition());

		runAction(new WaitAction(15));
	}
}
