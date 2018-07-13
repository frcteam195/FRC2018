package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromRight.Calculated;

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
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftFromRight3CubeScale.*;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ArmPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class LeftFromRight3CubeScaleModeCalculated extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new LeftFromRight3CubeScaleStep1();
		ConsoleReporter.report("Starting first path");
		runAction(new ResetPoseFromPathAction(pathContainer));

		ConsoleReporter.report("Placing first cube");
		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
				new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
						AutomatedActions.PreparePlaceCubeCalculatedScaleHeight(ElevatorPosition.OVER_THE_BACK_LOW, ArmPosition.BACK_SHOOT_LESS_HIGH))))));

		ConsoleReporter.report("Ejecting first cube");
		runAction(AutomatedActions.OutakeCubeExtraFast());
		runAction(AutomatedActions.StopIntake());

		ConsoleReporter.report("Getting second cube");
		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCube(),
				new SetIntakeAction(IntakeControl.INTAKE_IN),
				new DrivePathAction(new LeftFromRight3CubeScaleStep2()))));

		ConsoleReporter.report("Picking up second cube");
		runAction(AutomatedActions.GrabCube());

		ConsoleReporter.report("Placing second cube");
		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePlaceCubeCalculatedScaleHeight(ElevatorPosition.OVER_THE_BACK_LOW, ArmPosition.BACK_SHOOT_LESS_HIGH),
				new DrivePathAction(new LeftFromRight3CubeScaleStep3()))));

		ConsoleReporter.report("Ejecting second cube");
		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

		ConsoleReporter.report("Getting third cube");
		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCube(),
				new SetIntakeAction(IntakeControl.INTAKE_IN),
				new DrivePathAction(new LeftFromRight3CubeScaleStep4()))));

		ConsoleReporter.report("Picking up third cube");
		runAction(AutomatedActions.GrabCube());

		ConsoleReporter.report("Placing third cube");
		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePlaceCubeCalculatedScaleHeight(ElevatorPosition.OVER_THE_BACK_SHOOT_LOW, ArmPosition.BACK_SHOOT),
				new DrivePathAction(new LeftFromRight3CubeScaleStep5()))));

		ConsoleReporter.report("Ejecting third cube");
		runAction(AutomatedActions.OutakeCubeExtraFast());
		runAction(AutomatedActions.StopIntake());

		ConsoleReporter.report("Resting cube");
		runAction(AutomatedActions.SetRestingPosition());

		runAction(new WaitAction(15));
	}
}
