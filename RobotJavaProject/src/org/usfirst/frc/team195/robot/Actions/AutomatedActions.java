package org.usfirst.frc.team195.robot.Actions;

import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetArmRotationAction;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetElevatorHeightAction;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetIntakeAction;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetIntakeClampAction;
import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.Actions.Framework.ParallelAction;
import org.usfirst.frc.team195.robot.Actions.Framework.SeriesAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ArmPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;

import java.util.ArrayList;
import java.util.Arrays;

public class AutomatedActions {
	public static SeriesAction PlaceCubeOnScaleOverBack() {
		ArrayList<Action> actionArrayList = new ArrayList<Action>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK), new SetElevatorHeightAction(ElevatorPosition.MID))));
		actionArrayList.add(new SeriesAction(Arrays.asList(new WaitAction(0.25), new SetIntakeAction(IntakeControl.INTAKE_OUT, 0.2))));
		actionArrayList.add(new SetIntakeAction(IntakeControl.OFF));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetArmRotationAction(0), new SeriesAction(Arrays.asList(new WaitAction(0.2), new SetElevatorHeightAction(ElevatorPosition.HOME))))));

		return new SeriesAction(actionArrayList);
	}

	public static SeriesAction PlaceCubeOnSwitchArmOnly() {
		ArrayList<Action> actionArrayList = new ArrayList<Action>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.SWITCH), new SetElevatorHeightAction(ElevatorPosition.HOME))));
		actionArrayList.add(new SeriesAction(Arrays.asList(new WaitAction(0.1), new SetIntakeAction(IntakeControl.INTAKE_OUT, 0.2))));
		actionArrayList.add(new SetIntakeAction(IntakeControl.OFF));
		actionArrayList.add(new SetArmRotationAction(0));

		return new SeriesAction(actionArrayList);
	}

	public static ParallelAction PreparePlaceCubeOnScaleOverBack() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK), new SetElevatorHeightAction(ElevatorPosition.OVER_THE_BACK_HIGH)));
	}

	public static Action PreparePlaceCubeOnSwitch() {
		return new SetArmRotationAction(ArmPosition.SWITCH);
	}

	public static ParallelAction PreparePickupCube() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(0), new SetIntakeClampAction(true), new SetElevatorHeightAction(ElevatorPosition.HOME)));
	}

	public static Action LiftArmTo90() {
		return new SetArmRotationAction(ArmPosition.VERTICAL);
	}

	public static Action ElevetorTo0() {
		return new SetElevatorHeightAction(ElevatorPosition.HOME);
	}

	public static Action ClampIntake() {
		return new SetIntakeClampAction(false);
	}

	public static Action StopIntake() {
		return new SetIntakeAction(IntakeControl.OFF);
	}

	public static Action OutakeCubeSlow() {
		return new SetIntakeAction(IntakeControl.INTAKE_OUT_SLOW, 0.2);
	}

	public static Action OutakeCubeMidSpeed() {
		return new SetIntakeAction(IntakeControl.INTAKE_OUT_HALFSPEED, 0.2);
	}

	public static Action OutakeCubeFast() {
		return new SetIntakeAction(IntakeControl.INTAKE_OUT, 0.2);
	}

}
