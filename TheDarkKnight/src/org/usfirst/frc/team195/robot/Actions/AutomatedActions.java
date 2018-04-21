package org.usfirst.frc.team195.robot.Actions;

import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.*;
import org.usfirst.frc.team195.robot.Actions.Framework.*;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ArmPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

public class AutomatedActions {
	public static SeriesAction PlaceCubeOnScaleOverBack() {
		ArrayList<Action> actionArrayList = new ArrayList<Action>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK), new SetElevatorHeightAction(ElevatorPosition.MID))));
		actionArrayList.add(new SeriesAction(Arrays.asList(new WaitAction(0.25), new SetIntakeAction(IntakeControl.INTAKE_OUT, 0.2))));
		actionArrayList.add(new SetIntakeAction(IntakeControl.OFF));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetArmRotationAction(0), new SeriesAction(Arrays.asList(new WaitAction(0.2), new SetElevatorHeightAction(ElevatorPosition.GO_DOWN))))));

		return new SeriesAction(actionArrayList);
	}

	public static SeriesAction PlaceCubeOnSwitchArmOnly() {
		ArrayList<Action> actionArrayList = new ArrayList<Action>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.SWITCH),
															 new SetElevatorHeightAction(ElevatorPosition.GO_DOWN))));
		actionArrayList.add(new SeriesAction(Arrays.asList(new WaitAction(0.1),
														   new SetIntakeAction(IntakeControl.INTAKE_OUT, 0.2))));
		actionArrayList.add(new SetIntakeAction(IntakeControl.OFF));
		actionArrayList.add(new SetArmRotationAction(0));

		return new SeriesAction(actionArrayList);
	}

	public static ParallelAction PreparePlaceCubeOnSwitch() {
		return new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPosition.SWITCH),
												new SetArmRotationAction(ArmPosition.LOW)));
	}

	public static ParallelAction PreparePlaceCubeOnScaleOverBack() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK),
												new SetElevatorHeightAction(ElevatorPosition.OVER_THE_BACK_MID)));
	}

	public static ParallelAction PreparePlaceCubeOnSwitchElevator() {
		return new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPosition.SWITCH),
												new SetArmRotationAction(ArmPosition.LOW)));
	}

	public static ParallelAction SetRestingPosition() {
		return new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPosition.GO_DOWN),
												new SetArmRotationAction(ArmPosition.VERTICAL)));
	}

	public static ParallelAction PreparePlaceCubeOnScaleHigh() {
		return new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPosition.HIGH),
												new SetArmRotationAction(ArmPosition.DOWN)));
	}

	public static ParallelAction PreparePlaceCubeOnScaleMid() {
		return new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPosition.MID),
												new SetArmRotationAction(ArmPosition.DOWN)));
	}

	public static ParallelAction PreparePlaceCubeOnScaleLow() {
		return new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPosition.LOW),
				new SetArmRotationAction(ArmPosition.DOWN)));
	}

	public static ParallelAction PreparePlaceCubeOnScaleOverBackLow() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK),
												new SetElevatorHeightAction(ElevatorPosition.OVER_THE_BACK_LOW)));
	}

	public static ParallelAction PrepareShootCubeOverBack() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK_SHOOT),
												new SetElevatorHeightAction(ElevatorPosition.OVER_THE_BACK_LOW)));
	}

	public static ParallelAction PrepareShootCubeOverBackLow() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK_SHOOT),
												new SetElevatorHeightAction(ElevatorPosition.OVER_THE_BACK_SHOOT_LOW)));
	}

	public static ParallelAction PrepareShootCubeOverBackExtraLow() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK_SHOOT),
				new SetElevatorHeightAction(ElevatorPosition.OVER_THE_BACK_SHOOT_EXTRA_LOW)));
	}

	public static ParallelAction PrepareShootCubeOverBackLowHighArm() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK_SHOOT_HIGH),
												new SetElevatorHeightAction(ElevatorPosition.OVER_THE_BACK_SHOOT_LOW)));
	}

	public static ParallelAction PrepareShootCubeOverBackLowLessHighArm() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK_SHOOT_LESS_HIGH),
				new SetElevatorHeightAction(ElevatorPosition.OVER_THE_BACK_SHOOT_LOW)));
	}

	public static ParallelAction PrepareShootCubeOverBackHighArm() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK_SHOOT_HIGH),
												new SetElevatorHeightAction(ElevatorPosition.OVER_THE_BACK_LOW)));
	}

	public static ParallelAction PreparePlaceCubeOnScaleOverBackMid() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK),
												new SetElevatorHeightAction(ElevatorPosition.OVER_THE_BACK_MID)));
	}

	public static ParallelAction PreparePlaceCubeOnScaleOverBackKindaMid() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK),
												new SetElevatorHeightAction(ElevatorPosition.OVER_THE_BACK_KINDA_MID)));
	}

	public static ParallelAction PreparePlaceCubeOnScaleOverBackHigh() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.BACK),
												new SetElevatorHeightAction(ElevatorPosition.OVER_THE_BACK_HIGH)));
	}

	public static ParallelAction PreparePlaceCubeOnScaleShortHigh() {
		return new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPosition.SHORT_HIGH),
												new SetArmRotationAction(ArmPosition.SWITCH)));
	}

	public static Action PreparePlaceCubeOnSwitchArm() {
		return new SetArmRotationAction(ArmPosition.SWITCH);
	}

	public static Action PreparePickupCube() {
		return new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.VERTICAL),
												new SetIntakeClampAction(true),
												new SetElevatorHeightAction(ElevatorPosition.GO_DOWN),
												new SeriesAction(Arrays.asList(new WaitForElevatorDownAction(ElevatorPosition.BALL_LIKE_558),
																			   new SetArmRotationAction(ArmPosition.DOWN)))));
	}

	public static SeriesAction PreparePickupCubeSlow() {
//		return new SeriesAction(Arrays.asList(new SetArmRotationAction(ArmPosition.VERTICAL),
//											  new SetElevatorHeightAction(ElevatorPosition.HOME),
//											  new SetArmRotationAction(ArmPosition.DOWN)));

		return new SeriesAction(Arrays.asList(new ParallelAction(Arrays.asList(new SetArmRotationAction(ArmPosition.VERTICAL),
																			   new SetElevatorHeightAction(ElevatorPosition.HOME))),
											  new SetIntakeClampAction(true),
											  new SetArmRotationAction(ArmPosition.DOWN)));
	}

	public static ParallelAction PreparePlaceCubeOnSwitchOverBack() {
		return new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPosition.LOW),
												new SetArmRotationAction(ArmPosition.BACK)));
	}

	public static ParallelAction PreparePlaceCubeOnSwitchOverBackLow() {
		return new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPosition.OVER_BACK_SWITCH),
												new SetArmRotationAction(ArmPosition.BACK_SHOOT)));
	}

	public static Action LiftArmTo90() {
		return new SetArmRotationAction(ArmPosition.VERTICAL);
	}

	public static Action ElevetorTo0() {
		return new SetElevatorHeightAction(ElevatorPosition.GO_DOWN);
	}

	public static Action ClampIntake() {
		return new SetIntakeClampAction(false);
	}

	public static Action StopIntake() {
		return new SetIntakeAction(IntakeControl.OFF);
	}

	public static SeriesAction GrabCube() {
		return new SeriesAction(Arrays.asList(AutomatedActions.ClampIntake(),
											  new WaitAction(0.1),
											  AutomatedActions.StopIntake()));
	}

	public static Action OutakeCubeSlow() {
		return new SetIntakeAction(IntakeControl.INTAKE_OUT_SLOW, 0.25);
	}

	public static Action OutakeCubeMidSpeed() {
		return new SetIntakeAction(IntakeControl.INTAKE_OUT_HALFSPEED, 0.25);
	}

	public static Action OutakeCubeFast() {
		return new SetIntakeAction(IntakeControl.INTAKE_OUT_AUTO, 0.25);
	}

	public static Action OutakeCubeExtraFast() {
		return new SetIntakeAction(IntakeControl.INTAKE_OUT_EXTRA_FAST, 0.3);
	}

	public static Action OutakeCubeALittleLessFast() {
		return new SetIntakeAction(IntakeControl.INTAKE_OUT, 0.25);
	}

}
