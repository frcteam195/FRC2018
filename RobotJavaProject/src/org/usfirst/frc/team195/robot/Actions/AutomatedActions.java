package org.usfirst.frc.team195.robot.Actions;

import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetArmRotationAction;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetElevatorHeightAction;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetIntakeAction;
import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.Actions.Framework.ParallelAction;
import org.usfirst.frc.team195.robot.Actions.Framework.SeriesAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;

import java.util.ArrayList;
import java.util.Arrays;

public class AutomatedActions {
	public static SeriesAction PlaceCubeOnScaleOverBack() {
		ArrayList<Action> actionArrayList = new ArrayList<Action>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetArmRotationAction(45), new SetElevatorHeightAction(ElevatorPosition.HIGH))));
		actionArrayList.add(new SeriesAction(Arrays.asList(new WaitAction(0.25), new SetIntakeAction(IntakeControl.INTAKE_OUT))));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetArmRotationAction(0), new SetElevatorHeightAction(ElevatorPosition.HOME))));

		return new SeriesAction(actionArrayList);
	}
}
