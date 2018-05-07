package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromLeft.LeftLeft;

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
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.LeftLeft_3cube.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class LeftLeftFromLeftMode_3cube extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new LeftLeftFromLeftStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));
		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
				new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"), AutomatedActions.PreparePlaceCubeOnScaleOverBack())))));
		runAction(AutomatedActions.OutakeCubeFast());
		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCube(),
				new DrivePathAction(new LeftLeftFromLeftStep2()),
				new SetIntakeAction(IntakeControl.INTAKE_IN))));
		runAction(new SeriesAction(Arrays.asList(AutomatedActions.ClampIntake(),
				new WaitAction(0.1),
				AutomatedActions.StopIntake())));
		runAction(new SetElevatorHeightAction(ElevatorPosition.LOW));
		runAction(new DrivePathAction(new LeftLeftFromLeftStep3()));
		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());
		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new LeftLeftFromLeftStep4()), AutomatedActions.ElevetorTo0())));
		//runAction(new DrivePathAction(new RightRightFromRightStep4()));
		//runAction(new DrivePathAction(new RightRightFromRightStep5()));
		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new LeftLeftFromLeftStep5()),
				AutomatedActions.PreparePickupCube(), new SetIntakeAction(IntakeControl.INTAKE_IN))));
		runAction(AutomatedActions.ClampIntake());
		runAction(new WaitAction(0.1));
		runAction(AutomatedActions.StopIntake());
		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new LeftLeftFromLeftStep6Final()),
				AutomatedActions.LiftArmTo90(),
				new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"), AutomatedActions.PreparePlaceCubeOnScaleOverBack())))));
		runAction(AutomatedActions.OutakeCubeMidSpeed());
		runAction(AutomatedActions.StopIntake());

		runAction(new WaitAction(15));
	}
}
