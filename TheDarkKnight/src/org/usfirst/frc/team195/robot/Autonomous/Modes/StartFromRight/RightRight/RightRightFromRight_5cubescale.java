package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromRight.RightRight;

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
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_4cube.RightRightFromRightCube3Ready44;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_5cubescale.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class RightRightFromRight_5cubescale extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new RightRightFromRightStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));
		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
				new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"), AutomatedActions.PrepareShootCubeOverBackLow())))));

		runAction(AutomatedActions.OutakeCubeFast());


		//LaunchinCubes();
//		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
//												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("LiftElevator"),
//																				  new SetElevatorHeightAction(ElevatorPosition.BALL_LIKE_558),
//														   						  new WaitForPathMarkerAction("ChuteMe"),
//														   						  new SetIntakeAction(IntakeControl.INTAKE_OUT_EXTRA_FAST, 0.6))))));


		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCube(),
				new DrivePathAction(new RightRightFromRightStep2()),
				new SetIntakeAction(IntakeControl.INTAKE_IN))));

		runAction(new SeriesAction(Arrays.asList(AutomatedActions.ClampIntake(),
				new WaitAction(0.1),
				AutomatedActions.StopIntake())));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightRightFromRightGOBACK()),
				new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
						AutomatedActions.PrepareShootCubeOverBackLow())))));

		runAction(AutomatedActions.OutakeCubeExtraFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCube(),
				new SetIntakeAction(IntakeControl.INTAKE_IN),
				new DrivePathAction(new RightRightFromRightCube3()))));

		runAction(new SeriesAction(Arrays.asList(AutomatedActions.ClampIntake(),
				new WaitAction(0.1),
				AutomatedActions.StopIntake())));

//		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightRightFromRightCube3GOBACK()),
//									 new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
//											 						AutomatedActions.PreparePlaceCubeOnScaleOverBackMid())))));

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PrepareShootCubeOverBackLow(),
				new DrivePathAction(new RightRightFromRightCube3GOBACK()))));

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new ParallelAction(Arrays.asList(AutomatedActions.PreparePickupCube(),
				new DrivePathAction(new RightRightFromRightCube3Ready44()))));

		runAction(new WaitAction(15));
	}
}
