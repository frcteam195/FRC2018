package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromRight.RightLeft;

import org.usfirst.frc.team195.robot.Actions.AutomatedActions;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetArmRotationAction;
import org.usfirst.frc.team195.robot.Actions.CubeHandlerActions.SetIntakeAction;
import org.usfirst.frc.team195.robot.Actions.DrivePathAction;
import org.usfirst.frc.team195.robot.Actions.Framework.ParallelAction;
import org.usfirst.frc.team195.robot.Actions.Framework.SeriesAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitForPathMarkerAction;
import org.usfirst.frc.team195.robot.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightLeft_2cube.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ArmPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.lang.reflect.Array;
import java.util.Arrays;

public class RightLeftFromRightMode_2cube extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new RightLeftFromRightStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));
		//runAction(new DrivePathAction(pathContainer));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(pathContainer),
												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
													   AutomatedActions.PreparePlaceCubeOnScaleOverBackLow())))));

		runAction(AutomatedActions.OutakeCubeMidSpeed());
		runAction(AutomatedActions.StopIntake());

		//runAction(new DrivePathAction(new RightLeftFromRightStep2()));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightLeftFromRightStep2()),
								                   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePickupCube"),
									                   AutomatedActions.PreparePickupCube())),
												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("StartIntake"),
													   new SetIntakeAction(IntakeControl.INTAKE_IN))))));

		runAction(new WaitAction(0.2));
		runAction(AutomatedActions.ClampIntake());
		runAction(new WaitAction(0.2));
		runAction(AutomatedActions.StopIntake());

		//runAction(AutomatedActions.LiftArmTo90());
		//runAction(new WaitAction(0.2));

		//runAction(new DrivePathAction(new RightLeftFromRightStep3()));

		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightLeftFromRightStep3()),
												   AutomatedActions.PreparePlaceCubeOnScaleOverBackHigh())));

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

//		runAction(new DrivePathAction(new RightLeftFromRightStep2()));
//		runAction(new DrivePathAction(new RightLeftFromRightStep3()));
//		runAction(new DrivePathAction(new RightLeftFromRightStep4()));
//		runAction(new DrivePathAction(new RightLeftFromRightStep5Final()));
//
//		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightLeftFromRightStep2()),
//												   AutomatedActions.PreparePlaceCubeOnSwitchArm())));
//
//		runAction(AutomatedActions.OutakeCubeMidSpeed());
//		runAction(new WaitAction(1));
//		runAction(AutomatedActions.StopIntake());
//
//		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightLeftFromRightStep3()),
//				  AutomatedActions.LiftArmTo90())));
//
//		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightLeftFromRightStep4()),
//				  new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("LowerArm"),
//                      AutomatedActions.PreparePickupCube(),
//				      new SetIntakeAction(IntakeControl.INTAKE_IN))))));
//
//		runAction(AutomatedActions.ClampIntake());
//		runAction(new WaitAction(0.1));
//		runAction(AutomatedActions.StopIntake());
//
//		runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RightLeftFromRightStep5Final()),
//												   AutomatedActions.LiftArmTo90(),
//												   new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PreparePlaceCube"),
//												       AutomatedActions.PreparePlaceCubeOnScaleOverBack())))));
//
//		runAction(AutomatedActions.OutakeCubeFast());
//		runAction(new WaitAction(0.2));
//		runAction(AutomatedActions.StopIntake());

		runAction(new WaitAction(15));
	}
}
