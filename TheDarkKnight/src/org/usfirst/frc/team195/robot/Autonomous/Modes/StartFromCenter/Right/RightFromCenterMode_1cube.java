package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromCenter.Right;

import org.usfirst.frc.team195.robot.Actions.AutomatedActions;
import org.usfirst.frc.team195.robot.Actions.DrivePathAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromCenter.Right2Cube.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

public class RightFromCenterMode_1cube extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new RightFromCenterStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));

		runAction(new DrivePathAction(pathContainer));

		runAction(AutomatedActions.PreparePlaceCubeOnSwitchElevator());

		runAction(AutomatedActions.OutakeCubeFast());
		runAction(AutomatedActions.StopIntake());

		runAction(new WaitAction(15));
	}
}
