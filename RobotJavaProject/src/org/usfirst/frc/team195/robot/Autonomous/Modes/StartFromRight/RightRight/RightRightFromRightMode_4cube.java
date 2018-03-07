package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromRight.RightRight;

import org.usfirst.frc.team195.robot.Actions.DrivePathAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_4cube.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

public class RightRightFromRightMode_4cube extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new RightRightFromRightStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));
		runAction(new DrivePathAction(pathContainer));
		runAction(new DrivePathAction(new RightRightFromRightStep2()));
		runAction(new DrivePathAction(new RightRightFromRightStep3()));
		runAction(new DrivePathAction(new RightRightFromRightStep4()));
		runAction(new DrivePathAction(new RightRightFromRightStep5()));
		runAction(new DrivePathAction(new RightRightFromRightStep6()));
		runAction(new DrivePathAction(new RightRightFromRightStep7Final()));
		runAction(new WaitAction(15));
	}
}
