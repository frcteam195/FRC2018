package org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromRight.LeftLeft;

import org.usfirst.frc.team195.robot.Actions.DrivePathAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftLeft_3cube.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

public class LeftLeftFromRightMode_3cube extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer pathContainer = new LeftLeftFromRightStep1();
		runAction(new ResetPoseFromPathAction(pathContainer));
		runAction(new DrivePathAction(pathContainer));



		runAction(new DrivePathAction(new LeftLeftFromRightStep2()));
		runAction(new DrivePathAction(new LeftLeftFromRightStep3()));
		runAction(new DrivePathAction(new LeftLeftFromRightStep4()));
		runAction(new DrivePathAction(new LeftLeftFromRightStep5Final()));
		runAction(new WaitAction(15));
	}
}