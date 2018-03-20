package org.usfirst.frc.team195.robot.Autonomous;


import org.usfirst.frc.team195.robot.Actions.DrivePathAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team195.robot.Autonomous.Paths.CubeToScaleLeft;
import org.usfirst.frc.team195.robot.Autonomous.Paths.ScaleCloseIn;
import org.usfirst.frc.team195.robot.Autonomous.Paths.StartToCubeLeft;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

/**
 * Scores the preload gear onto the center peg then shoots the 10 preloaded fuel
 *
 * @see AutoModeBase
 */
public class SwitchCubeThenScaleMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer cubePath = new StartToCubeLeft();
		runAction(new ResetPoseFromPathAction(cubePath));
		runAction(new DrivePathAction(cubePath));
		runAction(new DrivePathAction(new CubeToScaleLeft()));
		runAction(new DrivePathAction(new ScaleCloseIn()));
		runAction(new WaitAction(15));
	}
}
