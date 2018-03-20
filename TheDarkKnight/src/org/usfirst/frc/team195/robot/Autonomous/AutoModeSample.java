package org.usfirst.frc.team195.robot.Autonomous;

import org.usfirst.frc.team195.robot.Actions.DrivePathAction;
import org.usfirst.frc.team195.robot.Actions.Framework.WaitAction;
import org.usfirst.frc.team195.robot.Actions.ResetPoseFromPathAction;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeEndedException;
import org.usfirst.frc.team195.robot.Autonomous.Paths.SamplePath;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

/**
 * Scores the preload gear onto the center peg then shoots the 10 preloaded fuel
 *
 * @see AutoModeBase
 */
public class AutoModeSample extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer samplePath = new SamplePath();
		runAction(new ResetPoseFromPathAction(samplePath));
		runAction(new DrivePathAction(samplePath));
		runAction(new WaitAction(15));
	}
}