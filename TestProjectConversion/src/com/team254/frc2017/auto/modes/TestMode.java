package com.team254.frc2017.auto.modes;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.TestPath;

/**
 * Scores the preload gear onto the center peg then shoots the 10 preloaded fuel
 *
 * @see AutoModeBase
 */
public class TestMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		PathContainer testPath = new TestPath();
		runAction(new ResetPoseFromPathAction(testPath));
		runAction(new DrivePathAction(testPath));
		runAction(new WaitAction(15));
	}
}
