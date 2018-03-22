package com.team254.frc2017.auto.modes;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.frc2017.paths.CubeToScaleLeft;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.ScaleCloseIn;
import com.team254.frc2017.paths.StartToCubeLeft;

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
