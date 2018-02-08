package org.usfirst.frc.team195.robot.Utilities;

import org.usfirst.frc.team195.robot.Utilities.Loops.Looper;

public interface CustomSubsystem {
	public void init();
	public void subsystemHome();
	public void registerEnabledLoops(Looper in);
	public void terminate();
}
