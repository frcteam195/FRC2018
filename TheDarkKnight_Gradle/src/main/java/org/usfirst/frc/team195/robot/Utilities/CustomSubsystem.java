package org.usfirst.frc.team195.robot.Utilities;

import org.usfirst.frc.team195.robot.Utilities.Loops.Looper;

public interface CustomSubsystem {
	void init();
	void subsystemHome();
	void registerEnabledLoops(Looper in);
}
