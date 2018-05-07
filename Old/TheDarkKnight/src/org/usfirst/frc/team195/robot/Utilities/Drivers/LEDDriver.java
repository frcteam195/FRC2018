package org.usfirst.frc.team195.robot.Utilities.Drivers;

import org.usfirst.frc.team195.robot.Utilities.RGBColor;

public interface LEDDriver {
	void set(boolean on);
	void setLEDColor(RGBColor rgbColor);
}
