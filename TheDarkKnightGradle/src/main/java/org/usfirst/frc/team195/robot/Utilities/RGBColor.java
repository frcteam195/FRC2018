package org.usfirst.frc.team195.robot.Utilities;

public class RGBColor {
	public int red;
	public int green;
	public int blue;

	public RGBColor(int red, int green, int blue) {
		this.red = red;
		this.green = green;
		this.blue = blue;
	}

	public boolean equals(RGBColor o) {
		if (o.red == red && o.green == green && o.blue == blue)
			return true;
		else
			return false;
	}
}
