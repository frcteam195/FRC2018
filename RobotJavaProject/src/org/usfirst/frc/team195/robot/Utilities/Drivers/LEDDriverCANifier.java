package org.usfirst.frc.team195.robot.Utilities.Drivers;

import com.ctre.phoenix.CANifier;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.RGBColor;

public class LEDDriverCANifier implements LEDDriver {
	private CANifier canifier;
	private RGBColor rgbColor = Constants.kDefaultColor;
	private RGBColor mPrevRGBColor = new RGBColor(0,0,0);
	private boolean on = false;

	public LEDDriverCANifier(CANifier canifier) {
		this.canifier = canifier;
		set(false);
	}

	@Override
	public synchronized void set(boolean on) {
		if (this.on != on || !rgbColor.equals(mPrevRGBColor)) {

			if (on) {
				canifier.setLEDOutput(rgbColor.red / 255.0, CANifier.LEDChannel.LEDChannelA);
				canifier.setLEDOutput(rgbColor.green / 255.0, CANifier.LEDChannel.LEDChannelB);
				canifier.setLEDOutput(rgbColor.blue / 255.0, CANifier.LEDChannel.LEDChannelC);
			} else {
				canifier.setLEDOutput(0, CANifier.LEDChannel.LEDChannelA);
				canifier.setLEDOutput(0, CANifier.LEDChannel.LEDChannelB);
				canifier.setLEDOutput(0, CANifier.LEDChannel.LEDChannelC);
			}

			mPrevRGBColor = rgbColor;
			this.on = on;
		}
	}

	@Override
	public synchronized void setLEDColor(RGBColor rgbColor) {
		this.rgbColor = rgbColor;
		set(on);
	}
}
