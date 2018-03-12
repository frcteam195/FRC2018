package org.usfirst.frc.team195.robot.Utilities;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;

public class GameSpecificMessageParser {
	private static GameSpecificMessageParser instance = null;
	private DriverStation ds;
	private boolean disableAuto = false;
	private boolean valuesUpdated = false;
	private FieldLayout fieldLayout;

	private GameSpecificMessageParser() {
		ds = DriverStation.getInstance();
	}

	public static GameSpecificMessageParser getInstance() {
		try {
			instance = new GameSpecificMessageParser();
		} catch (Exception ex) {
			ConsoleReporter.report(ex, MessageLevel.DEFCON1);
		}

		return instance;
	}

	public boolean isAutoDisabled() {
		if (!valuesUpdated)
			updateValues();
		return disableAuto;
	}

	public FieldLayout getTargetFieldLayout() {
		if (!valuesUpdated || fieldLayout == null)
			updateValues();
		if (fieldLayout != null)
			return fieldLayout;
		else
			return FieldLayout.UNDEFINED;
	}

	private synchronized void updateValues() {
		try {
			String s = ds.getGameSpecificMessage().toUpperCase();
			if (s.length() < 2)
				throw new Exception("Failed to get game specific data! No auto being used!");

			s = s.substring(0, 2);
			switch (s) {
				case "LL":
					fieldLayout = FieldLayout.LEFT_LEFT;
					break;
				case "LR":
					fieldLayout = FieldLayout.LEFT_RIGHT;
					break;
				case "RL":
					fieldLayout = FieldLayout.RIGHT_LEFT;
					break;
				case "RR":
					fieldLayout = FieldLayout.RIGHT_RIGHT;
					break;
				default:
					fieldLayout = FieldLayout.UNDEFINED;
					throw new Exception("Failed to parse game specific data! No auto being used!");
			}

			valuesUpdated = true;
		} catch (Exception ex) {
			disableAuto = true;
			ConsoleReporter.report(ex, MessageLevel.DEFCON1);
		}
	}
}
