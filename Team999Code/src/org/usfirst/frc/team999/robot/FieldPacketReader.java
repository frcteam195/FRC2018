package org.usfirst.frc.team999.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class FieldPacketReader {
	private DriverStation ds = DriverStation.getInstance();
	private FieldLayout fieldLayout = FieldLayout.LEFT_LEFT;
	
	private static FieldPacketReader instance = null;
	
	private FieldPacketReader() {
		
	}
	
	public static FieldPacketReader getInstance() {
		if (instance == null)
			instance = new FieldPacketReader();
		
		return instance;
	}
	
	public FieldLayout getFieldConfiguration() {
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
		} catch (Exception ex) {
			ex.printStackTrace();
		}
		
		return fieldLayout;
	}
}
