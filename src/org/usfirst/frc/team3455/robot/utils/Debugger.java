package org.usfirst.frc.team3455.robot.utils;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Debugger {
	
	private static boolean DEBUG = false;
	public static NetworkTable debugTable = NetworkTable.getTable("SmartDashboard");
	
	public static void setDebugMode(boolean a) {
		DEBUG = a;
	}
	
	public static boolean getDebugMode() {
		return DEBUG;
	}
	
	public static void debug(String key, Object value) {
		if(DEBUG)
			debugTable.putString("DEBUG-" + key, String.valueOf(value));
	}
}
