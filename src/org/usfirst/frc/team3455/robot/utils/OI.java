package org.usfirst.frc.team3455.robot.utils;

import org.usfirst.frc.team3455.robot.Robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {
	private static Joystick leftStick = new Joystick(0);
	private static Joystick rightStick = new Joystick(1);
	//private static Button buttonA = new JoystickButton(leftStick, 1);
	
	public OI(){
	}
	
	public static Joystick stickOutput(int stick){
		Robot.dashboardTable.putNumber("stick outputed: ", System.currentTimeMillis());
		if(stick == 0) return leftStick;
		if(stick == 1) return rightStick;
		else return null;
		
	}
}
