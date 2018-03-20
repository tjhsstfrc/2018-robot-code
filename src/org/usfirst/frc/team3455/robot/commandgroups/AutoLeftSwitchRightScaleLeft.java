package org.usfirst.frc.team3455.robot.commandgroups;

import org.usfirst.frc.team3455.robot.Robot;
import org.usfirst.frc.team3455.robot.commands.*;
import org.usfirst.frc.team3455.robot.utils.Constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * 
 */
public class AutoLeftSwitchRightScaleLeft extends CommandGroup {
	
    public AutoLeftSwitchRightScaleLeft() {
    	requires(Robot.chassis);
    	requires(Robot.elevator);
    	requires(Robot.intake);
    	
    }
}
