package org.usfirst.frc.team3455.robot.commandgroups;

import org.usfirst.frc.team3455.robot.Robot;
import org.usfirst.frc.team3455.robot.commands.DriveForwardAuto;
import org.usfirst.frc.team3455.robot.commands.ManipulateBlock;
import org.usfirst.frc.team3455.robot.commands.ManipulateTilt;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * 
 */
public class AutoRightSwitchLeftScaleRight extends CommandGroup {
	
    public AutoRightSwitchLeftScaleRight() {
    	requires(Robot.chassis);
    	requires(Robot.elevator);
    	requires(Robot.intake);

    }
}
