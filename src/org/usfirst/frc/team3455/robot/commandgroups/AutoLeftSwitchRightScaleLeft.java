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
public class AutoLeftSwitchRightScaleLeft extends CommandGroup { //START POSITION: CENTER || GOAL: SWITCH
	
    public AutoLeftSwitchRightScaleLeft() {
    	requires(Robot.chassis);
    	requires(Robot.elevator);
    	requires(Robot.intake);

    	addSequential(new DriveForwardAuto(1.75,0.5, true)); //move forward to switch from center position
		addSequential(new ManipulateTilt(500, -0.5)); //tilt intake down to switch
		addSequential(new ManipulateBlock(2500, -0.5)); //shoot block out
    }
}
