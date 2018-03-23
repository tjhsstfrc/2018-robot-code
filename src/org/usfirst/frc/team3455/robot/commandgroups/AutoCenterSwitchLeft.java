package org.usfirst.frc.team3455.robot.commandgroups;

import org.usfirst.frc.team3455.robot.Robot;
import org.usfirst.frc.team3455.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * 
 */
public class AutoCenterSwitchLeft extends CommandGroup {
	
    public AutoCenterSwitchLeft() {
    	requires(Robot.chassis);
    	requires(Robot.elevator);
    	requires(Robot.intake);
    	
    	addSequential(new DriveForwardEncoderAuto(12*.5, .35)); //drive forward 0.5 feet //TODO get the actual value for this
    	addSequential(new Sleep(250));
    	addSequential(new TurnPointAuto(-45, .35)); //turn 45 degrees to the left
    	addSequential(new Sleep(250));
    	addSequential(new DriveForwardEncoderAuto(12*.5, .35)); //drive forward 0.5 feet //TODO get the actual value for this
    	addSequential(new Sleep(250));
    	addSequential(new TurnPointAuto(45, .35)); //turn 45 degrees to the right
    	addSequential(new Sleep(250));
    	addSequential(new ManipulateTilt(500, -0.5)); //tilt intake down to switch
    	addSequential(new DriveForwardEncoderAuto(12*.5, .35)); //drive forward 0.5 feet to the switch //TODO get the actual value for this
    	addSequential(new Sleep(250));
    	addSequential(new ManipulateBlock(2500, -0.5)); //shoot out block
    	
    }
}
