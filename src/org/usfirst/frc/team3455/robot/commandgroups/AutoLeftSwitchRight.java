package org.usfirst.frc.team3455.robot.commandgroups;

import org.usfirst.frc.team3455.robot.Robot;
import org.usfirst.frc.team3455.robot.commands.*;
import org.usfirst.frc.team3455.robot.utils.Constants;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * 
 */
public class AutoLeftSwitchRight extends CommandGroup {
	
    public AutoLeftSwitchRight() {
    	requires(Robot.chassis);
    	requires(Robot.elevator);
    	requires(Robot.intake);

    	//moving through channel
    	addSequential(new DriveForwardEncoderAuto(12*Constants.DIST_TO_CHANNEL, 0.5)); //drive to the channel
    	addSequential(new Sleep(250));
    	addSequential(new TurnPointAuto(90, 0.35));
    	addSequential(new Sleep(250));
    	addSequential(new DriveForwardEncoderAuto(12*Constants.DIST_THROUGH_CHANNEL, 0.5)); //drive through the channel
    	addSequential(new Sleep(250));
    	addSequential(new TurnPointAuto(90, 0.35));
    	addSequential(new Sleep(250));
    	addSequential(new DriveForwardEncoderAuto(12*0.5, 0.35)); //TODO get the actual value for this
    	addSequential(new ManipulateTilt(500, -0.5)); //tilt intake down to switch
    	addSequential(new ManipulateBlock(2500, -0.5)); //shoot out block
    	
    	
    }
}
