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
    	
    	addParallel(new MoveElevatorToMaxPosition(1.0));
    	addSequential(new DriveForwardEncoderAuto(12*Constants.DIST_TO_SCALE, 0.5)); //drive to the scale
    	addSequential(new Sleep(250));
    	addSequential(new TurnPointAuto(25, 0.35)); //turn toward the scale //TODO get the actual value for this
    	addSequential(new Sleep(250));
    	addSequential(new ManipulateTilt(500, -0.5)); //tilt intake down to scale
    	addSequential(new Sleep(250));
    	addSequential(new ManipulateBlock(2500, -0.5)); //shoot out block
    	addParallel(new MoveElevatorToSwitchLevel(3000, -0.75));
    	addSequential(new Sleep(250));
    	addSequential(new TurnPointAuto(-25, 0.35)); //TODO get the actual value for this
    	addSequential(new Sleep(250));
    	addSequential(new DriveForwardEncoderAuto(12*Constants.DIST_TO_SCALE - 12*Constants.DIST_TO_CHANNEL, -0.5)); //drive backward to the channel
    	addSequential(new Sleep(250));
    	addSequential(new TurnPointAuto(90, 0.35));
    	addSequential(new Sleep(250));
    	addSequential(new DriveForwardEncoderAuto(12*Constants.DIST_THROUGH_CHANNEL, 0.5)); //drive through the channel
    	addSequential(new Sleep(250));
    	addSequential(new TurnPointAuto(90, 0.35)); 
    	addSequential(new Sleep(250));
    	addSequential(new DriveForwardEncoderAuto(12*0.5, 0.35)); //TODO get the actual value for this
    	//TODO we still have to pick up another block
    }
}
