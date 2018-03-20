package org.usfirst.frc.team3455.robot.commandgroups;

import org.usfirst.frc.team3455.robot.commands.*;
import org.usfirst.frc.team3455.robot.utils.Constants;
import org.usfirst.frc.team3455.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * 
 */
public class AutoLeftScaleLeft extends CommandGroup {
	
    public AutoLeftScaleLeft() {
    	requires(Robot.chassis);
    	requires(Robot.elevator);
    	requires(Robot.intake);
    	
    	addParallel(new MoveElevatorToMaxPosition(1.0));
    	addSequential(new DriveForwardEncoderAuto(12*Constants.DIST_TO_SCALE, 0.5)); //drive to the scale
    	addSequential(new Sleep(250));
    	addSequential(new TurnPointAuto(25, 0.35)); //turn toward the scale
    	addSequential(new Sleep(250));
    	addParallel(new ManipulateTilt(500, -0.5)); //tilt intake down to scale
    	addSequential(new Sleep(250));
    	addSequential(new ManipulateBlock(2500, -0.5)); //shoot out block
    }
}
