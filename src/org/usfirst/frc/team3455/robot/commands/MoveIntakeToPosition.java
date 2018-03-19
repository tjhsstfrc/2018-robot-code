package org.usfirst.frc.team3455.robot.commands;

import org.usfirst.frc.team3455.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MoveIntakeToPosition extends Command {

    
    public MoveIntakeToPosition() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.position = Robot.intake.getTiltEncoder();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.intake.getTiltEncoder() > Robot.intake.position){
			Robot.intake.tiltPower(-0.1);
		}else if(Robot.intake.getTiltEncoder() < Robot.intake.position){
			Robot.intake.tiltPower(0.1);
		}else{
			Robot.intake.tiltPower(0);
		}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !DriverStation.getInstance().isAutonomous();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
