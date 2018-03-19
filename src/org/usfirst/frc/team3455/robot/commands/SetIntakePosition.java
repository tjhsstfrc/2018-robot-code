package org.usfirst.frc.team3455.robot.commands;

import org.usfirst.frc.team3455.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetIntakePosition extends Command {

	private boolean valueSet;
	private double value;
    
    public SetIntakePosition(double pos) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intake);
    	this.value = pos;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	this.valueSet = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intake.position = this.value;
    	this.valueSet = true;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return this.valueSet;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
