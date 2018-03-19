package org.usfirst.frc.team3455.robot.commands;

import org.usfirst.frc.team3455.robot.Robot;

import org.usfirst.frc.team3455.robot.utils.Debugger;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveForwardEncoderAuto extends Command {

	protected double drivePower;
	protected long startTimeRun;
	protected double encoderDistance;
	
	final double CORRECTION_MULTIPLIER = 0.025;
	final double LEFT_COEFF = 1.0;
	final double RIGHT_COEFF = 1.0;
	final double CORRECTION_ADD = 0.0;
	double autoStraightSpeed;// = -0.6; //-0.4
	double autoStraightSpeedPrecise = -0.4; //-0.4
	double leftSpeedAdj, rightSpeedAdj;
	double adjustVar;
	
	double offsetIMU = 0.0;
	boolean errorIMU = false;
	double startHeading = 0.0;
	final double IMU_THRESHOLD = 0.5;
	
    public DriveForwardEncoderAuto(double inches, double drivePower) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.chassis);
    	this.autoStraightSpeed = -drivePower;
    	this.encoderDistance = inches;
    	Robot.chassis.resetHeadingIMU();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
		offsetIMU = Robot.chassis.getHeadingIMU();
		startHeading = Robot.chassis.getHeadingIMU()-offsetIMU;
    	//Robot.chassis.tankDrivePower(drivePower, drivePower);
    	startTimeRun = System.currentTimeMillis();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Robot.dashboardTable.putNumber("ex chassis is encoder-drive: ", Robot.chassis.getRawEncoderValue());
	
    	leftSpeedAdj = autoStraightSpeed*LEFT_COEFF;
    	rightSpeedAdj = autoStraightSpeed*RIGHT_COEFF;
    	if(Robot.chassis.getHeadingIMU() - offsetIMU - startHeading > IMU_THRESHOLD){
    		adjustVar = Math.abs(Robot.chassis.getHeadingIMU() -offsetIMU - startHeading);
    		rightSpeedAdj -= CORRECTION_MULTIPLIER * adjustVar;
    		Robot.dashboardTable.putNumber("STRAIGHT LINE CORRECTION RIGHT: ", rightSpeedAdj);
    	}else if(Robot.chassis.getHeadingIMU() -offsetIMU - startHeading < -IMU_THRESHOLD){
    		adjustVar = Math.abs(Robot.chassis.getHeadingIMU() -offsetIMU - startHeading);
    		leftSpeedAdj -= CORRECTION_MULTIPLIER * adjustVar;
    		Debugger.debug("STRAIGHT LINE CORRECTION LEFT: ", leftSpeedAdj);
    	}else{
    		//robot is driving within acceptable range
    		Debugger.debug("STRAIGHT LINE CORRECTION: ", 0);
    	}
    	Robot.chassis.tankDrivePower(-leftSpeedAdj, -rightSpeedAdj);    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.chassis.getEncoderDistance() > encoderDistance; //TODO: does not work if going backwards
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.chassis.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
