package org.usfirst.frc.team3455.robot.commands;

import org.usfirst.frc.team3455.robot.Robot;

import org.usfirst.frc.team3455.robot.utils.Debugger;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToBlockAuto extends Command {

	protected double drivePower;
	protected long startTimeRun;
	protected double timeSeconds;
	
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
	
	double blockd1 = -3455;
	double blockd2 = -3455;
	double blockTheta = -3455;
	
	double BLOCK_D2_THRESHOLD_PRECISE;
	double BLOCK_D1_THRESHOLD_PRECISE;
	
    public DriveToBlockAuto(double timeSeconds, double drivePower, double thresh, double close) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.chassis);
    	this.autoStraightSpeed = -drivePower;
    	this.timeSeconds = timeSeconds;
    	this.BLOCK_D2_THRESHOLD_PRECISE = thresh;
    	this.BLOCK_D1_THRESHOLD_PRECISE = close;
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
    	blockd1 = Robot.cameraTable.getDouble("d1", -3455);
		blockd2 = Robot.cameraTable.getDouble("d2", -3455);
		blockTheta = Robot.cameraTable.getDouble("theta", -3455);

		leftSpeedAdj = autoStraightSpeed*LEFT_COEFF;
    	rightSpeedAdj = autoStraightSpeed*RIGHT_COEFF;
    	if(Robot.chassis.getHeadingIMU() -offsetIMU - startHeading > IMU_THRESHOLD){
    		adjustVar = Math.abs(Robot.chassis.getHeadingIMU() -offsetIMU - startHeading);
    		rightSpeedAdj -= CORRECTION_MULTIPLIER * adjustVar;
    		Debugger.debug("STRAIGHT LINE CORRECTION RIGHT: ", rightSpeedAdj);
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
    	boolean isFinished = false;
    	if(blockTheta!=-3455 && Math.abs(blockd2) < BLOCK_D2_THRESHOLD_PRECISE && Math.abs(blockd1) < BLOCK_D1_THRESHOLD_PRECISE){
    		isFinished = true;
    	}
        return isFinished;
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
