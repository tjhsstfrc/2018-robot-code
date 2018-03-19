package org.usfirst.frc.team3455.robot.commands;

import org.usfirst.frc.team3455.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnPointAuto extends Command {

    private int degreeTargetValue;
    private double turnPower;
    private double turnPowerAdj;
    private final double CORRECTION_MULT = 0.8;
    private final double MIN_TURN_POWER = 0.1;
    
    public TurnPointAuto(int degreeTargetValue, double turnPower) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.chassis);
        this.degreeTargetValue = degreeTargetValue;
        this.turnPower = turnPower;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.chassis.resetHeadingIMU();
    	//Robot.chassis.setTargetPosition(this.degreeTargetValue);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//        turnPowerAdj = turnPower;
//        if(Math.abs(Robot.chassis.getHeadingIMU()-degreeTargetValue)<45){
//        	turnPowerAdj = 0.3;
//        }
//        if(Math.abs(Robot.chassis.getHeadingIMU()-degreeTargetValue)<15){
//        	turnPowerAdj = 0.2;
//        }
//        turnPowerAdj = turnPowerAdj-CORRECTION_MULT*turnPowerAdj*(1-Math.abs((Robot.chassis.getHeadingIMU()-degreeTargetValue)/degreeTargetValue))tepad;
    	//turnPowerAdj = Math.min(1-Math.abs(Robot.chassis.getHeadingIMU()/degreeTargetValue)*turnPower+0.2, turnPower);
    	    	
    	if(degreeTargetValue > 0){
    		Robot.chassis.tankDrivePower(turnPower, -turnPower);
    	}else{
    		Robot.chassis.tankDrivePower(-turnPower, turnPower);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(degreeTargetValue > 0) {
    		return Robot.chassis.getHeadingIMU() >= degreeTargetValue;
    	} else {
    		return Robot.chassis.getHeadingIMU() <= degreeTargetValue;
    	}
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
