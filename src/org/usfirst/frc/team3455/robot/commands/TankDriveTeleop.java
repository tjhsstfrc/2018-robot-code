package org.usfirst.frc.team3455.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.*;

import org.omg.PortableInterceptor.ObjectIdHelper;
import org.usfirst.frc.team3455.robot.Robot;
import org.usfirst.frc.team3455.robot.subsystems.Chassis;
import org.usfirst.frc.team3455.robot.subsystems.Elevator;

import org.usfirst.frc.team3455.robot.utils.Debugger;
import org.usfirst.frc.team3455.robot.utils.OI;

/**
 *
 */
public class TankDriveTeleop extends Command {
	
	private boolean intakeForceFlag = false;
	private long intakeForceFlagLastClicked;
	
	private double TILT_ENCODER_OFFSET;
	private double TILT_ENCODER_TARGET = -0.4;
	
    public TankDriveTeleop() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);(Chassis);
    	requires(Robot.chassis);
    	requires(Robot.intake);
    	requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//should this be here or in the actual robot file??
    	//Robot.chassis.init();
    	this.intakeForceFlagLastClicked = System.currentTimeMillis(); 
    	Robot.chassis.resetHeadingIMU();
    	TILT_ENCODER_TARGET = Robot.intake.getTiltEncoder();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//change to drive power or drive velocity depending on which
    	    	    
    	Robot.chassis.tankDrivePower(-thresh(OI.stickOutput(0).getRawAxis(1))+thresh(OI.stickOutput(0).getRawAxis(4))*0.75, -thresh(OI.stickOutput(0).getRawAxis(1))-thresh(OI.stickOutput(0).getRawAxis(4))*0.75);
    	
    	Robot.elevator.liftPower(thresh(OI.stickOutput(1).getRawAxis(1)));
    	
    	//winch
    	if(thresh(OI.stickOutput(1).getRawAxis(2)) != 0.0d) {
    		Robot.chassis.winchPower(thresh(OI.stickOutput(1).getRawAxis(2)));
    	} else if(thresh(OI.stickOutput(1).getRawAxis(3)) != 0.0d) {
    		Robot.chassis.winchPower(-thresh(OI.stickOutput(1).getRawAxis(3)));
    	} else {
    		Robot.chassis.winchPower(0.0);
    	}
    
    	//button to toggle intake always moving
    	if(OI.stickOutput(1).getRawButton(2) && System.currentTimeMillis() - this.intakeForceFlagLastClicked > 500L) {
    		this.intakeForceFlag = !this.intakeForceFlag;
    		this.intakeForceFlagLastClicked = System.currentTimeMillis();
    	}
    	if(this.intakeForceFlag && thresh(OI.stickOutput(1).getRawAxis(5)) == 0.0d) {
    		Robot.intake.intakePower(0.2);
    	} else {
    		Robot.intake.intakePower(0.0);
    	}    	
    	
//    	//actuators
//    	if(OI.stickOutput(0).getRawButton(5)) {
//    		Robot.chassis.moveActuatorUp();
//    	} else if(OI.stickOutput(0).getRawButton(6)) {
//    		Robot.chassis.moveActuatorDown();
//    	}
    	
    	if(thresh(OI.stickOutput(1).getRawAxis(5)) != 0.0d) {
    		Robot.intake.intakePower(thresh(OI.stickOutput(1).getRawAxis(5)));
    	} else {
    	}
    	if(OI.stickOutput(1).getRawButton(4)){
    		Robot.intake.tiltPower(0.7);
    		OI.stickOutput(1).setRumble(RumbleType.kRightRumble, 1);
    		TILT_ENCODER_TARGET += 0.05;
    	}else if(OI.stickOutput(1).getRawButton(1)){
    		Robot.intake.tiltPower(-0.7);
    		OI.stickOutput(1).setRumble(RumbleType.kRightRumble, 1);
    		TILT_ENCODER_TARGET -= 0.05;
    	}else{
    		if(Robot.intake.getTiltEncoder() > TILT_ENCODER_TARGET){
    			Robot.intake.tiltPower(-0.1);
    		}else if(Robot.intake.getTiltEncoder() < TILT_ENCODER_TARGET){
    			Robot.intake.tiltPower(0.1);
    		}else{
    			Robot.intake.tiltPower(0);
    		}
    		OI.stickOutput(1).setRumble(RumbleType.kRightRumble, 0);
    	}
    	
    	
    	Debugger.debug("LIFT CURRENT", Robot.elevator.getElevatorCurrent());
    	
//    	Robot.serialArduino.writeString("1");
//    	String recievedString = Robot.serialArduino.readString();
//    	Robot.dashboardTable.putString("RECIEVED STRING: ", recievedString);
//    	
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
    protected static double limit(double num) {
		if (num > 1.0) {
			return 1.0;
		}
		if (num < -1.0) {
			return -1.0;
		}
		return num;
	}
	
	protected static double thresh(double num) {
		if(num < 0.25 && num > -0.25) {
			return 0.0;
		}
		return num;
	}
}
	