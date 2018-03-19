package org.usfirst.frc.team3455.robot.subsystems;

import org.usfirst.frc.team3455.robot.Robot;
import org.usfirst.frc.team3455.robot.commands.TankDriveTeleop;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import org.usfirst.frc.team3455.robot.utils.BNO055;

/**
 *
 */
public class Intake extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public TalonSRX intakeMotors;
	public TalonSRX intakeTilt;
	
	public static double position = 0d;
	
	Encoder encoder;
	
	private final int TALON_INTAKE_PORT = 10;
	private final int TALON_TILT_PORT = 1;

	private double tiltEncoderOffset = 0;
	
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void init(){
    	intakeMotors = new TalonSRX(TALON_INTAKE_PORT);
    	intakeTilt = new TalonSRX(TALON_TILT_PORT);
    	
    	intakeTilt.configContinuousCurrentLimit(10, 0);
    	
    	//brDrive.setInverted(true);
    	
    	intakeMotors.setNeutralMode(NeutralMode.Brake);
    	intakeTilt.setNeutralMode(NeutralMode.Brake);
    	
    	//intakeMotors.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    	//intakeMotors.setSensorPhase(false);
    	
    	intakeMotors.configOpenloopRamp(0.5, 0);
    	intakeTilt.configOpenloopRamp(0.1, 0);
    	
    	encoder = new Encoder(9, 8, true, EncodingType.k4X);
    	encoder.setSamplesToAverage(5);
    	encoder.setDistancePerPulse(1.0/360);
    	
    	intakeMotors.configContinuousCurrentLimit(30, 10);
    	intakeMotors.configPeakCurrentLimit(0, 10);
    	intakeMotors.enableCurrentLimit(true);
    	
    	//Robot.dashboardTable.putNumber("intake inited: ", 1);
    	
    	tiltEncoderOffset = getRawTiltEncoder();
    	//flDrive.getSelectedSensorPosition(TALON_FL_PORT);
    }
    
    public double getTiltEncoder(){
    	return encoder.getDistance()-tiltEncoderOffset;
    }
    
    public double getRawTiltEncoder(){
    	return encoder.getDistance();
    }
    
    public void stopIntake(){
    	intakeMotors.set(ControlMode.PercentOutput, 0);
    }
    
    public void stopTilt(){
    	intakeTilt.set(ControlMode.PercentOutput, 0);
    }
	
	public void intakePower(double intakePower){
		intakeMotors.set(ControlMode.PercentOutput, intakePower);
	}
	
	public void tiltPower(double tiltPower){
		intakeTilt.set(ControlMode.PercentOutput, tiltPower);
	}

	public void coastDriveMode(){
		intakeMotors.setNeutralMode(NeutralMode.Coast);
	}

	public void brakeDriveMode(){
    	intakeMotors.setNeutralMode(NeutralMode.Brake);
	}
}

