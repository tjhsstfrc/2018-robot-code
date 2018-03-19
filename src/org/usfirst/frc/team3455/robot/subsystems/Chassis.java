package org.usfirst.frc.team3455.robot.subsystems;

import org.usfirst.frc.team3455.robot.Robot;
import org.usfirst.frc.team3455.robot.commands.TankDriveTeleop;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import org.usfirst.frc.team3455.robot.utils.BNO055;

import edu.wpi.first.wpilibj.Encoder;

/**
 *
 */
public class Chassis extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public TalonSRX flDrive;
	public TalonSRX frDrive;
	public TalonSRX blDrive;
	public TalonSRX brDrive;
		
	private TalonSRX winch;
	
	private final int TALON_FL_PORT = 6;
	private final int TALON_FR_PORT = 7;
	private final int TALON_BL_PORT = 8;
	private final int TALON_BR_PORT = 9;

	private final int TALON_WN_PORT = 3;
	
	private final int TALON_LL_PORT = 4;
	private final int TALON_LR_PORT = 5;

	public static BNO055 imu;

	private double IMUoffset = 0;

	private double encoderOffset = 0;
	
	private int servoPosition = 0;
	//true if forward, false if backward
	private boolean servoDirection;
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new TankDriveTeleop());
    	Robot.dashboardTable.putNumber("chassis init defaulted: ", 1);
		imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
				BNO055.vector_type_t.VECTOR_EULER);
    }
    
    public void init(){
    	
    	flDrive = new TalonSRX(TALON_FL_PORT);
    	frDrive = new TalonSRX(TALON_FR_PORT);
    	blDrive = new TalonSRX(TALON_BL_PORT);
    	brDrive = new TalonSRX(TALON_BR_PORT);
    	
    	winch = new TalonSRX(TALON_WN_PORT);
    	
    	blDrive.follow(flDrive);
    	brDrive.follow(frDrive);
    	
    	frDrive.setInverted(true); //could be the right ones inverted who knows (eric does)
    	brDrive.setInverted(true);
    	
    	flDrive.setNeutralMode(NeutralMode.Brake); //brake by default may change later on
    	frDrive.setNeutralMode(NeutralMode.Brake);
    	blDrive.setNeutralMode(NeutralMode.Brake);
    	brDrive.setNeutralMode(NeutralMode.Brake);
    	
    	winch.setNeutralMode(NeutralMode.Brake);
    	
    	flDrive.configOpenloopRamp(0.5, 0); //5 as temp test
    	frDrive.configOpenloopRamp(0.5, 0);
    	blDrive.configOpenloopRamp(0.5, 0); //5 as temp test
    	brDrive.configOpenloopRamp(0.5, 0);
    	
    	winch.configOpenloopRamp(0.5, 0);
    	
    	imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
				BNO055.vector_type_t.VECTOR_EULER);
    	
    	flDrive.set(ControlMode.Position, 0);
    	frDrive.set(ControlMode.Position, 0);
    	flDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    	frDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    }
    
    
    public void stop(){
    	flDrive.set(ControlMode.Velocity, 0);
    	frDrive.set(ControlMode.Velocity, 0);
    }
    
    public void tankDriveVelocity(double leftV, double rightV){
    	flDrive.set(ControlMode.Velocity, leftV);
    	frDrive.set(ControlMode.Velocity, rightV);
    }
    
    public void tankDrivePower(double leftPower, double rightPower){
		flDrive.set(ControlMode.PercentOutput, leftPower);
    	frDrive.set(ControlMode.PercentOutput, rightPower);
	}
	
	public void tankDriveCurrent(double leftPower, double rightPower){
		flDrive.set(ControlMode.Current, leftPower);
    	frDrive.set(ControlMode.Current, rightPower);
	}
	
	public void winchPower(double winchPower) {
		winch.set(ControlMode.PercentOutput, winchPower);
	}

	public void coastDriveMode(){
		
		flDrive.setNeutralMode(NeutralMode.Coast);
    	frDrive.setNeutralMode(NeutralMode.Coast);
    	blDrive.setNeutralMode(NeutralMode.Coast);
    	brDrive.setNeutralMode(NeutralMode.Coast);
	}

	public void brakeDriveMode(){
		flDrive.setNeutralMode(NeutralMode.Brake);
    	frDrive.setNeutralMode(NeutralMode.Brake);
    	blDrive.setNeutralMode(NeutralMode.Brake);
    	brDrive.setNeutralMode(NeutralMode.Brake);
	}
	
	public void resetDriveEncoders(){
		flDrive.setSelectedSensorPosition(0, 0, 0);
		frDrive.setSelectedSensorPosition(0, 0, 0);
	}
	
	public void setTargetPosition(double ticks){
		flDrive.set(ControlMode.Position, ticks, 1);
		frDrive.set(ControlMode.Position, ticks, 1);
	}
	
	public boolean reachedPosition(int target){
		return flDrive.getSelectedSensorPosition(0) >= target && frDrive.getSelectedSensorPosition(0) >= target;
	}
	
	public double getEncoderDistance(){
		double some_random_gear_ratio = 1;
		return (getRawEncoderValue()/90*12.56*some_random_gear_ratio);
	}
	
	public double getRawEncoderValue() {
		return Robot.cameraTable.getNumber("encoder_value", -0.3455) - this.encoderOffset;
	}
	
	public void resetEncoderValue() {
		this.encoderOffset = Robot.cameraTable.getNumber("encoder_value", -0.3455);
	}

	public void resetHeadingIMU(){
		IMUoffset = imu.getHeading();
	}

	public double getHeadingIMU(){
		return imu.getHeading()-IMUoffset;
	}
}

