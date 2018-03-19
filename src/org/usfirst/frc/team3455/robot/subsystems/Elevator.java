package org.usfirst.frc.team3455.robot.subsystems;

import org.usfirst.frc.team3455.robot.Robot;
import org.usfirst.frc.team3455.robot.commands.TankDriveTeleop;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 *
 */
public class Elevator extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private TalonSRX liftL;
	private TalonSRX liftR;

	private DigitalInput limitSwitch;
	
	private final int TALON_LL_PORT = 4;
	private final int TALON_LR_PORT = 5;

	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new TankDriveTeleop());
    }
    
    public void init(){
    	liftL = new TalonSRX(TALON_LL_PORT);
    	liftR = new TalonSRX(TALON_LR_PORT);   
    	
    	liftR.follow(liftL);
    	
    	liftL.setNeutralMode(NeutralMode.Brake);
    	
    	liftL.configOpenloopRamp(0.5, 0);
	
    	limitSwitch = new DigitalInput(6);
    	
    }
    
    public boolean getLimitSwitch() {
    	return limitSwitch.get();
    }
    
    public void stop(){
    	liftPower(0);
    }    
	
	public void liftPower(double liftPower){
		liftL.set(ControlMode.PercentOutput, liftPower);
	}
	public double getElevatorCurrent() {
		return liftL.getOutputCurrent();
	}
}

