	package org.usfirst.frc.team3455.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Encoder;

/**
 * This is a demo program showing the use of the CameraServer class. With start
 * automatic capture, there is no opportunity to process the image. Look at the
 * IntermediateVision sample for how to process the image before sending it to
 * the FRC PC Dashboard.
 */

public class Robot extends IterativeRobot {
	
	Encoder encoder;
	NetworkTable table;
	double data = 0.0;

	//RobotDrive myDrive;
	public static Joystick first;
	public static Joystick second;
	
	// Channels for the wheels
	// final int frontLeftChannel = 4;
	// final int rearLeftChannel = 2;
	// final int frontRightChannel = 3;
	// final int rearRightChannel = 1;

	Talon frontLeft;
	Talon frontRight;
	Talon backLeft;
	Talon backRight;
	Talon shooterMotor;
	Talon feederMotor;
	Talon climberMotor;
	Talon intakeMotor;
	

	double WHEEL_CIRCUM = 0.5 * Math.PI;
	double scalingFactorEncoder = 2.5;
	double errorFix = 0.05; //only when no friction
	double yAxis1;
	double yAxis2;
	double leftPower;
	double rightPower;
	
	boolean intakeFlag = false;
	boolean shooterFlag = false;
	
	public static boolean allowOperator = true;
	
	// update every 5 milliseconds
	double kUpdatePeriod = 0.005;

	private static BNO055 imu;
	
	private SerialPort thePort;
	
	public void robotInit() {
		
		table = NetworkTable.getTable("SmartDashboard");
    	CameraServer.getInstance().startAutomaticCapture();
		
    	encoder = new Encoder(1, 2, true, EncodingType.k4X);
		// CameraServer.getInstance().startAutomaticCapture();
		/*
		 * new Thread(() -> { UsbCamera camera =
		 * CameraServer.getInstance().startAutomaticCapture();
		 * camera.setResolution(640, 480);
		 * 
		 * CvSink cvSink = CameraServer.getInstance().getVideo(); CvSource
		 * outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
		 * 
		 * Mat source = new Mat(); Mat output = new Mat();
		 * 
		 * while(!Thread.interrupted()) { cvSink.grabFrame(source);
		 * outputStream.putFrame(source); } }).start();
		 * NetworkTable.setServerMode(); NetworkTable.setTeam(3455); table =
		 * NetworkTable.getTable("SmartDashboard");
		 */
    	encoder.setSamplesToAverage(5);
    	encoder.setDistancePerPulse(1.0/360 * scalingFactorEncoder);

    	first = new Joystick(1);
    	second = new Joystick(0);

		frontLeft = new Talon(5);
		frontRight = new Talon(8);
		backLeft = new Talon(6);
		backRight = new Talon(7);
		
		intakeMotor = new Talon(1);
		feederMotor = new Talon(2);
		shooterMotor = new Talon(3);
		climberMotor = new Talon(4);
		
		frontLeft.setInverted(true);
		backLeft.setInverted(true);

		imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
				BNO055.vector_type_t.VECTOR_EULER);
		
		thePort = new SerialPort(9600,Port.kUSB);
		
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	
	public void teleopInit() {
		encoder.reset();
	}
	
	String testMessage = "nothing yet";
	public void teleopPeriodic() {
		testMessage = thePort.readString();
		if(testMessage.equals("nothing yet")){
			table.putNumber("no message yet: ", System.currentTimeMillis());
		}else{
			table.putString("STRING RECIEVED FROM SERIAL: ", testMessage);
		}
		
		
	}
	
	/**
	 * Default Comment
	 */
	public void autonomousInit() { }

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() { }

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() { }

}