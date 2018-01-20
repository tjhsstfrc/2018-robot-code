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

public class Robot_IMU extends IterativeRobot {
	
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
		
		
	}
	
	double offsetIMU = 0.0;
	boolean errorIMU = false;
	double startHeading = 0.0;
	final double IMU_THRESHOLD = 0.5;

	public void autonomousInit() {
		long startAutoTime = System.currentTimeMillis();
		encoder.reset();
//		readyIMU: while(!imu.isCalibrated()||!imu.isInitialized()){
//			if(System.currentTimeMillis()-startAutoTime>5000){ //temporary timeout time for init and calibrating the imu, might change later to less lenient
//				errorIMU = true; //uh oh the imu didn't work, resort to backup auto sequence
//				System.out.println("ERROR: IMU DID NOT CALIBRATE");
//				table.putNumber("IMU READY", -1);
//				break readyIMU;
//			}
//		}
		if(!errorIMU){
			table.putNumber("IMU READY", 0);
			offsetIMU = imu.getHeading();
			System.out.println("START: "+(imu.getHeading()));
			System.out.println("START WITH OFFSET: "+(imu.getHeading()-offsetIMU));
			startHeading = imu.getHeading()-offsetIMU;
		}
	}
	
	//might make it scale logarithmically in the future
	final double CORRECTION_MULTIPLIER = 0.075;
	final double LEFT_COEFF = 0.4;
	final double RIGHT_COEFF = 1.0;
	final double CORRECTION_ADD = 0.0;
	double autoStraightSpeed = -0.4;
	double leftSpeedAdj, rightSpeedAdj;
	double adjustVar;

	public void autonomousPeriodic() {
		leftSpeedAdj = autoStraightSpeed*LEFT_COEFF;
		rightSpeedAdj = autoStraightSpeed*RIGHT_COEFF;

		if(imu.getHeading() -offsetIMU - startHeading > IMU_THRESHOLD){
			adjustVar = Math.abs(imu.getHeading() -offsetIMU - startHeading);
			rightSpeedAdj -= CORRECTION_MULTIPLIER * adjustVar;
			table.putNumber("STRAIGHT LINE CORRECTION RIGHT: ", rightSpeedAdj);
		}else if(imu.getHeading() -offsetIMU - startHeading < -IMU_THRESHOLD){
			adjustVar = Math.abs(imu.getHeading() -offsetIMU - startHeading);
			leftSpeedAdj -= CORRECTION_MULTIPLIER * adjustVar;
			table.putNumber("STRAIGHT LINE CORRECTION LEFT: ", leftSpeedAdj);
		}else{
			//robot is driving within acceptable range
			table.putNumber("STRAIGHT LINE CORRECTION: ", 0);
		}
		table.putNumber("imu heading: ", imu.getHeading());
		table.putNumber("final left speed: ", leftSpeedAdj);
		table.putNumber("final right speed: ", rightSpeedAdj);
		table.putNumber("current adjust state: ", imu.getHeading() -offsetIMU - startHeading);
		tankDriveNoThresh(leftSpeedAdj, rightSpeedAdj);
		// table.putNumber("HEADING w/ OFFSET", imu.getHeading()-offsetIMU);
		// //mod 360 because imu is continuous value
		// if((imu.getHeading()-offsetIMU)%360<90.0){
		// 	tankDrive(-0.6, 0.6);
		// 	System.out.println("no offset: "+(imu.getHeading()));
		// 	System.out.println("with offset: "+(imu.getHeading()-offsetIMU));
		// }else{
		// 	tankDrive(0, 0);
		// }
		
	}

	public void teleopInit() {
		encoder.reset();
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		while (isOperatorControl() && isEnabled() && this.allowOperator) {

			//Emergency Shutdown
			if((first.getRawButton(8) && first.getRawButton(9)) || ((second.getRawButton(8) && second.getRawButton(9)))) {
				System.exit(1);
			}
			
			if(second.getRawButton(1)) {
				shooterFlag = !shooterFlag;
			}
			
			if(first.getRawButton(1)) {
				intakeFlag = !intakeFlag;
			}
			/*
			if(first.getRawButton(6) && first.getRawButton(7) && Robot.allowOperator) {
				Thread cameraThread = new Thread(() -> {
					Robot.allowOperator = false;
					table.putBoolean("startCV", true);
					
					try {
						Thread.sleep(250);
					} catch (Exception) {
						;
					}

					double turn = table.getNumber("Turning_Value", 0.0);
					boolean turning = table.getBoolean("Turning", false);
			
					while(turning && turn != 0.0){
						if((first.getRawButton(8) && first.getRawButton(9)) || ((second.getRawButton(8) && second.getRawButton(9)))) {
							System.exit(1);
						}
						frontLeft.set(limit(turn));
						frontRight.set(limit(-turn));
						backLeft.set(limit(turn));
						backRight.set(limit(-turn));
						turning = table.getBoolean("Turning", false);
						turn = table.getNumber("Turning_Value", 0.0);
					}

					try {
						Thread.sleep(250);
					} catch (Exception) {
						;
					}

					double forward = table.getNumber("Linear_Value", 0.0);
					turning = table.getBoolean("Turning", false);

					while(!turning && forward != 0.0){
						if((first.getRawButton(8) && first.getRawButton(9)) || ((second.getRawButton(8) && second.getRawButton(9)))) {
							System.exit(1);
						}
						tankDrive(-forward, -forward);
						turning = table.getBoolean("Turning", false);
						turn = table.getNumber("Linear_Value", 0.0);
					}
					
					frontLeft.set(0.0);
					frontRight.set(0.0);
					backLeft.set(0.0);
					backRight.set(0.0);
					
					Robot.allowOperator = true;

					Thread rumbleThread = new Thread(() -> {
						first.setRumble(RumbleType.kLeftRumble, 0.5);
						first.setRumble(RumbleType.kRightRumble, 0.5);
						second.setRumble(RumbleType.kLeftRumble, 0.5);
						second.setRumble(RumbleType.kRightRumble, 0.5);
						Thread.sleep(500);
						first.setRumble(RumbleType.kLeftRumble, 0);
						first.setRumble(RumbleType.kRightRumble, 0);
						second.setRumble(RumbleType.kLeftRumble, 0);
						second.setRumble(RumbleType.kRightRumble, 0);
		        	});
		        	rumbleThread.start();
		        	table.putBoolean("startCV", false);
		        });
				cameraThread.setName("cameraThread");
				
				if(getThreadByName("cameraThread") != null) {
					cameraThread.start();
				}
			}			
			*/
			if(shooterFlag) {
				shooterMotor.set(0.75);
				feederMotor.set(0.75);
			} else {
				shooterMotor.set(0.0);
				feederMotor.set(0.0);
			}
			
			if(intakeFlag) {
				intakeMotor.set(0.25);
			} else {
				intakeMotor.set(0.0);
			}
			
			if(second.getRawButton(2)) {
				climberMotor.set(0.8);
			} else {
				climberMotor.set(0.0);
			}
			
			table.putNumber("encoder", encoder.getDistance());
			yAxis1 = first.getRawAxis(1);
			yAxis2 = first.getRawAxis(5);
    		tankDrive(yAxis1, yAxis2);
    		
    		table.putNumber("imu heading", imu.getHeading());
    		table.putNumber("yaxis2", yAxis2);
    		
    		Timer.delay(0.01);
    		// wait to avoid hogging CPU cycles
		}
	}

	public void tankDriveNoThresh(double leftValue, double rightValue) {

		// square the inputs (while preserving the sign) to increase fine
		// control while permitting
		// full power
		leftValue = limit((leftValue));
		rightValue = limit((rightValue));
		
		frontLeft.set(leftValue);
		backLeft.set(leftValue);
		frontRight.set(rightValue);
		backRight.set(rightValue);
		
	}
	
	public void tankDrive(double leftValue, double rightValue) {

		// square the inputs (while preserving the sign) to increase fine
		// control while permitting
		// full power
		leftValue = limit((leftValue));
		rightValue = limit((rightValue));
		
		frontLeft.set(thresh(leftValue));
		backLeft.set(thresh(leftValue));
		frontRight.set(thresh(rightValue));
		backRight.set(thresh(rightValue));
		
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
	
	public Thread getThreadByName(String threadName) {
	    for (Thread t : Thread.getAllStackTraces().keySet()) {
	        if (t.getName().equals(threadName)) return t;
	    }
	    return null;
	}

	public void testPeriodic() {
		
	}
	
	public double getEncoderValue(double feet) {
		return feet / WHEEL_CIRCUM;
	}


	//.
	//..
	//...
	//EXPERIMENTAL IMU CODE

	// public double imuTest() {
    // 	return imu.;
    // }

	public double getHeading() {
    	return imu.getHeading();
    }
    
    /**
     * Gets a vector representing the sensors position (heading, roll, pitch).
	 * heading:    0 to 360 degrees
	 * roll:     -90 to +90 degrees
	 * pitch:   -180 to +180 degrees
	 *
	 * For continuous rotation heading (doesn't roll over between 360/0) see
	 *   the getHeading() method.
	 *
	 * @return a vector [heading, roll, pitch]
	 */
    public double[] getVector() {
    	return imu.getVector();
    }
    
	/**
	 * @return true if the IMU is found on the I2C bus
	 */
	public boolean isSensorPresent() {
		return imu.isSensorPresent();
	}

	/** 
	 * @return true when the IMU is initialized.
	 */
	public boolean isInitialized() {
		return imu.isInitialized();
	}
	
	/**
	 * Gets current IMU calibration state.
	 * @return each value will be set to 0 if not calibrated, 3 if fully
	 *   calibrated.
	 */
	public BNO055.CalData getCalibration() {
		return imu.getCalibration();
	}
	
	/**
	 * Returns true if all required sensors (accelerometer, magnetometer,
	 *   gyroscope) in the IMU have completed their respective calibration
	 *   sequence.
	 * @return true if calibration is complete for all sensors required for the
	 *   mode the sensor is currently operating in. 
	 */
	public boolean isCalibrated() {
		return imu.isCalibrated();
	}

}