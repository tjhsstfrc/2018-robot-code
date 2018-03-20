package org.usfirst.frc.team3455.robot;

import org.usfirst.frc.team3455.robot.commandgroups.*;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team3455.robot.commands.DriveForwardAuto;
import org.usfirst.frc.team3455.robot.commands.TankDriveTeleop;
import org.usfirst.frc.team3455.robot.commands.TurnPointAuto;
import org.usfirst.frc.team3455.robot.subsystems.Chassis;
import org.usfirst.frc.team3455.robot.subsystems.Elevator;
import org.usfirst.frc.team3455.robot.subsystems.Intake;

import org.usfirst.frc.team3455.robot.utils.Debugger;
import org.usfirst.frc.team3455.robot.utils.OI;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	public static Chassis chassis = new Chassis();
	public static Intake intake = new Intake();
	public static Elevator elevator = new Elevator();
	public static NetworkTable dashboardTable;
	public static NetworkTable cameraTable;
	OI oi;
	Command autonomousCommand;
	Command teleopCommand;
	public static String gameData = "";
	SendableChooser autoChooser;

	public long startTime;
	public boolean firstRun = false;

	// public static SerialPort serialArduino = new SerialPort(9600,
	// SerialPort.Port.kMXP);
	public void robotInit() {
		// serialArduino.disableTermination();
		// serialArduino.readString();
		Debugger.setDebugMode(true);
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(160, 120);
		camera.setFPS(10);
		oi = new OI();
		dashboardTable = NetworkTable.getTable("SmartDashboard");
		cameraTable = NetworkTable.getTable("PIcamera");
		chassis.init();
		intake.init();
		elevator.init();
		
		teleopCommand = new TankDriveTeleop();
		autoChooser = new SendableChooser();
		autoChooser.addDefault("LEFT", "LEFT");
		autoChooser.addDefault("CENTER", "CENTER");
		autoChooser.addDefault("RIGHT", "RIGHT");
		SmartDashboard.putData("Autonomous position chooser", autoChooser);
	}

	public void autonomousInit() {
		String position = (String) autoChooser.getSelected();
		this.gameData = DriverStation.getInstance().getGameSpecificMessage();
		if(position.equals("CENTER")) {
			if(this.gameData.charAt(0) == 'L') {
				autonomousCommand = new AutoCenterSwitchLeft();
			} else if(this.gameData.charAt(0) == 'R') {
				autonomousCommand = new AutoCenterSwitchRight();
			} 
		} else if(position.equals("LEFT")) {
			if(this.gameData.charAt(0) == 'L') {
				if(this.gameData.charAt(1) == 'L') {
					autonomousCommand = new AutoLeftSwitchLeftScaleLeft();
				} else if(this.gameData.charAt(1) == 'R') {
					autonomousCommand = new AutoLeftSwitchLeft();
				} 
			} else if(this.gameData.charAt(0) == 'R') {
				if(this.gameData.charAt(1) == 'L') {
					autonomousCommand = new AutoLeftSwitchRightScaleLeft(); //AutoLeftSwitchRight
				} else if(this.gameData.charAt(1) == 'R') {
					autonomousCommand = new AutoLeftSwitchRight();
				} 
			} 
		} else if(position.equals("RIGHT")) {
			if(this.gameData.charAt(0) == 'L') {
				if(this.gameData.charAt(1) == 'L') {
					autonomousCommand = new AutoRightSwitchLeft();
				} else if(this.gameData.charAt(1) == 'R') {
					autonomousCommand = new AutoRightSwitchLeftScaleRight();
				} 
			} else if(this.gameData.charAt(0) == 'R') {
				if(this.gameData.charAt(1) == 'L') {
					autonomousCommand = new AutoRightSwitchRight(); //AutoRightSwitchRight
				} else if(this.gameData.charAt(1) == 'R') {
					autonomousCommand = new AutoRightSwitchRightScaleRight();
				} 
			} 
		}
		Debugger.debug("GameData", gameData);
		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	public void teleopInit() {
		if (teleopCommand != null) {
			teleopCommand.start();
		}
	}

	public void teleopPeriodic() {

		Debugger.debug("limit switch", elevator.getLimitSwitch());
		Debugger.debug("imu value", chassis.getHeadingIMU());
		Debugger.debug("left drive encoder", chassis.flDrive.getSensorCollection().getQuadraturePosition());
		Debugger.debug("right drive encoder", chassis.frDrive.getSensorCollection().getQuadraturePosition());

		Scheduler.getInstance().run();
	}
}