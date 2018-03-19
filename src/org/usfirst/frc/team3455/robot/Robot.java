package org.usfirst.frc.team3455.robot;

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
		// = new AutonomousGroup();
		teleopCommand = new TankDriveTeleop();
	}

	public void autonomousInit() {

		////

		////
		/*
		this.gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (this.gameData.charAt(0) == 'R') {
			autonomousCommand = new AutonomousGroupRightSwitch();
		} else {
			autonomousCommand = new AutonomousGroupNoSwitch();
			// autonomousCommand = new AutonomousGroupLeftSwitch();
		}
		*/

		// autonomousCommand = new AutonomousGroupNoSwitch();

		Debugger.debug("GameData", gameData);
		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
		firstRun = false;
	}

	public void autonomousPeriodic() {
		// addSequential(autonomousCommand);
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