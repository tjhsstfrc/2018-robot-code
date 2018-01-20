import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * Default comment
 */
public class Robot extends IterativeRobot {
	SerialPort thePort;
	Joystick joy;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		thePort = new SerialPort(9600,Port.kUSB);
		joy = new Joystick(0);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		if(joy.getRawButton(1)) {
			//System.out.println("Red");
			thePort.writeString("!255R000G000B");  //Sends successfully, (Neopixels turn red) then crashes robot code
			thePort.readString();
		}

		if(joy.getRawButton(2)) {
			//System.out.println("Green");
			thePort.writeString("!000R255G000B"); //Sends successfully, (Neopixels turn Green) then crashes robot code
		}

		if(joy.getRawButton(3)) {
			//System.out.println("Blue");
			thePort.writeString("!000R000G255B"); //Sends successfully, (Neopixels turn Blue) then crashes robot code
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