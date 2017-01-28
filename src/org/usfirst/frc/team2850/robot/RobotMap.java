package org.usfirst.frc.team2850.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	//Joy stick ports
	public static int p_xbox1 = 0;
	
	//Actuator ports
	public static int p_leftDrive1 = 0;
	public static int p_leftDrive2 = 1;
	public static int p_leftDrive3 = 2;
	public static int p_rightDrive1 = 3;
	public static int p_rightDrive2 = 4;
	public static int p_rightDrive3 = 5;
	
	public static int p_driveshifter = 0;
	
	//Sensor ports
	public static int p_leftEncoderA = 0;
	public static int p_leftEncoderB = 1;
	public static int p_rightEncoderA = 3;
	public static int p_rightEncoderB = 2;
	public static int p_navX = 0;
}
