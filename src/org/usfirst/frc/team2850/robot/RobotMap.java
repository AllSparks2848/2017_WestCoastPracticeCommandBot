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
	 public static final double NANOS_PER_MINUTE = 60.0e9;
	  public static final double NANOS_PER_SECOND = 1.0e9;
	  public static final double MILLIMETERS_PER_INCH = 25.4;
	  public static final double MILLIVOLTS_PER_MILLIMETER = 0.977;
	  public static final int QUAD_ENCODER_TICKS_PER_REV = 256;
	  public static final double WHEEL_RADIUS = 4; // in
	  public static final double WHEEL_DIAMETER = WHEEL_RADIUS * 2;
	  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
	  public static final double GEARING_FACTOR = 1;
	  public static final double AUTO_TURN_MAX_SPEED = 0.3;
	  public static final double AUTO_TURN_MIN_SPEED = 0.18;
	  public static final double AUTO_TURN_RAMP_ZONE = 80.0;
	  public static final double AUTO_TURN_ACCEPTABLE_ERROR = 3; // Degrees
	  
	  public static final double AUTO_DRIVE_ACCEPTABLE_ERROR = 2.0; // in
	  public static final double AUTO_FINE_DRIVE_ACCEPTABLE_ERROR = 0.5;
	  
	  public static final double AUTO_DRIVE_TURN_P = 0.02;
	  
	  public static final double AUTO_DRIVE_MIN_SPEED = 0.1; // percent voltage from 12.5V
	  public static final double AUTO_DRIVE_START_SPEED = 0.3;
	  public static final double AUTO_DRIVE_MAX_SPEED = 0.5;
	  
	  public static final double AUTO_DRIVE_RAMP_UP_DIST = 24.0; // in
	  public static final double AUTO_DRIVE_RAMP_DOWN_DIST = 60.0;
}
