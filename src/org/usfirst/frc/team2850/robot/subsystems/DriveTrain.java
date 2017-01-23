package org.usfirst.frc.team2850.robot.subsystems;

import org.usfirst.frc.team2850.robot.Robot;
import org.usfirst.frc.team2850.robot.RobotMap;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem {	
		//Actuator objects
		public static Spark leftDrive1 = new Spark(RobotMap.p_leftDrive1);
		public static Spark rightDrive1 = new Spark(RobotMap.p_rightDrive1);
		public static Spark leftDrive2 = new Spark(RobotMap.p_leftDrive2);
		public static Spark rightDrive2 = new Spark(RobotMap.p_rightDrive2);
		public static Spark leftDrive3 = new Spark(RobotMap.p_leftDrive3);
		public static Spark rightDrive3 = new Spark(RobotMap.p_rightDrive3);
		
		public static Compressor compressor = new Compressor();
		
		//RobotDrive objects
		public static RobotDrive drivetrain1 = new RobotDrive(leftDrive1, leftDrive2, rightDrive1, rightDrive2);
		public static RobotDrive drivetrain2 = new RobotDrive(leftDrive3, rightDrive3);
		
		//Sensor Objects
		public static Encoder leftEncoder = new Encoder(RobotMap.p_leftEncoderA, RobotMap.p_leftEncoderB, false, Encoder.EncodingType.k4X);
		public static Encoder rightEncoder = new Encoder(RobotMap.p_rightEncoderA, RobotMap.p_rightEncoderB, false, Encoder.EncodingType.k4X);;
		public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();
		
		//PID constants
		public static double pDrive = .07;
		public static double iDrive = 0;
		public static double dDrive = .007;
		public static double pGyro = .01;
		public static double iGyro = 0;
		public static double dGyro = 0.001;
		public static PIDController driveControllerLeft = new PIDController(pDrive, iDrive, dDrive, leftEncoder, leftDrive1);
		public static PIDController driveControllerRight = new PIDController(pDrive, iDrive, dDrive, rightEncoder, rightDrive1);
		public PIDController gyroController = new PIDController(pGyro, iGyro, dGyro, gyro, leftDrive1);
		
		//shifter
		public static Solenoid driveshifter = new Solenoid(RobotMap.p_driveshifter);
	    public static boolean high = true;
	    
	    public static boolean pidDriveFinished() {
	    	System.out.println("Left Enc: " + leftEncoder.get());
	    	System.out.println("Right Enc: " + rightEncoder.get());
	    	System.out.println("Left PID: " + driveControllerLeft.get());
	    	System.out.println("Right PID: " + driveControllerRight.get());
	    	if (driveControllerLeft.getError() > 2 && driveControllerRight.getError() > 2) {
	    		return false;
	    	}
	    	else
	    	{
	    		System.out.println("Return True");
	    	}
	    	return false;
	    }
	    
	    public static void pidDriveEnd() {
	    	leftDrive2.set(0);
			leftDrive3.set(0);
			rightDrive2.set(0);
			rightDrive3.set(0);
			driveControllerLeft.disable();
			driveControllerRight.disable();
	    }
	    
	    public static void pidDriveInit(double distance) {
	    	leftEncoder.reset();
	    	rightEncoder.reset();
	    	
	    	driveControllerLeft.setOutputRange(-1, 1);
	    	driveControllerRight.setOutputRange(-1, 1);
	    	
	    	leftEncoder.setDistancePerPulse(0.0115);
	    	rightEncoder.setDistancePerPulse(-0.0117);
	    	
	    	driveControllerLeft.setSetpoint(-distance);
			driveControllerRight.setSetpoint(distance);
	    }
	    
	    public static void pidDriveEnable() {
	    	driveControllerLeft.enable();
	    	driveControllerRight.enable();
	    }
	    
	    public static void pidDriveDisable() {
	    	driveControllerLeft.disable();
	    	driveControllerRight.disable();
	    }

	    public static void pidDrive(double distance) {
	    	leftDrive2.set(driveControllerLeft.get());
			leftDrive3.set(driveControllerLeft.get());
			rightDrive2.set(driveControllerRight.get());
			rightDrive3.set(driveControllerRight.get());
	    }
	    
	    public void initDefaultCommand() {
	        // Set the default command for a subsystem here.
	        //setDefaultCommand(new MySpecialCommand());
	    }
}

