package org.usfirst.frc.team2850.robot.subsystems;

import org.usfirst.frc.team2850.robot.RobotMap;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class DriveTrain extends PIDSubsystem {
	Spark leftDrive1 = new Spark(RobotMap.p_leftDrive1);
	Spark leftDrive2 = new Spark(RobotMap.p_leftDrive2);
	Spark leftDrive3 = new Spark(RobotMap.p_leftDrive3);
	Spark rightDrive1 = new Spark(RobotMap.p_rightDrive1);
	Spark rightDrive2 = new Spark(RobotMap.p_rightDrive2);
	Spark rightDrive3 = new Spark(RobotMap.p_rightDrive3);
	
	private static double pDrive = .07;
	private static double iDrive = 0;
	private static double dDrive = .007;
	
	//private static double PIDIn = 0;
	
	public static Encoder rightEncoder = new Encoder(RobotMap.p_rightEncoderA, RobotMap.p_rightEncoderB, false, Encoder.EncodingType.k4X);
	public static Encoder leftEncoder = new Encoder(RobotMap.p_leftEncoderA, RobotMap.p_leftEncoderB, false, Encoder.EncodingType.k4X);
	public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	RobotDrive drive1 = new RobotDrive(leftDrive1, leftDrive2, rightDrive1, rightDrive2);
	RobotDrive drive2 = new RobotDrive(leftDrive3, rightDrive3);
	
    // Initialize your subsystem here
    public DriveTrain() {
    	super("DriveTrain", pDrive, iDrive, dDrive);
    	leftEncoder.setDistancePerPulse(-0.01057);
    	rightEncoder.setDistancePerPulse(.01125);
    	enable();
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return (leftEncoder.getDistance()+rightEncoder.getDistance())/2;
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	drive1.tankDrive(output, output);
    	drive2.tankDrive(output, output);
    }
    
    public void tankDrive(double left, double right){
    	drive1.tankDrive(left, right);
    	drive2.tankDrive(left, right);
    }
    
//    public void pickPIDType(String type) {
//    	if(type.equals("Encoder")) {
//    		PIDIn = (leftEncoder.getDistance()+rightEncoder.getDistance())/2;
//    	}
//    	else {
//    		PIDIn = gyro.getAngle();
//    	}
//    }
}
