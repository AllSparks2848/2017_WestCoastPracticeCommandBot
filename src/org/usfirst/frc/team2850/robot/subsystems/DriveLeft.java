package org.usfirst.frc.team2850.robot.subsystems;

import org.usfirst.frc.team2850.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class DriveLeft extends PIDSubsystem {
	Spark leftDrive1 = new Spark(RobotMap.p_leftDrive1);
	Spark leftDrive2 = new Spark(RobotMap.p_leftDrive2);
	Spark leftDrive3 = new Spark(RobotMap.p_leftDrive3);
	
	private static double pDrive = .07;
	private static double iDrive = 0;
	private static double dDrive = .007;
	
	public static Encoder leftEncoder = new Encoder(RobotMap.p_leftEncoderA, RobotMap.p_leftEncoderB, false, Encoder.EncodingType.k4X);
	
    // Initialize your subsystem here
    public DriveLeft() {
        super("DriveLeft", pDrive, iDrive, dDrive);
    	leftEncoder.setDistancePerPulse(-0.0115);
    	enable();
    }

    public void initDefaultCommand() {
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return leftEncoder.getDistance();
    }

    protected void usePIDOutput(double output) {
    	leftDrive1.set(output);
    	leftDrive2.set(output);
		leftDrive3.set(output);
    }
}
