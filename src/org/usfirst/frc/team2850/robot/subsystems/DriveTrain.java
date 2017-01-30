package org.usfirst.frc.team2850.robot.subsystems;

import org.usfirst.frc.team2850.robot.RobotMap;
import org.usfirst.frc.team2850.robot.lib.Rotation2d;

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
	
	private int leftEncoderZero, rightEncoderZero;
    private boolean leftReverseEnc;
    private boolean rightReverseEnc;
    private int leftEncSign;
    private int rightEncSign;
    public double kP = 0, kI = 0, kD = 0;
	public double kF;
	double pPos = 0, iPos = 0, fPos = 0;
	int iZone = 0;
	
	private static final double WHEEL_DIAMETER_IN = 4.0;
	private static final int COUNTS_PER_REV = 360; //TODO: Determine actual value
	
	double leftSetpoint, rightSetpoint;
	double tolerance = 4.0 / COUNTS_PER_REV;

	private static double pDrive = .07;
	private static double iDrive = 0;
	private static double dDrive = .007;
	
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
    	drive1.tankDrive(output, output);
    	drive2.tankDrive(output, output);
    }
    
    public void tankDrive(double left, double right){
    	drive1.tankDrive(left, right);
    	drive2.tankDrive(left, right);
    }
    public void stop() {
		setMotorsRaw(0,0);
	}
	
	// Converts inches/second to motor percentage using feed-forward term
	public void setMotorsInchesPerSecondOpenLoop(double left, double right) {
		double leftSpeed = kF * inchesPerSecondToRPM(left) * COUNTS_PER_REV / 1023 / 60;
		double rightSpeed = kF * inchesPerSecondToRPM(right) * COUNTS_PER_REV / 1023 / 60;
		
		setMotors(leftSpeed, rightSpeed);
	}
	
	public void setMotors(double left, double right) {
		left = scaleLeft(left);
		right = scaleRight(right);
		
		setMotorsRaw(left, right);
	}

	public void setMotorsRaw(double left, double right) {
		left = safetyTest(left);
		right = safetyTest(right);
		
		
		leftDrive1.set(left);
		rightDrive1.set(right);	
		leftDrive2.set(left);
		rightDrive2.set(right);	
		leftDrive3.set(left);
		rightDrive3.set(right);	
	}
	
	private double safetyTest(double motorValue) {
	    motorValue = (motorValue < -1) ? -1 : motorValue;
	    motorValue = (motorValue > 1) ? 1 : motorValue;
	    
	    return motorValue;
	}
	
	
	
	
	/*
	public boolean nearSetpoint() {
		double currentLeftPosition = leftMotorFront.getPosition();
		boolean nearLeft = Math.abs(leftSetpoint - currentLeftPosition) < tolerance;
		
		double currentRightPosition = rightMotorFront.getPosition();
		boolean nearRight = Math.abs(rightSetpoint - currentRightPosition) < tolerance;
		
		return nearLeft && nearRight;
	}
	*/
	private double scaleLeft(double left) {
		return left;
	}
	
	private double scaleRight(double right) {
		return right;
	}
	
	public void resetEncoders() {
		leftEncoderZero = leftEncoder.get();
		rightEncoderZero = rightEncoder.get();
	}
	
	public int getLeftEncoder() {
		return leftEncSign * (leftEncoder.get() - leftEncoderZero);
	}
	
	public int getRightEncoder() {
		return rightEncSign * (rightEncoder.get() - rightEncoderZero);
	}
	
	public double getLeftPositionInches() {
		double rotations = ((double) getLeftEncoder()) / COUNTS_PER_REV;
		
		return rotationsToInches(rotations);
	}
	
	public double getRightPositionInches() {
		double rotations = ((double) getRightEncoder()) / COUNTS_PER_REV;
		
		return rotationsToInches(rotations);
	}
	
	public double getLeftVelocity() {
		return leftEncoder.getRate();
	} 
	
	public double getRightVelocity() {
		return rightEncoder.getRate();
	}
	
	public double getLeftVelocityInchesPerSecond() {
		return rpmToInchesPerSecond(getLeftVelocity());
	}
	
	public double getRightVelocityInchesPerSecond() {
		return rpmToInchesPerSecond(getRightVelocity());
	}
	
	
	
	public Rotation2d getGyro() {
		return Rotation2d.fromDegrees(gyro.getAngle());
	}
	
	public void resetGyro() {
		gyro.reset();
	}
	
	public void calibrateGyro() {
		gyro.calibrate();
	}
	
	
	
	
	
	
	
	private double rotationsToInches(double rotations) {
		return rotations * (Math.PI * WHEEL_DIAMETER_IN);
	}
	
	private double inchesToRotations(double inches) {
		return inches / (Math.PI * WHEEL_DIAMETER_IN);
	}
	
	private double rpmToInchesPerSecond(double rpm) {
		return rotationsToInches(rpm) / 60;
	}
	
	private double inchesPerSecondToRPM(double inchesPerSecond) {
		return inchesToRotations(inchesPerSecond * 60);
	}
 
}

