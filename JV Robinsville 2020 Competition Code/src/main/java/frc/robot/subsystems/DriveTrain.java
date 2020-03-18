package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.swerve.SwerveModule;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//The drive train subsystem, which handles driving calculations and gives modules commands
//Consists of four SwerveModules
public class DriveTrain extends SubsystemBase {
	
	private final SwerveModule modFrontLeft;
	private final SwerveModule modFrontRight;
	private final SwerveModule modBackLeft;
	private final SwerveModule modBackRight;

	public DigitalOutput frontLeftStop;
	public DigitalOutput frontRightStop;
	public DigitalOutput backLeftStop;
	public DigitalOutput backRightStop;

	public double speedModifier = 1;
	
	//This set of inversions works when all of the wide parts of the modules begin facing (---inwards---) Correction: outwards
	//Invert accordingly to have them all face out/to one side.
	public DriveTrain() {
		modFrontLeft = new SwerveModule(
			Constants.FL_TURN_PORT, Constants.FL_DRIVE_PORT, Constants.FL_OFFSET, false,
			Constants.SWERVE_P_GAIN, Constants.SWERVE_I_GAIN, Constants.SWERVE_P_GAIN);

		modFrontRight = new SwerveModule(
			Constants.FR_TURN_PORT, Constants.FR_DRIVE_PORT, Constants.FR_OFFSET, true,
			Constants.SWERVE_P_GAIN, Constants.SWERVE_I_GAIN, Constants.SWERVE_P_GAIN);

		modBackLeft = new SwerveModule(
			Constants.BL_TURN_PORT, Constants.BL_DRIVE_PORT, Constants.BL_OFFSET, false,
			Constants.SWERVE_P_GAIN, Constants.SWERVE_I_GAIN, Constants.SWERVE_P_GAIN);

		modBackRight = new SwerveModule(
			Constants.BR_TURN_PORT, Constants.BR_DRIVE_PORT, Constants.BR_OFFSET, true,
			Constants.SWERVE_P_GAIN, Constants.SWERVE_I_GAIN, Constants.SWERVE_P_GAIN);

			
		frontLeftStop = new DigitalOutput(4);
		frontRightStop = new DigitalOutput(5);
		backLeftStop = new DigitalOutput(6);
		backRightStop = new DigitalOutput(7);
	}
	
	//Crab drive - sets all motors to the same angle and speed
	//Basically swerve drive without rotation
	public void crabDrive(double angle, double speed) {
		modFrontLeft.setModule(angle, speed);
		modFrontRight.setModule(angle, speed);
		modBackLeft.setModule(angle, speed);
		modBackRight.setModule(angle, speed);
	}

	//A couple of methods that work like regular tank drives
	//Not particularly useful
	public void tankDrive(double left, double right) {
		modFrontLeft.setModule(0, left);
		modFrontRight.setModule(0, right);
		modBackLeft.setModule(0, left);
		modBackRight.setModule(0, right);
	}
	public void arcadeDrive(double power, double turn) {
		modFrontLeft.setModule(0, power + turn);
		modFrontRight.setModule(0, power - turn);
		modBackLeft.setModule(0, power + turn);
		modBackRight.setModule(0, power - turn);
	}

	//Default swerve drive is field oriented
	public void swerveDrive(double forwardSpeed, double strafeSpeed, double rotateSpeed) {
		swerveDrive(forwardSpeed, strafeSpeed, rotateSpeed, true);
	}
	//Swerve drive - takes forward, sideways, and rotational speeds, and does calculations to make the robot move
	//Allows for field/robot orientation
	public void swerveDrive(double forwardSpeed, double strafeSpeed, double rotateSpeed, boolean isFieldOriented) {

		double gyroAngle = Robot.ahrs.getAngle();

		double sin = Math.sin(Math.toRadians(gyroAngle));
		double cos = Math.cos(Math.toRadians(gyroAngle));

		if(isFieldOriented) {
			double T = (forwardSpeed * cos) + (strafeSpeed * sin);
			strafeSpeed = (-forwardSpeed * sin) + (strafeSpeed * cos);
			forwardSpeed = T;
		}

		double J = Constants.WHEELBASE_INCHES / Constants.TURN_RADIUS_INCHES;
		double K = Constants.TRACKWIDTH_INCHES / Constants.TURN_RADIUS_INCHES;

		double A = strafeSpeed - (rotateSpeed * J);
		double B = strafeSpeed + (rotateSpeed * J);
		double C = forwardSpeed - (rotateSpeed * K);
		double D = forwardSpeed + (rotateSpeed * K);

		double FLspeed = Math.hypot(B, D);
		double FRspeed = Math.hypot(B, C);
		double BLspeed = Math.hypot(A, D);
		double BRspeed = Math.hypot(A, C);

		double FLangle = Math.atan2(B, D) * 180 / Math.PI;
		double FRangle = Math.atan2(B, C) * 180 / Math.PI;
		double BLangle = Math.atan2(A, D) * 180 / Math.PI;
		double BRangle = Math.atan2(A, C) * 180 / Math.PI;

		double max = Math.max(Math.max(FLspeed, FRspeed), Math.max(BLspeed, BRspeed));
		if(max > 1) {FLspeed /= max; FRspeed /= max; BLspeed /= max; BRspeed /= max;}

		modFrontLeft.setModule(FLangle, FLspeed * speedModifier);
		modFrontRight.setModule(FRangle, FRspeed * speedModifier);
		modBackLeft.setModule(BLangle, BLspeed * speedModifier);
		modBackRight.setModule(BRangle, BRspeed * speedModifier);

		SmartDashboard.putNumber("FL Speed", FLspeed);
		SmartDashboard.putNumber("FR Speed", FRspeed);
		SmartDashboard.putNumber("BR Speed", BRspeed);
		SmartDashboard.putNumber("BL Speed", BLspeed);
	}

	//Stops ALL motors from moving
	public void fullStop() {
		modFrontLeft.getDriveController().set(0);
		modFrontRight.getDriveController().set(0);
		modBackLeft.getDriveController().set(0);
		modBackRight.getDriveController().set(0);

		modFrontLeft.getTurnController().set(ControlMode.PercentOutput, 0);
		modFrontRight.getTurnController().set(ControlMode.PercentOutput, 0);
		modBackLeft.getTurnController().set(ControlMode.PercentOutput, 0);
		modBackRight.getTurnController().set(ControlMode.PercentOutput, 0);
	}

	//Resets all turn encoders - see SwerveModule.newOffset()
	public void newOffsets() {
		modFrontLeft.newOffset();
		modFrontRight.newOffset();
		modBackLeft.newOffset();
		modBackRight.newOffset();
	}
	//Resets all drive encoders
	public void resetDriveEncoders() {
		modFrontLeft.resetDriveEncoder();
		modFrontRight.resetDriveEncoder();
		modBackLeft.resetDriveEncoder();
		modBackRight.resetDriveEncoder();
	}
	
	//Average position of all drive encoders
	//Use carefully - will not necessarily be distance travelled, as some modules might turn different ways than others
	public double averageDrivePosition() {

		double sum = 0;

		if(modFrontLeft.isInverted()) {sum -= modFrontLeft.getDriveRawPosition();}
		else {sum += modFrontLeft.getDriveRawPosition();}

		if(modFrontRight.isInverted()) {sum -= modFrontRight.getDriveRawPosition();}
		else {sum += modFrontRight.getDriveRawPosition();}

		if(modBackLeft.isInverted()) {sum -= modBackLeft.getDriveRawPosition();}
		else {sum += modBackLeft.getDriveRawPosition();}

		if(modBackRight.isInverted()) {sum -= modBackRight.getDriveRawPosition();}
		else {sum += modBackRight.getDriveRawPosition();}

		return sum / 4;
	}
	
	//Sets PID gains of all modules
	//Individual tuning is recommended
	public void setAllTurnPID(double kP, double kI, double kD) {
		modFrontLeft.setTurnPID(kP, kI, kD);
		modFrontRight.setTurnPID(kP, kI, kD);
		modBackLeft.setTurnPID(kP, kI, kD);
		modBackRight.setTurnPID(kP, kI, kD);
	}

	

	// New Methods/Variables Not Made by AidanBele™  
	// 😀 <-- this is Paul

	// Turn Encoder Values
	public double getTurnFrontLeft() {
		return modFrontLeft.getTurnRawPosition();
	}

	public double getTurnFrontRight() {
		return modFrontRight.getTurnRawPosition();
	}

	public double getTurnBackLeft() {
		return modBackLeft.getTurnRawPosition();
	}

	public double getTurnBackRight() {
		return modBackRight.getTurnRawPosition();
	}


	// Drive Encoder Values
	public double getDriveFrontLeft() {
		double sum = 0;

		if(modFrontLeft.isInverted()) {sum -= modFrontLeft.getDriveRawPosition();}
		else {sum += modFrontLeft.getDriveRawPosition();}

		return sum;
	}

	public double getDriveFrontRight() {
		double sum = 0;
		
		if(modFrontRight.isInverted()) {sum -= modFrontRight.getDriveRawPosition();}
		else {sum += modFrontRight.getDriveRawPosition();}

		return sum;
	}

	public double getDriveBackLeft() {
		double sum = 0;

		if(modBackLeft.isInverted()) {sum -= modBackLeft.getDriveRawPosition();}
		else {sum += modBackLeft.getDriveRawPosition();}

		return sum;
	}

	public double getDriveBackRight() {
		double sum = 0;

		if(modBackRight.isInverted()) {sum -= modBackRight.getDriveRawPosition();}
		else {sum += modBackRight.getDriveRawPosition();}

		return sum;
	}

	// Average Encoder Feet Traveled
	public double EncFeetTraveled(){
		return averageDrivePosition() / Constants.AVERAGE_FOOT;
	}
	
	// Absolute Average Drive Position
	public double absoluteAverageDrivePosition() {
		double sum = 0;

		sum += Math.abs(modFrontLeft.getDriveRawPosition());
		sum += Math.abs(modFrontRight.getDriveRawPosition());
		sum += Math.abs(modBackLeft.getDriveRawPosition());
		sum += Math.abs(modBackRight.getDriveRawPosition());

		return sum / 4;
	}

	// Absolute Average Encoder Feet Traveled
	public double absEncFeetTraveled() {
		return absoluteAverageDrivePosition() / Constants.AVERAGE_FOOT;
	}

	public double setSpeedModifier(double mod) {
		speedModifier = mod;
		return speedModifier;
	}

	@Override
	public void periodic(){
		// Gets average drive encoder positions
		SmartDashboard.putNumber("Average Drive Enconder Value", averageDrivePosition());
    
		// Gets individual drive encoder positions
		SmartDashboard.putNumber("Front Left Module DRIVE ENCODER", getDriveFrontLeft());
		SmartDashboard.putNumber("Front Right Module DRIVE ENCODER", getDriveFrontRight());
		SmartDashboard.putNumber("Back Left Module DRIVE ENCODER", getDriveBackLeft());
		SmartDashboard.putNumber("Back Right Module DRIVE ENCODER", getDriveBackRight());
		
		// Absolute Drive Enc Feet Traveled
		SmartDashboard.putNumber("Absolute Encoder Feet Traveled", absoluteAverageDrivePosition());
	
		SmartDashboard.putNumber("Speed Modifier", speedModifier);
	}

	public int round360(int n) { 
		int a = (n / 360) * 360; 
		int b = a + 360; 
			
		return (n - a > b - n) ? b : a; 
	  } 
	
	  public int round180(int n) {
		int a = (n / 180) * 180; 
		int b = a + 180; 
		  
		return (n - a > b - n) ? b : a; 
	  }

	public void turnAllModules(double power) {
		modFrontLeft.getTurnController().set(ControlMode.PercentOutput, power);
		modFrontRight.getTurnController().set(ControlMode.PercentOutput, power);
		modBackLeft.getTurnController().set(ControlMode.PercentOutput, power);
		modBackRight.getTurnController().set(ControlMode.PercentOutput, power);
	}

	public void turnFrontLeft(double power) {
		modFrontLeft.getTurnController().set(ControlMode.PercentOutput, power);
	}

	public void turnFrontRight(double power) {
		modFrontRight.getTurnController().set(ControlMode.PercentOutput, power);
	}

	public void turnBackLeft(double power) {
		modBackLeft.getTurnController().set(ControlMode.PercentOutput, power);
	}

	public void turnBackRight(double power) {
		modBackRight.getTurnController().set(ControlMode.PercentOutput, power);
	}

	public void stopAllTurnModules() {
		modFrontLeft.getTurnController().set(ControlMode.PercentOutput, 0);
		modFrontRight.getTurnController().set(ControlMode.PercentOutput, 0);
		modBackLeft.getTurnController().set(ControlMode.PercentOutput, 0);
		modBackRight.getTurnController().set(ControlMode.PercentOutput, 0);
	}
}