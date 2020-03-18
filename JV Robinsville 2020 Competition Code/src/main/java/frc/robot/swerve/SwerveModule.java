package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

//An individual module for swerve drive, can turn the wheel to certain angles at certan speeds
//Consists of a Talon SRX for the turn motor, and a Spark MAX for the drive motor
public class SwerveModule {

	private final WPI_TalonSRX turn;
	private final CANSparkMax drive;

	private boolean isInverted;
	private double offset;

	/**
     * @param turnID   		the ID of the turn motor
     * @param driveID       the ID of the drive motor
     * @param isInverted    if the module is physically reversed on the robot
     * @param offset		encoder value when wheel is pointing stright
     */

	 // Old Swerve Module Object (it sets all modules to one set of PID values)
	 public SwerveModule(int turnID, int driveID, double offset, boolean isInverted) {
		turn = new WPI_TalonSRX(turnID);
		drive = new CANSparkMax(driveID, MotorType.kBrushless);
		this.offset = offset;
		this.isInverted = isInverted;

		turn.configFactoryDefault();
		turn.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, Constants.SRX_PIDLOOPIDX, Constants.SRX_TIMEOUT_MS);
		turn.config_kP(Constants.SRX_PIDLOOPIDX, Constants.SWERVE_P_GAIN);
		turn.config_kI(Constants.SRX_PIDLOOPIDX, Constants.SWERVE_I_GAIN);
		turn.config_kD(Constants.SRX_PIDLOOPIDX, Constants.SWERVE_D_GAIN);
		turn.setNeutralMode(NeutralMode.Brake);
	}

	public SwerveModule(int turnID, int driveID, double offset, boolean isInverted, double pVal, double iVal, double dVal) {
		turn = new WPI_TalonSRX(turnID);
		drive = new CANSparkMax(driveID, MotorType.kBrushless);
		this.offset = offset;
		this.isInverted = isInverted;

		turn.configFactoryDefault();
		turn.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, Constants.SRX_PIDLOOPIDX, Constants.SRX_TIMEOUT_MS);
		turn.config_kP(Constants.SRX_PIDLOOPIDX, pVal);
		turn.config_kI(Constants.SRX_PIDLOOPIDX, iVal);
		turn.config_kD(Constants.SRX_PIDLOOPIDX, dVal);
		turn.setNeutralMode(NeutralMode.Brake);
	}
	
	//Wheel spins in reverse if the module is reversed
	public void spinWheel(double speed) {
		if(isInverted) {
			drive.set(-speed);
		}
		else {
			drive.set(speed);
		}
	}

	//Main function to set a module
	//targetAngle is in degrees - 0 is forward
	//targetSpeed is from -1.0 to 1.0
	public void setModule(double targetAngle, double targetSpeed) {

		double target, current, speed;
			
		target = targetAngle;
		current = getTurnDegrees();
		speed = targetSpeed;

		//Most of this is optimizing the movement of the turn motor -
		//Sometimes you will want to reverse the drive motor instead of turning another 180 degrees
		while(current > 180) current -= 360;
		while(current < -180) current += 360;

		while(target > 180) target -= 360;
		while(target < -180) target += 360;

		double error = target - current;

		if(Math.abs(error) > 180.0) {
			error -= 360.0 * Math.signum(error);
		}

		if(Math.abs(error) > 90.0) {
			error -= 180.0 * Math.signum(error);
			speed *= -1;
		}

		//Gives the target position to the built-in PID controller on the Talon SRX
		turn.set(ControlMode.Position, getTurnRawPosition() + (error * Constants.MODULE_FULL_ROTATION / 360));
		//May want to use a velocity PID instead of raw voltage, but this is perfectly fine for now
		spinWheel(speed);
	}
	
	//The raw encoder position of the turn motor
	public double getTurnRawPosition() {
		return turn.getSelectedSensorPosition(Constants.SRX_PIDLOOPIDX);
	}
	//Turn encoder adjusted with given offset
	public double getTurnAdjPosition() {
		return getTurnRawPosition() - offset;
	}
	//Converted to degrees from encoder ticks
	public double getTurnDegrees() {
		return getTurnAdjPosition() / Constants.MODULE_FULL_ROTATION * 360;
	}

	//The raw encoder position of the drive motor
	public double getDriveRawPosition() {
		return drive.getEncoder().getPosition();
	}
	//The raw encoder velocity of the turn motor
	public double getDriveVelocity() {
		return drive.getEncoder().getVelocity();
	}


	//Re-offsets the turn encoder - useful if you want to recenter at the beginning of a match
	//Basically works like resetting the encoder
	public void newOffset() {
		this.offset = getTurnRawPosition();
	}
	//Resets the drive encoder
	public void resetDriveEncoder() {
		drive.getEncoder().setPosition(0);
	}

	
	
	//Sets PID gains for turn motor
	public void setTurnPID(double kP, double kI, double kD) {
		turn.config_kP(Constants.SRX_PIDLOOPIDX, kP);
		turn.config_kI(Constants.SRX_PIDLOOPIDX, kI);
		turn.config_kD(Constants.SRX_PIDLOOPIDX, kD);
	}

	//Returns true if the drive motor is inverted
	public boolean isInverted() {
		return isInverted;
	}

	public TalonSRX getTurnController() {
		return turn;
	}
	public CANSparkMax getDriveController() {
		return drive;
	}
	
}