package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

//The command running swerve drive using a single stick
public class SwerveDrive extends CommandBase {

  private final DriveTrain drivetrain;
  private final DoubleSupplier forwardSpeed, strafeSpeed, rotateSpeed;
  private final boolean isFieldOriented;

    public SwerveDrive(DriveTrain drivetrain, boolean isFieldOriented, 
    DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed, DoubleSupplier rotateSpeed) {
      this.drivetrain = drivetrain;
      this.isFieldOriented = isFieldOriented;
      this.forwardSpeed = forwardSpeed;
      this.strafeSpeed = strafeSpeed;
      this.rotateSpeed = rotateSpeed;
      addRequirements(drivetrain);
    }

    public void execute() {

      double curvedForwardSpeed, curvedStrafeSpeed, curvedRotateSpeed;
      
      double xForwardSpeed = Constants.filter(forwardSpeed.getAsDouble());
      double xStrafeSpeed = Constants.filter(strafeSpeed.getAsDouble());
      double xRotateSpeed = Constants.filter(rotateSpeed.getAsDouble());

      curvedForwardSpeed = Math.copySign(xForwardSpeed * xForwardSpeed * xForwardSpeed * xForwardSpeed, forwardSpeed.getAsDouble());
      curvedStrafeSpeed = Math.copySign(xStrafeSpeed * xStrafeSpeed * xStrafeSpeed * xStrafeSpeed, strafeSpeed.getAsDouble());
      curvedRotateSpeed = Math.copySign(xRotateSpeed * xRotateSpeed, rotateSpeed.getAsDouble());

      drivetrain.swerveDrive(-curvedForwardSpeed, curvedStrafeSpeed, xRotateSpeed, isFieldOriented);

      SmartDashboard.putNumber("Forward Speed", curvedForwardSpeed);
      SmartDashboard.putNumber("Strafe Speed", curvedStrafeSpeed);
      SmartDashboard.putNumber("Rotate Speed", curvedRotateSpeed);

    }

    public boolean isFinished() {
        return false;
    }

    public void end() {
    }

    public void interrupted() {
	}
	
}
