/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class FieldOrientBackward extends CommandBase {
  private final DriveTrain drivetrain;
  private final AHRS ahrs;
  private final DoubleSupplier forwardSpeed, strafeSpeed;
  private double targetAngle;

  public FieldOrientBackward(DriveTrain drivetrain, AHRS ahrs, 
  DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed) {
    this.drivetrain = drivetrain;
    this.ahrs = ahrs;
    this.forwardSpeed = forwardSpeed;
    this.strafeSpeed = strafeSpeed;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAngle = drivetrain.round180((int)ahrs.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnDirection = ahrs.getAngle() < targetAngle ? 1 : -1;
    double angleOffset = targetAngle - ahrs.getAngle();
    double pTurn;
    if(Math.abs(angleOffset) > 90) {
      pTurn = .9 * turnDirection;
    }
    else {
      pTurn = ((angleOffset / 100) - (ahrs.getAngle() / 100));
    }

    double xForwardSpeed = Constants.filter(forwardSpeed.getAsDouble());
    double xStrafeSpeed = Constants.filter(strafeSpeed.getAsDouble());

    double curvedForwardSpeed = Math.copySign(xForwardSpeed * xForwardSpeed * xForwardSpeed * xForwardSpeed, forwardSpeed.getAsDouble());
    double curvedStrafeSpeed = Math.copySign(xStrafeSpeed * xStrafeSpeed * xStrafeSpeed * xStrafeSpeed, strafeSpeed.getAsDouble());

    drivetrain.swerveDrive(-curvedForwardSpeed, curvedStrafeSpeed, pTurn);

    SmartDashboard.putNumber("pTurn FieldOrientForward", pTurn);
    SmartDashboard.putNumber("angleOffset FieldOrientForward", angleOffset);
    SmartDashboard.putNumber("targetAngle FieldOrientForward", targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.swerveDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
