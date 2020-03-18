/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoGyroForward extends CommandBase {
  
  private final DriveTrain drivetrain;
  private final AHRS ahrs;
  private final double forwardSpeed, distance;
  private double targetAngle;

  public AutoGyroForward(DriveTrain drivetrain, AHRS ahrs,
  double forwardSpeed, double distance) {
    this.drivetrain = drivetrain;
    this.ahrs = ahrs;
    this.forwardSpeed = forwardSpeed;
    this.distance = distance;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetDriveEncoders();
    targetAngle = ahrs.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleOffset = (targetAngle / 100) - (ahrs.getAngle() / 100);
    drivetrain.swerveDrive(forwardSpeed, 0, angleOffset);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.swerveDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.absEncFeetTraveled() >= distance;
  }
}
