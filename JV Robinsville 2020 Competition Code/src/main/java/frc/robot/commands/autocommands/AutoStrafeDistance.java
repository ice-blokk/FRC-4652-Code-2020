/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoStrafeDistance extends CommandBase {
  
  private final DriveTrain drivetrain;
  private final double strafeSpeed;
  private final double distance;

  /**
   *  @param strafeSpeed   Positive is to the right, negative is to the left  
   */

  public AutoStrafeDistance(DriveTrain drivetrain, double strafeSpeed, double distance) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.strafeSpeed = strafeSpeed;
    this.distance = distance;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetDriveEncoders();
    drivetrain.swerveDrive(0, strafeSpeed, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
