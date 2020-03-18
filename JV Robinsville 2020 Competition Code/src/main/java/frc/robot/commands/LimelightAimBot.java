/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Limelight;

public class LimelightAimBot extends CommandBase {
  
  private final DriveTrain drivetrain;
  private final Limelight limelight;
  private final AHRS ahrs;
  double forwardOffset, strafeOffset, turnOffset, targetAngle;

  public LimelightAimBot(DriveTrain drivetrain, Limelight limelight, AHRS ahrs) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.ahrs = ahrs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.LEDon();
    targetAngle = ahrs.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    strafeOffset =  (2.1 * limelight.getX()) / 100;
    forwardOffset = (2.1 * -limelight.getY() / 100);
    double angleOffset = (targetAngle / 100) - (ahrs.getAngle() / 100);

    drivetrain.swerveDrive(forwardOffset, strafeOffset, angleOffset);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.LEDoff();
    drivetrain.swerveDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
