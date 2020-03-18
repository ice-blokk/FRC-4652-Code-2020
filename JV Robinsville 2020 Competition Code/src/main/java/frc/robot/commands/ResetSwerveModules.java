/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ResetSwerveModules extends CommandBase {
  
  private final DriveTrain drivetrain;
  double power = .3;

  public ResetSwerveModules(DriveTrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drivetrain.frontLeftStop.get()) {
      drivetrain.turnFrontLeft(power);
    }
    else {
      drivetrain.turnFrontLeft(0);
    }

    if(drivetrain.frontRightStop.get()) {
      drivetrain.turnFrontRight(power);
    }
    else {
      drivetrain.turnFrontRight(0);
    }

    if(drivetrain.backLeftStop.get()) {
      drivetrain.turnBackLeft(power);
    }
    else {
      drivetrain.turnBackLeft(0);
    }

    if(drivetrain.backRightStop.get()) {
      drivetrain.turnBackRight(power);
    }
    else {
      drivetrain.turnBackRight(0);
    }

    Shuffleboard.getTab("Competition").add("Front Left Stop", drivetrain.frontLeftStop.get());
    Shuffleboard.getTab("Competition").add("Front Right Stop", drivetrain.frontRightStop.get());
    Shuffleboard.getTab("Competition").add("Back Left Stop", drivetrain.backLeftStop.get());
    Shuffleboard.getTab("Competition").add("Back Right Stop", drivetrain.backRightStop.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopAllTurnModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
