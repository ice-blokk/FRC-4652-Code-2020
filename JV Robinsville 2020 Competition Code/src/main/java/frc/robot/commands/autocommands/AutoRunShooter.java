/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class AutoRunShooter extends CommandBase {
  
  private final Shooter shooter;
  double startingPower;

  public AutoRunShooter(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPower = -.6;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(shooter.shooterMotor.getEncoder().getVelocity()) < 3425) {
      startingPower -= .0007;
    }
    else if(Math.abs(shooter.shooterMotor.getEncoder().getVelocity()) > 3455) {
      startingPower += .0007;
    }

    shooter.setShooterMotor(startingPower);

    SmartDashboard.putNumber("Shooter Motor Velocity", shooter.shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Power", shooter.shooterMotor.get());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(shooter.shooterMotor.getEncoder().getVelocity()) > 3425 && Math.abs(shooter.shooterMotor.getEncoder().getVelocity()) < 3455;
  }
}
