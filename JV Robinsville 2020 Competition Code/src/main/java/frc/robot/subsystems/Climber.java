/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  
  public final WPI_TalonSRX hookMotor;
  public final CANSparkMax winchMotor;
  public final CANSparkMax carl;
  private Encoder hookEncoder;
  private double winchEncoderValue, hookEncoderValue;

  public Climber() {

    hookMotor = new WPI_TalonSRX(Constants.CLIMB_MOTOR_PORT); //grabby boi
    winchMotor = new CANSparkMax(Constants.WINCH_MOTOR_PORT, MotorType.kBrushless); // pully boi
    carl = new CANSparkMax(Constants.CRAWL_MOTOR_PORT, MotorType.kBrushless); // Carl the crawler
    hookEncoder = new Encoder(0, 1, false, EncodingType.k1X); // useless boi
  }

  public void setWinchMotor(double power) {
    winchMotor.set(power);
  }

  public void setHookMotor(double power) {
    hookMotor.set(power);
  }

  public void stopClimberMotor() {
    hookMotor.set(0);
    winchMotor.set(0);
  }

  public void setCarl(double power) {
    carl.set(power);
  }

  @Override
  public void periodic() {
    
    winchEncoderValue = winchMotor.getEncoder().getPosition();
    hookEncoderValue = hookEncoder.getRaw();

    SmartDashboard.putNumber("Winch Encoder Value", winchEncoderValue);
    SmartDashboard.putNumber("Hook Encoder Value", hookEncoderValue);

  }
}
 