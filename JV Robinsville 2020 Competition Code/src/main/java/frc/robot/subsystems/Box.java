/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Box extends SubsystemBase {

  public final Servo tiltLeft, tiltRight, dumpLeft, dumpRight, shooterGate, shooterTilt;

  public Box() {
    
    tiltLeft = new Servo(Constants.TILT_LEFT_PORT);
    tiltRight = new Servo(Constants.TILT_RIGHT_PORT);
    dumpLeft = new Servo(Constants.DUMP_LEFT_PORT);
    dumpRight = new Servo(Constants.DUMP_RIGHT_PORT);
    shooterGate = new Servo(Constants.SHOOT_GATE_PORT);
    shooterTilt = new Servo(Constants.SHOOT_TILT_PORT);

  }

  public void tiltCenter() {
    tiltLeft.setAngle(90);
    tiltRight.setAngle(90);
  } 

  public void tiltUp() {
    tiltLeft.setAngle(50);
    tiltRight.setAngle(130);
  }

  public void tiltDown() {
    tiltLeft.setAngle(130);
    tiltRight.setAngle(50);
  }

  public void dumpOpen() {
    dumpLeft.setAngle(180);
    dumpRight.setAngle(90);
  }

  public void dumpClose() {
    dumpLeft.setAngle(0);
    dumpRight.setAngle(180);
  }

  public void resetAllServos() {
    tiltLeft.setAngle(95);
    tiltRight.setAngle(85);
    dumpLeft.setAngle(0);
    dumpRight.setAngle(180);
    shooterGate.setAngle(0);
    shooterTilt.setAngle(120);
  }


  public void up() {
    tiltUp();
    dumpOpen();
  }
  
  public void down() {
    tiltDown();
    dumpOpen();
  }

  public void shootGateOpen() {
    shooterGate.setAngle(120);
    shooterTilt.setAngle(0);
  }

  public void shootGateClose() {
    shooterGate.setAngle(0);
    shooterTilt.setAngle(120);
  }

  @Override
  public void periodic() {
  }
}
