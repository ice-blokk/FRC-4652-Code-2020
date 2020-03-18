/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ColorWheelColor;
import frc.robot.commands.ColorWheelRotation;
import frc.robot.commands.FieldOrientBackward;
import frc.robot.commands.FieldOrientForward;
import frc.robot.commands.GyroDrive;
import frc.robot.commands.LimelightAimBot;
import frc.robot.commands.ModifySwerveSpeed;
import frc.robot.commands.ResetSwerveModules;
import frc.robot.commands.RunBoxDown;
import frc.robot.commands.RunBoxUp;
import frc.robot.commands.RunCarlLeft;
import frc.robot.commands.RunCarlRight;
import frc.robot.commands.RunColorWheelElevator;
import frc.robot.commands.RunShooter;
import frc.robot.commands.ShooterGateOpen;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.autocommands.AutoGyroForward;
import frc.robot.commands.autocommands.AutoGyroDriveAndStrafe;
import frc.robot.commands.autocommands.AutoDriveForwardDistance;
import frc.robot.commands.autocommands.AutoGyroStrafe;
import frc.robot.commands.autocommands.AutoLimelightAlign;
import frc.robot.commands.autocommands.AutoRunShooter;
import frc.robot.commands.autocommands.AutoStrafeDistance;
import frc.robot.subsystems.Box;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.util.AutonomousChooser;
import frc.robot.util.LeftTriggerButton;
import frc.robot.util.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;  
import edu.wpi.first.wpilibj2.command.button.POVButton;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveTrain drivetrain;
  private final Climber climber;
  private final Box box;
  private final ColorWheel colorwheel;
  private final Shooter shooter;

  private final XboxController driver;
  private final XboxController operator;

  private final Limelight limelight;

  private SendableChooser<SequentialCommandGroup> chooser = new SendableChooser<>();

  public final SequentialCommandGroup 
    None, 
    //TestMoveAndStrafe,
    InitiationLine,

    StraightScoreLowGoal, 
    RightScoreLowGoal, 
    MiddleScoreLowGoal,
    LeftScoreLowGoal,

    StraightScoreLowGoalStay,
    RightScoreLowGoalStay,
    MiddleScoreLowGoalStay,
    LeftScoreLowGoalStay,

    StraightScoreLowGoalLeft,
    RightScoreLowGoalLeft,
    MiddleScoreLowGoalLeft,
    LeftScoreLowGoalLeft,

    ForwardLimelightShoot,
    LimelightShoot,
    TwoFeetLeftShoot,
    Test;

    /*LeftLimelightAutoAlign,
    LimelightAutoAlignAndleave*/
    

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    drivetrain = new DriveTrain();
    climber = new Climber();
    box = new Box();
    colorwheel = new ColorWheel();
    shooter = new Shooter();

    driver = new XboxController(Constants.DRIVER_PORT);
    operator = new XboxController(Constants.OPERATOR_PORT);

    limelight = new Limelight();

    box.resetAllServos();
    drivetrain.newOffsets();
    drivetrain.resetDriveEncoders();
    limelight.LEDoff();

    setDefaultCommands();
    configureButtonBindings();

    // Autonomous Commands
    None = null;

    InitiationLine = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs, .5, 3)
    );
    /*
    TestMoveAndStrafe = new SequentialCommandGroup(
      new AutoGyroDriveAndStrafe(drivetrain, Robot.ahrs, .25, 0, 5.0),
      new AutoGyroDriveAndStrafe(drivetrain, Robot.ahrs, 0, -.20, 10)
    );
    */
    StraightScoreLowGoal = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs, .40, 7.55),
      new InstantCommand(() -> box.down(), box),
      new WaitCommand(3),
      new ParallelCommandGroup(
          new InstantCommand(() -> box.resetAllServos(), box),
          new AutoGyroForward(drivetrain, Robot.ahrs, -.40, 2.5)),
      new AutoGyroStrafe(drivetrain, Robot.ahrs, .40, 5.7)
      );

      RightScoreLowGoal = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 5),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  -.4, 5.8),
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 2.55),
      new InstantCommand(() -> box.down(), box),
      new WaitCommand(3),
      new ParallelCommandGroup(
          new InstantCommand(() -> box.resetAllServos(), box),
          new AutoGyroForward(drivetrain, Robot.ahrs, -.4, 2.5)),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  .4, 5.7)
    );

    MiddleScoreLowGoal = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 5),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  .4, 4.3),
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 2.55),
      new InstantCommand(() -> box.down(), box),
      new WaitCommand(3),
      new ParallelCommandGroup(
          new InstantCommand(() -> box.resetAllServos(), box),
          new AutoGyroForward(drivetrain, Robot.ahrs, -.4, 2.55)),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  .4, 5.7)
    );

    // The first strafe to the right is an approximation
    LeftScoreLowGoal = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs,  .4, 5),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  .4, 19.75),
      new AutoGyroForward(drivetrain, Robot.ahrs,  .4, 2.55),
      new InstantCommand(() -> box.down(), box),
      new WaitCommand(3),
      new ParallelCommandGroup(
          new InstantCommand(() -> box.resetAllServos(), box),
          new AutoGyroForward(drivetrain, Robot.ahrs,  -.4, 2.5)),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  .4, 5.7)
    );


    StraightScoreLowGoalStay = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs,  .4, 7.5),
      new InstantCommand(() -> box.down(), box),
      new WaitCommand(3),
      new InstantCommand(() -> box.resetAllServos(), box)
    );

    RightScoreLowGoalStay = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs,  .4, 5),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  -.4, 5.8),
      new AutoGyroForward(drivetrain, Robot.ahrs,  .4, 2.55),
      new InstantCommand(() -> box.down(), box),
      new WaitCommand(3),
      new InstantCommand(() -> box.resetAllServos(), box)
    );

    MiddleScoreLowGoalStay = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 5),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  .4, 4.3),
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 2.55),
      new InstantCommand(() -> box.down(), box),
      new WaitCommand(3),
      new InstantCommand(() -> box.resetAllServos(), box)
    );

    LeftScoreLowGoalStay = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs,  .4, 5),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  .4, 19.75),
      new AutoGyroForward(drivetrain, Robot.ahrs,  .4, 2.55),
      new InstantCommand(() -> box.down(), box),
      new WaitCommand(3),
      new InstantCommand(() -> box.resetAllServos(), box)
    );


    StraightScoreLowGoalLeft = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs, .40, 7.55),
      new InstantCommand(() -> box.down(), box),
      new WaitCommand(3),
      new ParallelCommandGroup(
          new InstantCommand(() -> box.resetAllServos(), box),
          new AutoGyroForward(drivetrain, Robot.ahrs, -.40, 2.5)),
      new AutoGyroStrafe(drivetrain, Robot.ahrs, -.40, 5.7)
      );

      RightScoreLowGoalLeft = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 5),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  -.4, 5.8),
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 2.55),
      new InstantCommand(() -> box.down(), box),
      new WaitCommand(3),
      new ParallelCommandGroup(
          new InstantCommand(() -> box.resetAllServos(), box),
          new AutoGyroForward(drivetrain, Robot.ahrs, -.4, 2.5)),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  -.4, 5.7)
    );

    MiddleScoreLowGoalLeft = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 5),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  .4, 4.3),
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 2.55),
      new InstantCommand(() -> box.down(), box),
      new WaitCommand(3),
      new ParallelCommandGroup(
          new InstantCommand(() -> box.resetAllServos(), box),
          new AutoGyroForward(drivetrain, Robot.ahrs, -.4, 2.55)),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  -.4, 5.7)
    );

    // The first strafe to the right is an approximation
    LeftScoreLowGoalLeft = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs,  .4, 5),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  .4, 19.75),
      new AutoGyroForward(drivetrain, Robot.ahrs,  .4, 2.55),
      new InstantCommand(() -> box.down(), box),
      new WaitCommand(3),
      new ParallelCommandGroup(
          new InstantCommand(() -> box.resetAllServos(), box),
          new AutoGyroForward(drivetrain, Robot.ahrs,  -.4, 2.5)),
      new AutoGyroStrafe(drivetrain, Robot.ahrs,  -.4, 5.7)
    );

    ForwardLimelightShoot = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 3.5),
      new AutoLimelightAlign(drivetrain, limelight, Robot.ahrs),
      new AutoRunShooter(shooter),
      new InstantCommand(() -> box.shootGateOpen(), box),
      new InstantCommand(() -> box.tiltUp(), box),
      new WaitCommand(5),
      new InstantCommand(() -> shooter.setShooterMotor(0), shooter),
      new InstantCommand(() -> box.resetAllServos())
    );

    LimelightShoot = new SequentialCommandGroup(
      new AutoLimelightAlign(drivetrain, limelight, Robot.ahrs),
      new AutoRunShooter(shooter),
      new InstantCommand(() -> box.shootGateOpen(), box),
      new InstantCommand(() -> box.tiltUp(), box),
      new WaitCommand(5),
      new InstantCommand(() -> shooter.setShooterMotor(0), shooter),
      new InstantCommand(() -> box.resetAllServos())
    );

    TwoFeetLeftShoot = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs, .2, 4.5),
      new AutoGyroStrafe(drivetrain, Robot.ahrs, .2, 3.5),
      new AutoLimelightAlign(drivetrain, limelight, Robot.ahrs),
      new AutoRunShooter(shooter),
      new InstantCommand(() -> box.shootGateOpen(), box),
      new InstantCommand(() -> box.tiltUp(), box),
      new WaitCommand(5),
      new InstantCommand(() -> shooter.setShooterMotor(0), shooter),
      new InstantCommand(() -> box.resetAllServos())
    );

    Test = new SequentialCommandGroup(
      new AutoRunShooter(shooter),
      new WaitCommand(3),
      new InstantCommand(() -> shooter.setShooterMotor(0), shooter)
    );

//The wheels seem to break everytime I excecute this command. I have'nt tried it with other commands. Goes off course and needs desperate therapy.
    /*LeftLimelightAutoAlign = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 5.0),
      new AutoGyroStrafe(drivetrain, Robot.ahrs, .4, 20.75),
      new AutoLimelightAlign(drivetrain, limelight),
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 2.55),
      new ParallelCommandGroup(
        new InstantCommand(() -> box.down(), box),
        new AutoGyroForward(drivetrain, Robot.ahrs, -.4, 2.55)),
      new AutoGyroStrafe(drivetrain, Robot.ahrs, .4, 5.7)
    );*/

    /*LimelightAutoAlignAndleave = new SequentialCommandGroup(
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 5.0),
      new AutoGyroStrafe(drivetrain, Robot.ahrs, .4, 20.75),
      new AutoLimelightAlign(drivetrain, limelight),
      new AutoGyroForward(drivetrain, Robot.ahrs, .4, 4.0),
      new InstantCommand(() -> box.down(), box),
      new AutoGyroStrafe(drivetrain, Robot.ahrs, .4, 5.0)
    );*/

    // Autonomous Chooser
    chooser.setDefaultOption("None", None);
    //chooser.addOption("Test Move And Strafe", TestMoveAndStrafe);
    chooser.addOption("Initiation Line", InitiationLine);
    chooser.addOption("Straight Score Low Goal", StraightScoreLowGoal);
    chooser.addOption("Right Score Low Goal", RightScoreLowGoal);
    chooser.addOption("Middle Score Low Goal", MiddleScoreLowGoal);
    chooser.addOption("Left Score Low Goal", LeftScoreLowGoal);
    chooser.addOption("Straight Score Low Goal Stay", StraightScoreLowGoalStay);
    chooser.addOption("Right Score Low Goal Stay", RightScoreLowGoalStay);
    chooser.addOption("Middle Score Low Goal Stay", MiddleScoreLowGoalStay);
    chooser.addOption("Left Score Low Goal Stay", LeftScoreLowGoalStay);
    chooser.addOption("Forward Limelight Shoot", ForwardLimelightShoot);
    chooser.addOption("Limelight Shoot", LimelightShoot);
    chooser.addOption("Two Feet Left Shoot", TwoFeetLeftShoot);
    chooser.addOption("Test", Test);
    //chooser.addOption("Left Limelight Auto Align", LeftLimelightAutoAlign);
    //chooser.addOption("Limelight Auto Align And Leave", LimelightAutoAlignAndleave);

    // Puts Autonomous Chooser on SmartDashboard
    Shuffleboard.getTab("Competition").add("Autonomous Choices", chooser);
  }

  public void setDefaultCommands() {
    drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, true,
                                () -> driver.getY(Hand.kLeft),
                                () -> driver.getX(Hand.kLeft),
                                () -> driver.getX(Hand.kRight)));
    climber.setDefaultCommand(new RunCommand(() -> climber.stopClimberMotor(), climber));
    //box.setDefaultCommand(new RunCommand(() -> box.resetAllServos(), box));
    colorwheel.setDefaultCommand(new RunColorWheelElevator(colorwheel, 
                                () -> operator.getY(Hand.kLeft)));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    final JoystickButton dA, dB, dX, dY, dLB, dRB, dSelect, dMenu;
    final JoystickButton oA, oB, oX, oY, oRB, oLB, oSelect, oMenu, o9, o10;

    dA = new JoystickButton(driver, 1);
    dB = new JoystickButton(driver, 2);
    dX = new JoystickButton(driver, 3);
    dY = new JoystickButton(driver, 4);
    dLB = new JoystickButton(driver, 5);
    dRB = new JoystickButton(driver, 6);
    dMenu = new JoystickButton(driver, 7);
    dSelect = new JoystickButton(driver, 8);

    oA = new JoystickButton(operator, 1);
    oB = new JoystickButton(operator, 2);
    oX = new JoystickButton(operator, 3);
    oY = new JoystickButton(operator, 4);
    oLB = new JoystickButton(operator, 5);
    oRB = new JoystickButton(operator, 6);
    oMenu = new JoystickButton(operator, 7);
    oSelect = new JoystickButton(operator, 8);
    o9 = new JoystickButton(operator, 9);
    o10 = new JoystickButton(operator, 10);
    
    Command runWinchUp = new RunCommand(() -> climber.setHookMotor(-.7), climber);
    Command runWinchDown = new RunCommand(() -> climber.setHookMotor(.7), climber);
    Command runHookUp = new RunCommand(() -> climber.setWinchMotor(-.8), climber);
    Command runHookDown = new RunCommand(() -> climber.setWinchMotor(.8), climber);

    oY.whileHeld(runWinchUp);
    oA.whileHeld(runWinchDown);
    oB.whileHeld(runHookUp);
    oX.whileHeld(runHookDown);

    new POVButton(operator, 0).whileHeld(new RunBoxUp(box));
    new POVButton(operator, 180).whileHeld(new RunBoxDown(box));
 
    oLB.whenHeld(new ColorWheelColor(colorwheel));
    oRB.whenHeld(new ColorWheelRotation(colorwheel));

    new POVButton(operator, 90).whileHeld(new RunCarlRight(climber));
    new POVButton(operator, 270).whileHeld(new RunCarlLeft(climber));

    oMenu.whenHeld(new ShooterGateOpen(box));
    oSelect.whenHeld(new RunShooter(shooter));

    dMenu.whenPressed(new InstantCommand(() -> drivetrain.newOffsets(), drivetrain));
    dMenu.whenPressed(new PrintCommand("New Offset Set"));
    dSelect.whenPressed(new ResetSwerveModules(drivetrain));

    dX.whenPressed(new InstantCommand(() -> Robot.ahrs.reset()));
    
    dRB.whenHeld(new LimelightAimBot(drivetrain, limelight, Robot.ahrs));
    
    dLB.whenHeld(new GyroDrive(drivetrain, Robot.ahrs, 
    () -> driver.getY(Hand.kLeft), 
    () -> driver.getX(Hand.kLeft)));

    /*
    dY.whenHeld(new FieldOrientForward(drivetrain, Robot.ahrs, 
    () -> driver.getY(Hand.kLeft), 
    () -> driver.getX(Hand.kLeft)));

    dA.whenHeld(new FieldOrientBackward(drivetrain, Robot.ahrs, 
    () -> driver.getY(Hand.kLeft), 
    () -> driver.getX(Hand.kLeft)));
    */
    dRB.whenHeld(new AutoLimelightAlign(drivetrain, limelight, Robot.ahrs));
    
    //dLB.whenHeld(new GyroDrive(drivetrain, Robot.ahrs, .2, 0));
    //dRB.whenHeld(new GyroDrive(drivetrain, Robot.ahrs, 0, .2));
    //new LeftTriggerButton(driver, .80).whenHeld(new ModifySwerveSpeed(drivetrain));

  }

  public void resetAllDriveEncodersRT() {
    drivetrain.resetDriveEncoders();
  }

  public void getLimelight() {
    SmartDashboard.putNumber("Limelight X", limelight.getX());
    SmartDashboard.putNumber("Limelight Y", limelight.getY());
    SmartDashboard.putNumber("Limelight Area", limelight.getArea());
  }

  /*public void colorsensor() {
    colorwheel.printColorSensor();
  }*/

  /** 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public SequentialCommandGroup getAutonomousCommand() {
    return chooser.getSelected();
  }


}
