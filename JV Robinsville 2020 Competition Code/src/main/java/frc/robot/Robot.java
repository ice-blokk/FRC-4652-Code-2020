package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

//This project uses commands/subsystems, but not the entire WPILib "Command Based" structure
//You may want to follow the documentation and create a RobotContainer, following their directions
//On the other hand, command based may not be useful for any given robot
public class Robot extends TimedRobot {

	public static AHRS ahrs;
	public static Timer time;

	RobotContainer rt;
	Command auto;
	WaitCommand autoWait;
	ShuffleboardTab comp = Shuffleboard.getTab("Competition");
	NetworkTableEntry autoDelay = comp.add("Autonomous Delay", 0).getEntry();

	@Override
	public void robotInit() {
	ahrs = new AHRS(SPI.Port.kMXP);
	rt = new RobotContainer();

	ahrs.reset();
	
	time = new Timer();

	comp.add("Gyro Angle", (Sendable) ahrs);
	}

	
	@Override
	public void robotPeriodic() {
		//SmartDashboard.putNumber("angle boi", ahrs.getAngle());
		rt.getLimelight();
		//rt.colorsensor();
		CommandScheduler.getInstance().run();

	}	

	@Override
	public void autonomousInit() {

		double delay = autoDelay.getDouble(0);

		// Gets Auto Command from the Chooser
		auto = rt.getAutonomousCommand();

		rt.resetAllDriveEncodersRT();

		// Schedules Auto Command
		if (auto != null) {
			Timer.delay(delay);
			auto.schedule();
		}
		else {
			System.out.println("[INFO]: No Autonomous Case Scheduled or None Selected");
		}

		System.out.println("[INFO]: Autonomous Delay (" + autoDelay + ")");

		// Resets and starts timer
		time.reset();
		time.start();

		// Resets all drive encoders
		//drivetrain.resetDriveEncoders();

	}

	@Override
	public void autonomousPeriodic() {

	}	

	@Override
	public void teleopInit() {

		if (auto != null) {
			auto.cancel();
		}

		time.stop();

		// MAKE SURE TO COMMENT THIS OUT FOR COMPETITION. 
		// we don't want the robot to die because the offsets were reset
		//drivetrain.newOffsets();

		
	}

	@Override
	public void teleopPeriodic() {
		CommandScheduler.getInstance().run();
/*
		if (driver.getRawButton(1)) {
			drivetrain.resetDriveEncoders();
		}

		if (driver.getRawButton(2)) {
			ahrs.reset();
		}

		if (driver.getRawButton(3)) {
			drivetrain.newOffsets();
		}
*/
		/*
		*	OPERATOR CODE
		*/

		/*
		// Box Code
		if(operator.getPOV() == 0) {
			box.tiltUp();
		}
		else if(operator.getPOV() == 180) {
			box.tiltDown();
		}
		
		if(operator.getPOV() == 270) {
			box.dumpOpen();
		}
		else if(operator.getPOV() == 90) {
			box.dumpClose();
		}

		if(operator.getRawButton(5)) {
			box.resetAllServos();
		}
		
		if(operator.getRawButton(6)) {
			box.tiltCenter();
		}
		*/
	}

}
