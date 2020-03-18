package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {

	private String id;
    private NetworkTable kTable;
    //comands
	private NetworkTableEntry tv, tx, ty, ta, ts, tl, tshort, tlong, thor, tvert, getpipe; 

	public Limelight(String id) {
		this.id = id;
		kTable = NetworkTableInstance.getDefault().getTable("limelight-" + this.id);
		tv = kTable.getEntry("tv");
		tx = kTable.getEntry("tx");
		ty = kTable.getEntry("ty");
		ta = kTable.getEntry("ta");
		ts = kTable.getEntry("ts");
		tl = kTable.getEntry("tl");
		tshort = kTable.getEntry("tshort");
		tlong = kTable.getEntry("tlong");
		thor = kTable.getEntry("thor");
		tvert = kTable.getEntry("tvert");
		getpipe = kTable.getEntry("getpipe"); 
	}

	public Limelight() {
		this.id = "";
		kTable = NetworkTableInstance.getDefault().getTable("limelight");
		tv = kTable.getEntry("tv");
		tx = kTable.getEntry("tx");
		ty = kTable.getEntry("ty");
		ta = kTable.getEntry("ta");
		ts = kTable.getEntry("ts");
		tl = kTable.getEntry("tl");
		tshort = kTable.getEntry("tshort");
		tlong = kTable.getEntry("tlong");
		thor = kTable.getEntry("thor");
		tvert = kTable.getEntry("tvert");
		getpipe = kTable.getEntry("getpipe"); 
	}

	

	public double getValidTarget() {
		return tv.getDouble(0.0);
	}
	public double getX() {
		return tx.getDouble(0.0);
	}
	public double getY() {
		return ty.getDouble(0.0);
	}
	public double getArea() {
		return ta.getDouble(0.0);	
	}
	public double getSkew() {
		return ts.getDouble(0.0);
	}
	public double getLatency() {
		return tl.getDouble(0.0);
	}
	public double getShort() {
		return tshort.getDouble(0.0);
	}
	public double getLong() {
		return tlong.getDouble(0.0);
	}
	public double getHorizontal() {
		return thor.getDouble(0.0);
	}
	public double getVertical() {
		return tvert.getDouble(0.0);
	}
	public double getPipeline() {
		return getpipe.getDouble(0.0);
	}

	// New Methods

	// Toggles LED among the 4 possible modes
	public void toggleLED(String state) {
		switch(state) {
			case "pipeline":
				kTable.getEntry("ledMode").setNumber(0);
				break;
			case "off":
				kTable.getEntry("ledMode").setNumber(1);
				break;
			case "blink":
				kTable.getEntry("ledMode").setNumber(2);
				break;
			case "on":
				kTable.getEntry("ledMode").setNumber(3);
				break;
			default:
				break;
		}
	}

	public void LEDoff() {
		kTable.getEntry("ledMode").setNumber(1);
	}

	public void LEDon() {
		kTable.getEntry("ledMode").setNumber(0);
	}

	// Toggles between the 2 possible camera modes
	public void toggleCamMode(String state) {
		switch(state) {
			case "vision":
				kTable.getEntry("camMode").setNumber(0);
				break;
			case "driver":
				kTable.getEntry("camMode").setNumber(1);
				break;
			default:
				break;
		}
	}
	

	public void updateDashboard() {
		SmartDashboard.putNumber("(" + id + ")LimelightX", getX());
		SmartDashboard.putNumber("(" + id + ")LimelightY", getY());
		SmartDashboard.putNumber("(" + id + ")LimelightArea", getArea());
		SmartDashboard.putNumber("(" + id + ")LimelightSkew", getSkew());
		SmartDashboard.putNumber("(" + id + ")LimelightHasTar", getValidTarget());
	}
}
