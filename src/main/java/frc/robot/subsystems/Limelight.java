package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  NetworkTable table   = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tid = table.getEntry("tid");

  double x,y,area,id;

  public Limelight() {}

  public void setMode (int mode) {
    table.getEntry("ledMode").setNumber(mode);
  }

  public double getX () {
    return x;
  }

  public double getTrack() {
    return x * 0.01;
  }

  public Boolean tagSpeaker () {
    return id == 4 || id == 7;
  }
 
  /* 
  public Boolean tagSource () {
    return id == 1 || id == 2 || id == 9 || id == 10;
  }*/

  @Override
  public void periodic() {
    x    = tx.getDouble(0.0) + 0.;
    y    = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    id   = tid.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("getTrack", getTrack());
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightID", id);

  }
}
