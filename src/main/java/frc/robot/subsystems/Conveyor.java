package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

  WPI_TalonSRX conveyor = new WPI_TalonSRX(0);
  DigitalInput  sensor   = new DigitalInput(0);
  Intake        intake;
  Limelight     limelight;

  public Conveyor(Intake intake, Limelight limelight) {
    this.intake    = intake;
    this.limelight = limelight;
  }

  public void setConveyor(double vel) {
    if(sensor.get() && (intake.getIntake() != 0)){
      setConveyor(0);
      limelight.setMode(2);
    } else {
      setConveyor(vel);
      limelight.setMode(1);
    }

    
  }

  @Override
  public void periodic() {
  }
}
