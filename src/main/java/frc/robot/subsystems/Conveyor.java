package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {

  WPI_TalonSRX conveyor = new WPI_TalonSRX(ConveyorConstants.id_Conveyor);
  DigitalInput  sensor  = new DigitalInput(ConveyorConstants.id_Sensor);
  Intake        intake;
  Limelight     limelight;
  Timer time;

  public Conveyor(Intake intake, Limelight limelight) {
    this.intake    = intake;
    this.limelight = limelight;
    time = new Timer();
  }

  public Conveyor() {
      time = new Timer();
  }

  public void setConveyor(double vel) { 
      /*if(sensor.get() && (intake.getIntake() != 0)){
        conveyor.set(0);
        limelight.setMode(2);
      } else {
        conveyor.set(-vel);
        limelight.setMode(1);
      }
      */
      conveyor.set(-vel);
    
  }

  @Override
  public void periodic() {
  }
}
