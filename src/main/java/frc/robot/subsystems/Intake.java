package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  CANSparkMax  frontIntake = new CANSparkMax(0, MotorType.kBrushless);
  WPI_TalonSRX backIntake = new WPI_TalonSRX(0);

  public Intake() {}

  public void setIntake (double vel) {
    frontIntake.set(vel);
    backIntake.set(-vel);
  }

  public double getIntake(){
    return frontIntake.get();
  }

  @Override
  public void periodic() {
  }
}
