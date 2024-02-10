package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  CANSparkMax  frontIntake = new CANSparkMax(IntakeConstants.id_FrontWheel, MotorType.kBrushless);
  WPI_TalonSRX backIntake = new WPI_TalonSRX(IntakeConstants.id_BackWheel);

  public Intake() {}

  public void setIntake (double vel) {
    frontIntake.set(-vel);
    backIntake.set(vel);
  }

  public double getIntake(){
    return frontIntake.get();
  }

  @Override
  public void periodic() {
  }
}
