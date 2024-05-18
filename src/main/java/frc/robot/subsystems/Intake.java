package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  //Criação do controlador do Intake
  WPI_TalonSRX  backIntake  = new WPI_TalonSRX(IntakeConstants.id_BackWheel);
  WPI_VictorSPX frontIntake = new WPI_VictorSPX(IntakeConstants.id_FrontWheel);

  public Intake() {}

  //Função de setagem da velocidade do Intake
  public void setIntake(double vel) {
    frontIntake.set(-vel);
    backIntake.set(-vel);
  }

  @Override
  public void periodic() {
  }
}
