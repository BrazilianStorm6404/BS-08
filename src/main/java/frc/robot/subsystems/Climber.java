package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  //Criação do controlador do Climber
  WPI_VictorSPX climber  = new WPI_VictorSPX(ClimberConstants.id_Climber);

  public Climber() {}
  
  //Função de setagem da velocidade do Climber
  public void setClimber(double vel) {
    climber.set(vel);
  }

  @Override
  public void periodic() {
  }
}
