package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {

  //Criação do controlador do Conveyor
  WPI_TalonSRX conveyor = new WPI_TalonSRX(ConveyorConstants.id_Conveyor);

  public Conveyor() {}

  //Função de setagem da velocidade do Climber
  public void setConveyor(double vel) { 
      conveyor.set(vel); 
  }

  @Override
  public void periodic() {
  }
}
