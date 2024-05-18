package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  //Criação dos controladores do shooter
  WPI_TalonSRX frontShoot = new WPI_TalonSRX(ShooterConstants.id_FrontWheel);
  WPI_TalonSRX backShoot  = new WPI_TalonSRX(ShooterConstants.id_BackWheel);

  //Criaçaõ da Limelight dentro do Shooter
  Limelight limelight;

  public Shooter() {
    //this.limelight = limelight;
  }

  //Função de setagem do Shooter
  public void setShooter(double vel) {

    frontShoot.set(vel);
    backShoot.set(vel * 0.9);

    /* //Verificação da condição de shoot
    if(vel!=0 && (Math.abs(limelight.getX()) < 5) && (Math.abs(limelight.getY()) < 2) && limelight.tagSpeaker()) {
      limelight.setMode(2);
    } else {
      limelight.setMode(1);
    }
    */
    
  }

  public void retrain(boolean retrain) {
    if (retrain) {
      frontShoot.set(-0.7);
      backShoot.set(-0.7);
    } else {
      frontShoot.set(0);
      backShoot.set(0);
    }
  }

  @Override
  public void periodic() {
  }
}
