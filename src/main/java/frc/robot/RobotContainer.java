package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TestAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  //Criação dos controles
  XboxController pilot   = new XboxController(DriverConstants.id_Pilot);
  XboxController copilot = new XboxController(DriverConstants.id_CoPilot);

  //Criação dos subsistemas
  Swerve    sb_swerve   = new Swerve();
  Limelight sb_ll       = new Limelight();
  Shooter   sb_shooter  = new Shooter(sb_ll);
  Intake    sb_intake   = new Intake();
  Conveyor  sb_conveyor = new Conveyor();
  Vision    sb_vision   = new Vision();
  Climber   sb_climber  = new Climber();

  //Variáveis intermediarias de trackeamento
  double trackCorrectionX, trackCorrectionY;

  public RobotContainer() {}

  public void configureBindings() {
 
    //Shooter
    sb_shooter.setDefaultCommand(new RunCommand(() -> {
      
      trackCorrectionX  = 0;
      trackCorrectionY  = 0;

      if (copilot.getLeftTriggerAxis() != 0){
        trackCorrectionX = sb_ll.getTrackX();
        trackCorrectionY = sb_ll.getTrackY();
        if(sb_ll.noTag()) {
          trackCorrectionX  = 0;
          trackCorrectionY  = 0;
        }
        sb_shooter.setShooter(copilot.getLeftTriggerAxis());
      } else if (pilot.getRightBumper()) {
        sb_shooter.setShooter(-0.6);
      } else {
        sb_shooter.setShooter(0);
      }

    }, sb_shooter));
 
    //Climber
    sb_climber.setDefaultCommand(new RunCommand(() -> {
      
      if (copilot.getYButton()) {
        if(copilot.getBButton()){
        sb_climber.setClimber(.7);
        }
      } else {
        sb_climber.setClimber(0);
      }
      
    }, sb_climber));
    
    //Intake
    sb_intake.setDefaultCommand(new RunCommand(() -> {
      
      sb_intake.setIntake(pilot.getRightTriggerAxis() - pilot.getLeftTriggerAxis() * 0.8);

    }, sb_intake));

    //Conveyor
    sb_conveyor.setDefaultCommand(new RunCommand(() -> {

      if (copilot.getRightTriggerAxis() != 0) {
        sb_conveyor.setConveyor(-0.7);
      } else if (copilot.getLeftTriggerAxis() != 0) {
        sb_conveyor.setConveyor(0.4);
        if (copilot.getRightTriggerAxis() != 0) {
        sb_conveyor.setConveyor(-0.7);
        }
      }   else {
        sb_conveyor.setConveyor(pilot.getLeftTriggerAxis() - pilot.getRightTriggerAxis() * 0.2);
      }

    }, sb_conveyor));

    //Swerve
    sb_swerve.setDefaultCommand(new SwerveJoystickCmd(
      sb_swerve,
      () -> (pilot.getLeftY()  - trackCorrectionY) * (pilot.getRightBumper() ? 0.5 : 1),
      () ->  pilot.getLeftX()                      * (pilot.getRightBumper() ? 0.5 : 1),
      () -> (pilot.getRightX() + trackCorrectionX) * (pilot.getRightBumper() ? 0.5 : 1),
      () ->  !pilot.getAButton(),
      () ->  pilot.getBButton()));

  }
  
  public Command getAutonomousCommand() {

    return null;
    //return new TestAuto(sb_swerve, sb_shooter, sb_conveyor, sb_intake);
    //return new AutoFirstCenter(sb_swerve, sb_shooter, sb_conveyor, sb_intake);
    //return new AutoSecondCenter(sb_swerve, sb_shooter, sb_conveyor, sb_intake);
    //return new AutoThirdCenter(sb_swerve, sb_shooter, sb_conveyor, sb_intake);

    //OBS: Lados em relação a visão do piloto na quadra
    //return new AutoBlueRightAndRedLeft(sb_swerve, sb_shooter, sb_conveyor, sb_intake);
    //return new AutoBlueLeftAndRedRight(sb_swerve, sb_shooter, sb_conveyor, sb_intake);

  }
}
