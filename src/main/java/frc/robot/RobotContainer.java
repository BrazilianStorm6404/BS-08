package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.AutoBlueLeftAndRedRight;
import frc.robot.commands.AutoBlueRightAndRedLeft;
import frc.robot.commands.AutoFirstCenter;
import frc.robot.commands.AutoSecondCenter;
import frc.robot.commands.AutoSecondCenterRed;
import frc.robot.commands.AutoThirdCenter;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TestAuto;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Camera;
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
  XboxController copilot = new XboxController(2); 
  //XboxController copilot = new XboxController(2); 

  //Criação dos subsistemas
  Swerve    sb_swerve   = new Swerve();
  //Limelight sb_ll       = new Limelight();
  Shooter   sb_shooter  = new Shooter();
  Intake    sb_intake   = new Intake();
  Conveyor  sb_conveyor = new Conveyor();
  Vision    sb_vision   = new Vision();
  Climber   sb_climber  = new Climber();
  Camera    sb_camera   = new Camera();
  //Amp       sb_amp      = new Amp();

  //Variáveis intermediarias de trackeamento
  double trackCorrectionX, trackCorrectionY;

  public RobotContainer() {}

  public void configureBindings() {
    
    //Shooter
    sb_shooter.setDefaultCommand(new RunCommand(() -> {
      
      trackCorrectionX  = 0;
      trackCorrectionY  = 0;

      if (copilot.getAButton()) {
        sb_shooter.retrain(true);
      }else if (copilot.getLeftTriggerAxis() != 0){
        /*trackCorrectionX = sb_ll.getTrackTeleX();
        trackCorrectionY = sb_ll.getTrackY();
        if(sb_ll.noTag()) {
          trackCorrectionX  = 0;
          trackCorrectionY  = 0;
        }
        */
        sb_shooter.setShooter(copilot.getLeftTriggerAxis());
      } else if (pilot.getRightTriggerAxis() != 0){
        sb_shooter.setShooter(-0.6);
      } else if (copilot.getRightBumper()) {
        sb_shooter.setShooter(-0.6);
      } else {
        sb_shooter.setShooter(0);
        sb_shooter.retrain(false);
      }
      
    }, sb_shooter));

    //Amp
    /*sb_amp.setDefaultCommand(new RunCommand(() -> {

      if (copilot.getRightBumper()){
        sb_amp.setAmp(0.8);
      } else if (copilot.getLeftBumper()) {
        sb_amp.setAmp(-0.8);
      } else {
        sb_amp.setAmp(0);
      }

    }, sb_amp));
 */
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
      
      if (pilot.getRightTriggerAxis() != 0) {
        sb_intake.setIntake(pilot.getRightTriggerAxis());
      } else if (pilot.getLeftTriggerAxis() != 0) {
        sb_intake.setIntake(-pilot.getLeftTriggerAxis() * 0.7);
      } else {
        sb_intake.setIntake(0);
      }

    }, sb_intake));

    //Conveyor
    sb_conveyor.setDefaultCommand(new RunCommand(() -> {

      if (pilot.getRightTriggerAxis() != 0 && copilot.getLeftTriggerAxis() == 0) {
        sb_conveyor.setConveyor(0.8);
      } else if (copilot.getLeftTriggerAxis() != 0) {
        sb_conveyor.setConveyor(-0.6);
        if (copilot.getRightTriggerAxis() != 0) {
        sb_conveyor.setConveyor(0.5);//0.7
        }
        
      }else {
        sb_conveyor.setConveyor(0);
      }

    }, sb_conveyor));

    //Swerve
     
    sb_swerve.setDefaultCommand(new SwerveJoystickCmd(
      sb_swerve,
      () -> (pilot.getLeftY())  * (pilot.getRightBumper() ? 0.2 : 0.8),
      () ->  pilot.getLeftX()   * (pilot.getRightBumper() ? 0.2 : 0.8),
      () -> (pilot.getRightX())  * (pilot.getRightBumper() ? 0.2 : 0.8),
      () ->  true,
      () ->  pilot.getBButton()));
//*/

      //Shooter
   /*  sb_shooter.setDefaultCommand(new RunCommand(() -> {
      
      trackCorrectionX  = 0;
      trackCorrectionY  = 0;

      if (copilot.getSquareButton()) {
        sb_shooter.retrain(true);
      }else if (copilot.getL2Axis() != 0){
        trackCorrectionX = sb_ll.getTrackX();
        trackCorrectionY = sb_ll.getTrackY();
        if(sb_ll.noTag()) {
          trackCorrectionX  = 0;
          trackCorrectionY  = 0;
        }
        sb_shooter.setShooter(copilot.getL2Axis());
      } else if (pilot.getRightTriggerAxis() != 0){
        sb_shooter.setShooter(-1);
      } else if (copilot.getR1Button()) {
        sb_shooter.setShooter(-0.6);
      } else {
        sb_shooter.setShooter(0);
        sb_shooter.retrain(false);
      }

    }, sb_shooter));

    //Amp
    sb_amp.setDefaultCommand(new RunCommand(() -> {

      if (copilot.getR1Button()){
        sb_amp.setAmp(0.8);
      } else if (copilot.getL1Button()) {
        sb_amp.setAmp(-0.8);
      } else {
        sb_amp.setAmp(0);
      }

    }, sb_amp));
 
    //Climber
    sb_climber.setDefaultCommand(new RunCommand(() -> {
      
      if (copilot.getTriangleButton()) {
        if(copilot.getCircleButton()){
        sb_climber.setClimber(.7);
        }
      } else {
        sb_climber.setClimber(0);
      }
      
    }, sb_climber));
    
    //Intake
    sb_intake.setDefaultCommand(new RunCommand(() -> {
      
      if (pilot.getRightTriggerAxis() != 0){
        sb_intake.setIntake(1);
      } else if (pilot.getLeftTriggerAxis() != 0){
        sb_intake.setIntake(-0.7);
      } else {
        sb_intake.setIntake(0);
      }

    }, sb_intake));

    //Conveyor
    sb_conveyor.setDefaultCommand(new RunCommand(() -> {

      if (pilot.getRightTriggerAxis() != 0) {
        sb_conveyor.setConveyor(0.6);
      } else if (copilot.getL2Axis() != 0) {
        sb_conveyor.setConveyor(-0.5);
        if (copilot.getR2Axis() != 0) {
        sb_conveyor.setConveyor(0.8);//0.7
        }
      }else {
        sb_conveyor.setConveyor(0);
      }

    }, sb_conveyor));

    //Swerve
    sb_swerve.setDefaultCommand(new SwerveJoystickCmd(
      sb_swerve,
      () -> (pilot.getLeftY()) * (pilot.getRightBumper() ? 0.2 : 0.8),
      () ->  pilot.getLeftX()                      * (pilot.getRightBumper() ? 0.2 : 0.8),
      () -> (pilot.getRightX() + trackCorrectionX) * (pilot.getRightBumper() ? 0.2 : 0.8),
      () ->  !pilot.getAButton(),
      () ->  pilot.getBButton()));
*/
  }
  
  public Command getAutonomousCommand() {

    //return null;
    //return new TestAuto(sb_swerve, sb_shooter, sb_conveyor, sb_intake);
    //return new AutoFirstCenter(sb_swerve, sb_shooter, sb_conveyor, sb_intake);
    //return new AutoSecondCenter(sb_swerve, sb_shooter, sb_conveyor, sb_intake);
    //return new ShooterCmd(sb_shooter, sb_conveyor);
    //return new AutoSecondCenterRed(sb_swerve, sb_shooter, sb_conveyor, sb_intake);
    //return new AutoThirdCenterRed(sb_swerve, sb_shooter, sb_conveyor, sb_intake);

    //OBS: Lados em relação a visão do piloto na quadra
    //return new AutoBlueRightAndRedLeft(sb_swerve, sb_shooter, sb_conveyor, sb_intake);//
    return new AutoBlueLeftAndRedRight(sb_swerve, sb_shooter, sb_conveyor, sb_intake);

  }
}
