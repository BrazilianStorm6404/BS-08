package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TestAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  XboxController pilot = new XboxController(0);
  XboxController copilot = new XboxController(1);

  Swerve sb_swerve     = new Swerve();
  Limelight sb_ll      = new Limelight();
  Shooter sb_shooter   = new Shooter(sb_ll);
  //Intake sb_intake     = new Intake();
  //Conveyor sb_conveyor = new Conveyor(sb_intake, sb_ll);
  //Climber sb_climber   = new Climber();

  double trackCorrection = 0;

  public RobotContainer() {
    
    configureBindings();
  }

  private void configureBindings() {
 
    sb_shooter.setDefaultCommand(new RunCommand(() -> {
      trackCorrection = 0;
      if (copilot.getLeftTriggerAxis() != 0){
        trackCorrection = sb_ll.getTrack();
        sb_shooter.setShooter(1);
      } else if (pilot.getLeftBumper()) {
        sb_shooter.setShooter(-0.5);
      } else {
        sb_shooter.setShooter(0);
      }

    }, sb_shooter));
/*
    sb_climber.setDefaultCommand(new RunCommand(() -> {
      
      if (copilot.getYButton()) {
        sb_climber.setClimber(1);
      } else if (copilot.getAButton()) {
        sb_climber.setClimber(-1);
      } else {
        sb_climber.setClimber(0);
      }
      
    }, sb_climber));

    sb_intake.setDefaultCommand(new RunCommand(() -> {
      
      sb_intake.setIntake(pilot.getLeftTriggerAxis() - pilot.getRightTriggerAxis());

    }, sb_intake));

    sb_conveyor.setDefaultCommand(new RunCommand(() -> {

      if (copilot.getRightTriggerAxis() != 0) {
        sb_conveyor.setConveyor(0.5);
      } else {
        sb_conveyor.setConveyor(pilot.getLeftTriggerAxis() - pilot.getRightTriggerAxis());
      }

    }, sb_conveyor));
*/
    sb_swerve.setDefaultCommand(new RunCommand(() -> {
      if(pilot.getBButton()) sb_swerve.zeroHeading();
    }, sb_swerve));

    sb_swerve.setDefaultCommand(new SwerveJoystickCmd(
                sb_swerve,
                () -> pilot.getLeftY()                       * (pilot.getRightBumper() ? 0.5 : 1),
                () -> pilot.getLeftX()                       * (pilot.getRightBumper() ? 0.5 : 1),
                () -> (-pilot.getRightX() + trackCorrection) * (pilot.getRightBumper() ? 0.5 : 1),
                () -> !pilot.getAButton()));
  }

  public Command getAutonomousCommand() {
    return new TestAuto(sb_swerve);
  }
}
