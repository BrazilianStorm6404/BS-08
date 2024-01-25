package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TestAuto;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  XboxController pilot = new XboxController(0);
  Swerve sb_swerve     = new Swerve();

  public RobotContainer() {

    //Seta comandos da swerve
    /*sb_swerve.setDefaultCommand(new SwerveJoystickCmd(
                sb_swerve,
                () -> -pilot.getLeftX(),
                () -> pilot.getLeftY(),
                () -> pilot.getRightX(),
                () -> !pilot.getAButton()));*/
    
    configureBindings();
  }

  private void configureBindings() {
    sb_swerve.setDefaultCommand(new RunCommand(() -> {
      //Reseta Giroscópio
      if(pilot.getBButton()) sb_swerve.zeroHeading();
    }, sb_swerve));

    sb_swerve.setDefaultCommand(new SwerveJoystickCmd(
                sb_swerve,
                () -> pilot.getLeftY() * 0.5,
                () -> pilot.getLeftX() * 0.5,
                () -> -pilot.getRightX()* 0.5,
                () -> !pilot.getAButton()));
  }

  public Command getAutonomousCommand() {
    return new TestAuto(sb_swerve);
  }
}
