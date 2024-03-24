package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeCmd extends InstantCommand {

  Intake sb_intake;
  Conveyor sb_conveyor;
  Shooter sb_shooter;
  boolean isFinish = false, isActivated = false;

  /** Creates a new ShooterCmd. */
  public IntakeCmd(Intake intake, Conveyor conveyor, Shooter shooter, Boolean activated) {
    sb_intake = intake;
    sb_conveyor = conveyor;
    sb_shooter = shooter;
    isActivated = activated;
    isFinish = false;
  }

  @Override
  public void initialize() { 
  }

  @Override
  public void execute() {
    if(isActivated) {
      sb_intake.setIntake(1);
      sb_conveyor.setConveyor(1);
      sb_shooter.setShooter(-0.6);
    } else {
      sb_intake.setIntake(0);
      sb_conveyor.setConveyor(0);
      sb_shooter.setShooter(0);
    }
    
    isFinish = true;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinish;
  }
}
