package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends InstantCommand {

  Intake sb_intake;
  Conveyor sb_conveyor;
  boolean isFinish = false, isActivated = false;

  /** Creates a new ShooterCmd. */
  public IntakeCmd(Intake intake, Conveyor conveyor, Boolean activated) {
    sb_intake = intake;
    sb_conveyor = conveyor;
    isActivated = activated;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(isActivated) {
      sb_intake.setIntake(0.8);
     sb_conveyor.setConveyor(-0.2);
    } else {
      sb_intake.setIntake(0);
      sb_conveyor.setConveyor(0);
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
