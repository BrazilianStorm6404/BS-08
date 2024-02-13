// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class ShooterCmd extends InstantCommand {

  Shooter sb_shooter;
  Conveyor sb_conveyor;
  boolean isFinish = false;
  Timer timer;

  /** Creates a new ShooterCmd. */
  public ShooterCmd(Shooter shooter, Conveyor conveyor) {
    timer = new Timer();
    sb_shooter = shooter;
    sb_conveyor = conveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();
    if(timer.get() < 2){
      sb_shooter.setShooter(1);
    } else if (timer.get() < 4) {
      sb_conveyor.setConveyor(1);
    } else if (timer.get() >= 4){
      sb_conveyor.setConveyor(0);
      sb_shooter.setShooter(0);
      isFinish = true;
    }
    System.out.println("aaaaaaaaaaaaaaaaaaaaaaa");
    SmartDashboard.putBoolean("getName()", isFinish);
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
