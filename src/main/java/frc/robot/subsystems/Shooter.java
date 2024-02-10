// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  WPI_TalonSRX frontShoot = new WPI_TalonSRX(ShooterConstants.id_FrontWheel);
  WPI_TalonSRX backShoot  = new WPI_TalonSRX(ShooterConstants.id_BackWheel);

  Limelight limelight;
  Timer time;

  public Shooter(Limelight limelight) {
    this.limelight = limelight;
    time = new Timer();
  }

  public void setShooter(double vel) {
    
    frontShoot.set(vel /* 0.4*/);
    backShoot.set(vel *0.8);

    if(vel!=0 && (limelight.getX() < Math.abs(10)) && limelight.tagSpeaker()) {
      limelight.setMode(2);
    } else {
      limelight.setMode(1);
    }
  }

  @Override
  public void periodic() {
  }
}
