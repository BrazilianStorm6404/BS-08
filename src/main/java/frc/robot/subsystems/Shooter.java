// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  //WPI_TalonSRX frontShoot = new WPI_TalonSRX(0);
  //WPI_TalonSRX backShoot  = new WPI_TalonSRX(0);

  Limelight limelight;

  public Shooter(Limelight limelight) {
    this.limelight = limelight;
  }

  public void setShooter(double vel) {
    //frontShoot.set(vel);
    //backShoot.follow(frontShoot);
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
