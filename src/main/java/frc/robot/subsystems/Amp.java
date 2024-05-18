// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Amp extends SubsystemBase {
  
  WPI_VictorSPX amp   = new WPI_VictorSPX(14);
  DigitalInput  limit = new DigitalInput(0);

  public Amp() {}

  public void setAmp(double vel) {

    amp.set(vel);

  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("limit", limit.get());
  }
}
