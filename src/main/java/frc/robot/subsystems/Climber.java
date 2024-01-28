// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  WPI_VictorSPX climber  = new WPI_VictorSPX(0);
  Encoder       coder    = new Encoder(0, 1);
  WPI_VictorSPX solenoid = new WPI_VictorSPX(0);

  boolean lastIsUp = false, initTime = false;
  Timer time = new Timer();

  public Climber() {
    climber.setInverted(true);
  }

  public void setClimber(double vel) {

    if(coder.get() <= 0 && vel < 0)         vel = 0;
    else if (coder.get() > 1000 && vel > 0) vel = 0;

    if (vel > 0) solenoid.set(1);
    else         solenoid.set(0);
    
    if (!lastIsUp && vel > 0) {
      time.reset();
      time.start();
    }

    if (time.get() >= 0.5) initTime = true;

    if (time.get() < 0.5 && initTime) climber.set(-0.5);
    else                              climber.set(vel);

    lastIsUp = vel > 0;

  }

  @Override
  public void periodic() {
  }
}
