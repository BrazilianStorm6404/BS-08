// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

  // Drivers IDs
  public static class DriverConstants {
    // Pilots IDs
    public static final int id_Pilot   = 0;
    public static final int id_CoPilot = 1;
  }

  // Swerve Drive constants
  public static class SwerveConstants {
    // Comprimento do robô
    public static final double length = 0.75;
    // Largura do robô
    public static final double width  = 0.75;
    // Criação da posição dos módulos em relação ao centro do robô
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(width  / 2, -length / 2),
                new Translation2d(width  / 2, length  / 2),
                new Translation2d(-width / 2, -length / 2),
                new Translation2d(-width / 2, length  / 2));
  }

  // Swerve Autonônomo constants
  public static class SwerveAutoConstants {
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(Math.PI,Math.PI);
  }

  // Intake IDs
  public static class IntakeConstants {
    public static final int id_FrontWheel = 14;
    public static final int id_BackWheel  = 13;
  }

  // Conveyor IDs
    public static class ConveyorConstants {
    public static final int id_Conveyor = 17;
    public static final int id_Sensor   = 0;
  }
  
  // Shooter IDs
    public static class ShooterConstants {
    public static final int id_FrontWheel = 15;
    public static final int id_BackWheel  = 16;
  }

  // Climber IDs
    public static class ClimberConstants {
    public static final int id_Climber    = 18;
    public static final int id_Sol        = 19;
    public static final int id1_BoreCoder = 1;
    public static final int id2_BoreCoder = 2;
  }
}
