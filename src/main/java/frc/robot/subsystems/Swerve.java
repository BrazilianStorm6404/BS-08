package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {

  //TODO Definição dos módulos swerve e criação da odometria:

  //Define os módulos
  SwerveModule frontLeft  = new SwerveModule(1, 2, false, true, 9, 0);
  SwerveModule frontRight = new SwerveModule(3, 4, false, true, 10, 0);
  SwerveModule backLeft   = new SwerveModule(5, 6, false, true, 11, 0);
  SwerveModule backRight  = new SwerveModule(7, 8, false, true, 12, 0);

  //Obtém posições dos módulos
  SwerveModulePosition[] modulePositions = {frontLeft.getSwerveModulePosition(), frontRight.getSwerveModulePosition(), 
                                            backLeft.getSwerveModulePosition(), backRight.getSwerveModulePosition()};

  //Giroscópio                                          
  AHRS navx = new AHRS(SPI.Port.kMXP);
  //Odometria
  SwerveDriveOdometry odometer = new SwerveDriveOdometry(SwerveConstants.kinematics, new Rotation2d(0), modulePositions);
  
  //Pausa para estabilização do sistema e resetar o giroscópio adequadamente
  public Swerve() {
    new Thread(() -> {
      try {
          Thread.sleep(1000);
          zeroHeading();
      } catch (Exception e) {
      }
  }).start();
  }

  //Reseta Giroscópio
  public void zeroHeading(){
    navx.reset();
  }

  //Obtém a posição do giroscópio em relação a quadra
  public double getHeading(){
    return Math.IEEEremainder(navx.getAngle(), 360.0);
  }

  //Obtém a angulação da quadra 
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-getHeading());
  }

  //Obtém a posição do robô em um plano 2D 
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  //Reseta odometria
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), modulePositions, pose);
  }

  //Para módulos
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  //Define cada estado de módulo
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 5);
    frontRight.setDesiredState(desiredStates[0]);
    frontLeft.setDesiredState(desiredStates[1]);
    backRight.setDesiredState(desiredStates[2]);
    backLeft.setDesiredState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    //Atualiza odometria
    odometer.update(getRotation2d(), modulePositions);

    SmartDashboard.putNumber("frontLeft", frontLeft.getTurningPosition()/360.);
    SmartDashboard.putNumber("frontRight", frontRight.getTurningPosition()/360.);
    SmartDashboard.putNumber("backLeft", backLeft.getTurningPosition()/360.);
    SmartDashboard.putNumber("backRight", backRight.getTurningPosition()/360.);

    SmartDashboard.putNumber("Heading", -getHeading());
    SmartDashboard.putNumber("Gyro", -navx.getAngle());

  }
}
