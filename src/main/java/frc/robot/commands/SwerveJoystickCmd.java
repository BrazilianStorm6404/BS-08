package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class SwerveJoystickCmd extends Command {

  //TODO Criação dos comandos do controle da swerve

  //Subsistema
  Swerve sb_swerve;
  //Eixos do controle
  Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  //Quadra
  Supplier<Boolean> fieldOrientedFunction, resetGyro;
  //Suaviza transições rápidas
  SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  Boolean resetBlock = false;

  public SwerveJoystickCmd(Swerve sb_swerve, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
  Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> resetGyro) {

    this.sb_swerve             = sb_swerve;
    this.xSpdFunction          = xSpdFunction;
    this.ySpdFunction          = ySpdFunction;
    this.turningSpdFunction    = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.resetGyro             = resetGyro;

    //Quantidade de mudanças que podem ocorrer por segundo
    this.xLimiter       = new SlewRateLimiter(3);
    this.yLimiter       = new SlewRateLimiter(3);
    this.turningLimiter = new SlewRateLimiter(3);

    addRequirements(sb_swerve);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    //Obtém valores analógicos
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();
    boolean resetG = resetGyro.get();

    if (resetG && !resetBlock) {
      sb_swerve.zeroHeading();
      sb_swerve.setGyroOffset(0);
    }
    resetBlock = resetG;

    //Impõe valor mínimo para a execução
    xSpeed       = Math.abs(xSpeed)       > 0.05 ? xSpeed : 0.0;
    ySpeed       = Math.abs(ySpeed)       > 0.05 ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;  

    //Torna a direção mais suave
    xSpeed       = xLimiter.calculate(xSpeed);
    ySpeed       = yLimiter.calculate(ySpeed);
    turningSpeed = turningLimiter.calculate(turningSpeed);//*/

    //Criação da velocidade do chassi desejada
    ChassisSpeeds chassisSpeeds;

    //Verifica se a direçaõ é em relação a quadra
    if (fieldOrientedFunction.get()) {
      //Relativo a quadra
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, sb_swerve.getRotation2d());
    } else {
      //Relativo ao robô
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    //Converte a velocidade do chassi ao estado individual dos módulos
    SwerveModuleState[] moduleStates = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);

    //Seta cada estado para suas rodas
    sb_swerve.setModuleStates(moduleStates);


  }

  @Override
  public void end(boolean interrupted) {
    //Para Módulos
    sb_swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
