package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {

    CANSparkMax driveMotor;
    CANSparkMax turningMotor;
    RelativeEncoder driveCoder;

    CANcoder coder;
    CANcoderConfiguration coderConfig;
    PIDController turningPID;

    //TODO Criação do módulo swerve:
    public SwerveModule (int driveID, int turningID, boolean driveInverted, boolean turningInverted, int coderID, double coderOffset) {

        //motores
        driveMotor   = new CANSparkMax(coderID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(coderID, MotorType.kBrushless);

        //inverção de motores
        driveMotor.setInverted(driveInverted);
        turningMotor.setInverted(turningInverted);

        //Modo dos motores
        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);

        //drive encoders e converção de fatores para reais
        driveCoder = driveMotor.getEncoder();
        driveCoder.setPositionConversionFactor((1/6.12) * Math.PI * Units.inchesToMeters(4));//
        driveCoder.setVelocityConversionFactor((1/6.12) * Math.PI * Units.inchesToMeters(4)/60);//

        //turning encoders
        coder       = new CANcoder(coderID);
        coderConfig = new CANcoderConfiguration();
        coderConfig.MagnetSensor.MagnetOffset = coderOffset;
        coder.getConfigurator().apply(coderConfig);

        //turning PID 
        turningPID = new PIDController(0.006, 0, 0);
        turningPID.enableContinuousInput(0, 360);

        resetCoder();
    }

    //Obtém posição do encoder de direção
    public double getDrivePosition() {
        return driveCoder.getPosition();
    }

    //Obtém posição absoluta do encoder de giro
    public double getTurningPosition() {
        return coder.getAbsolutePosition().getValue() * 360;
    }

    //Obtém velocidade de direção
    public double getDriveVelocity() {
        return driveCoder.getVelocity();
    }

    //Obtém velocidade de giro
    public double getTurningVelocity() {
        return coder.getVelocity().getValue();
    }

    //Reseta encoders
    private void resetCoder() {
        coder.setPosition(0);
        driveCoder.setPosition(0);
      }

    //Obtém estado do módulo
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    //Obtém a posição do módulo
    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    //Define o estado desejado do módulo
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond);
        turningMotor.set(turningPID.calculate(getTurningPosition(), state.angle.getDegrees() + 180));
    }

    //Para módulo
    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }
    
}
