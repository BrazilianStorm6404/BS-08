package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//TODO Criação do módulo swerve:

public class SwerveModule {

    //Criação dos controladores da tração
    CANSparkMax driveMotor;
    CANSparkMax turningMotor;

    //Criação dos encoders dos Neos
    RelativeEncoder driveCoder;
    RelativeEncoder turningCoder;

    //Criação do estado do módulo
    SwerveModuleState currentState;

    //Criaçaõ do CANcoder
    CANcoder coder;
    PIDController turningPID;

    public SwerveModule (int driveID, int turningID, boolean driveInverted, boolean turningInverted, int coderID, double coderOffset) {

        //motores
        driveMotor   = new CANSparkMax(driveID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningID, MotorType.kBrushless);

        //inverção de motores
        driveMotor.setInverted(driveInverted);
        turningMotor.setInverted(turningInverted);

        //Modo dos motores
        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kCoast);

        //Neos encoders
        driveCoder = driveMotor.getEncoder();
        turningCoder = turningMotor.getEncoder();

        //turning encoders
        coder = new CANcoder(coderID);

        //turning PID 
        turningPID = new PIDController(0.006, 0, 0);
        turningPID.enableContinuousInput(0, 360);

        //Estado do módulo
        currentState = new SwerveModuleState(5, new Rotation2d(Units.degreesToRadians(coder.getAbsolutePosition().getValue())));

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
        driveCoder.setPosition(0);
    }

    //Obtém estado do módulo
    public SwerveModuleState getState() {
        return currentState;
    }

    //Obtém a posição do módulo
    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(Units.degreesToRadians(getTurningPosition())));
    }

    //Funçaõ de definção do estado desejado do módulo
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }
        currentState = SwerveModuleState.optimize(state, currentState.angle);
        driveMotor.set(currentState.speedMetersPerSecond);
        turningMotor.set(turningPID.calculate(getTurningPosition(), currentState.angle.getDegrees() + 180));

        //SmartDashboard.putNumber("turningPID", turningPID.calculate(getTurningPosition(), currentState.angle.getDegrees() + 180));
        //SmartDashboard.putNumber("state.speedMetersPerSecond", currentState.speedMetersPerSecond);
        //SmartDashboard.putNumber("state.angle.", currentState.angle.getDegrees() + 180);
    }

    //Função de parar módulo
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
    
}
