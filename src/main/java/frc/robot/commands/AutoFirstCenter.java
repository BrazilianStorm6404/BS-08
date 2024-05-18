package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoFirstCenter extends SequentialCommandGroup {

    Shooter  sb_shooter;
    Conveyor sb_conveyor;
    Intake   sb_intake;
    Limelight sb_limelight;

    public AutoFirstCenter(Swerve sb_swerve, Shooter shooter, Conveyor conveyor, Intake intake) {

        sb_shooter   = shooter;
        sb_conveyor  = conveyor;
        sb_intake    = intake;
        //sb_limelight = limelight;

        sb_swerve.zeroHeading();
        sb_swerve.setGyroOffset(0);

        // Configuração da trajetória com uma velocidade máxima de 3 unidades/s e uma aceleração máxima de 3 unidades/s^2
        TrajectoryConfig config = new TrajectoryConfig(1, 0.8)
                                      .setKinematics(SwerveConstants.kinematics);

        // Geração de uma trajetória de teste
        Trajectory trajectoryBack = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
                List.of(new Translation2d(.15,-0.006)),
            new Pose2d(0.35, -0.006, new Rotation2d(Math.toRadians(0))),
            config
        );

        // Geração de uma trajetória de teste
        Trajectory trajectoryFwd = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.4, 0, new Rotation2d(Math.toRadians(0))),
                List.of(new Translation2d(.15,-0.02)),
            new Pose2d(0,-0.02, new Rotation2d(Math.toRadians(0))),
            config.setReversed(true)
        );

        //Trajectory FinalTrajectory = trajectoryBack.concatenate(trajectoryFwd);

        
        // Configuração de um controlador PID (pid soma com o interno)
        ProfiledPIDController thetaController = new ProfiledPIDController(0,0,0,SwerveAutoConstants.kThetaControllerConstraints);
        //thetaController.enableContinuousInput(0, 0);
        thetaController.disableContinuousInput();


        PIDController xPID = new PIDController(0, 0, 0);
        PIDController yPID = new PIDController(0, 0, 0);    

        // Criação de um comando de controle Swerve usando a trajetória gerada
        SwerveControllerCommand Step1 = new SwerveControllerCommand(
            trajectoryFwd,                        
            sb_swerve::getPose,                    
            SwerveConstants.kinematics,              
            xPID,                                  
            yPID,                                  
            thetaController,                       
            sb_swerve::setModuleStates,            
            sb_swerve                              
        );

        SwerveControllerCommand Step2 = new SwerveControllerCommand(
            trajectoryBack,
            sb_swerve::getPose,                    
            SwerveConstants.kinematics,              
            xPID,                                  
            yPID,                                  
            thetaController,                       
            sb_swerve::setModuleStates,            
            sb_swerve                              
        );
        
        // Sequência de comandos
        addCommands(
            new ShooterCmd(sb_shooter, sb_conveyor),
            new IntakeCmd(intake, conveyor, shooter, true),
            new InstantCommand(() -> sb_swerve.resetOdometry(trajectoryFwd.getInitialPose())),  
            Step1,
            new IntakeCmd(intake, conveyor, shooter, false),
            new InstantCommand(() -> sb_swerve.resetOdometry(trajectoryBack.getInitialPose())),
            Step2,
            new InstantCommand(() -> sb_swerve.stopModules()),
            new ShooterCmd(sb_shooter, sb_conveyor)

            );
    }
}
