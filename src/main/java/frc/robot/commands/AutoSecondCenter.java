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
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoSecondCenter extends SequentialCommandGroup {

    Shooter  sb_shooter;
    Conveyor sb_conveyor;
    Intake   sb_intake;

    public AutoSecondCenter (Swerve sb_swerve, Shooter shooter, Conveyor conveyor, Intake intake) {

        sb_shooter  = shooter;
        sb_conveyor = conveyor;
        sb_intake   = intake;

        // Configuração da trajetória com uma velocidade máxima de 3 unidades/s e uma aceleração máxima de 3 unidades/s^2
        TrajectoryConfig config = new TrajectoryConfig(2.5, 1)
                                      .setKinematics(SwerveConstants.kinematics);

        Trajectory trajectoryFwd1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
                List.of(new Translation2d(.215,0)),
            new Pose2d(0.43, 0, new Rotation2d(Math.toRadians(0))),
            config
        );

        Trajectory trajectoryBack1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.43, 0, new Rotation2d(Math.toRadians(0))),
                List.of(new Translation2d(.215,0)),
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
            config.setReversed(true)
        );

        Trajectory FirstTrajectory  = trajectoryBack1.concatenate(trajectoryFwd1);

        Trajectory trajectoryFwd2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
                List.of(new Translation2d(-.15,-.4)),
            new Pose2d(-.43, -.4, new Rotation2d(Math.toRadians(0))),
            config
        );

        Trajectory trajectoryBack2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-.43, -.4, new Rotation2d(Math.toRadians(0))),
                List.of(new Translation2d(-.15,-.15)),
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
            config
        );

        //Trajectory SecondTrajectory = trajectoryBack2.concatenate(trajectoryFwd2);

        // Configuração de um controlador PID (soma com o interno)
        var thetaController = new ProfiledPIDController(0.0,0,0,SwerveAutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PIDController xPID = new PIDController(0.0, 0.00001, 0.0);
        PIDController yPID = new PIDController(0.0, 0.01, 0.0);    

        // Criação de um comando de controle Swerve usando a trajetória gerada
        SwerveControllerCommand firstControllerCommand = new SwerveControllerCommand(
                FirstTrajectory,                        
                sb_swerve::getPose,                    
                SwerveConstants.kinematics,              
                xPID,                                  
                yPID,                                  
                thetaController,                       
                sb_swerve::setModuleStates,            
                sb_swerve                              
        );

        // Criação de um comando de controle Swerve usando a trajetória gerada
        SwerveControllerCommand secondControllerCommand = new SwerveControllerCommand(
                trajectoryFwd2,                        
                sb_swerve::getPose,                    
                SwerveConstants.kinematics,              
                xPID,                                  
                yPID,                                  
                thetaController,                       
                sb_swerve::setModuleStates,            
                sb_swerve                              
        );

        SwerveControllerCommand thirdControllerCommand = new SwerveControllerCommand(
                trajectoryBack2,                        
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
        //new ShooterCmd(sb_shooter, sb_conveyor),
        //new IntakeCmd(intake, conveyor, true),
        new InstantCommand(() -> sb_swerve.resetOdometry(FirstTrajectory.getInitialPose())),  
        firstControllerCommand,
        //new InstantCommand(() -> sb_swerve.stopModules()),
        //new IntakeCmd(intake, conveyor, false),
        //new ShooterCmd(sb_shooter, sb_conveyor),
        //new IntakeCmd(intake, conveyor, true),
        //new InstantCommand(() -> sb_swerve.resetOdometry(trajectoryFwd2.getInitialPose())),  
        secondControllerCommand,
        //new InstantCommand(() -> sb_swerve.stopModules()),
        //new InstantCommand(() -> sb_swerve.resetOdometry(trajectoryBack2.getInitialPose())),  
        thirdControllerCommand,
        new InstantCommand(() -> sb_swerve.stopModules())
        //new IntakeCmd(intake, conveyor, false),
        //new ShooterCmd(sb_shooter, sb_conveyor)
        );
    }
}
