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
import frc.robot.subsystems.Swerve;

public class TestAuto extends SequentialCommandGroup {

    public TestAuto(Swerve sb_swerve) {

        // Configuração da trajetória com uma velocidade máxima de 3 unidades/s e uma aceleração máxima de 3 unidades/s^2
        TrajectoryConfig config = new TrajectoryConfig(3, 3)
                                      .setKinematics(SwerveConstants.kinematics);

        // Geração de uma trajetória de teste
        Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(Math.toRadians(0))),
            config
        );
        
        // Configuração de um controlador PID para o ângulo theta
        var thetaController = new ProfiledPIDController(
                1,
                0,
                0,
                SwerveAutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Criação de um comando de controle Swerve usando a trajetória gerada
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                testTrajectory,                        // Trajetória do robô
                sb_swerve::getPose,                    // Posição do robô em metros
                SwerveConstants.kinematics,            // Localização de cada módulo em relação ao centro do robô
                new PIDController(1, 0, 0),   // PID controller para a componente X
                new PIDController(1, 0, 0),   // PID controller para a componente Y
                thetaController,                       // Controlador PID para o ângulo theta
                sb_swerve::setModuleStates,            // Método para definir os estados dos módulos do Swerve
                sb_swerve                              // Instância do sistema Swerve
        );

        // Adição de comandos à sequência
        addCommands(
            new InstantCommand(() -> sb_swerve.resetOdometry(testTrajectory.getInitialPose())), // Comando instantâneo para resetar a odometria
            swerveControllerCommand,                                                       // Comando de controle Swerve usando a trajetória
            new InstantCommand(()-> sb_swerve.stopModules())
            );
    }
}
