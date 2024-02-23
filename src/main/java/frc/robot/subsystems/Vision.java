
package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  public Vision() {
    
    //Codigo da camera
    new Thread(() -> {

        // Criacao da camera
        UsbCamera vs_camera = CameraServer.startAutomaticCapture();
        //resolucao da camera
        vs_camera.setResolution(640, 480); 

    }).start();
  }

  @Override
  public void periodic() {}
}