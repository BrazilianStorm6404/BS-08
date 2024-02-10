
package frc.robot.subsystems;

// IMPORTS
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// CODE
public class Vision extends SubsystemBase {

  public Vision() {
    
    //Codigo da camera
    new Thread(() -> {

      // Criacao da camera

        UsbCamera vs_camera = CameraServer.startAutomaticCapture();
        vs_camera.setResolution(640, 480); //resolucao da camera

    }).start();
  }

  @Override
  public void periodic() {}
}