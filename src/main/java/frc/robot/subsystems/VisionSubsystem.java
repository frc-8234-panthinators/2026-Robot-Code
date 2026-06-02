package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera rightCam;
    private double yaw = 0.0;
    /**
     * Initialize photon vision wiht a single camera and this year's april tags
     */
    public VisionSubsystem() {
        // leftCam = new PhotonCamera("leftCam");
        rightCam = new PhotonCamera("rightCam");
    }

    public void updateYaw() {
        var results = rightCam.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                double best = 0;
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.objDetectId == 0 && target.getArea() / (1 + Math.abs(target.getYaw())) > best) {
                        best = target.getArea() / (1 + Math.abs(target.getYaw()));
                        // Found Tag 7, record its information
                        yaw = target.getYaw();
                    }
                }
            }
        }
    }

    public double getYaw(){
        return yaw;
    }

}
