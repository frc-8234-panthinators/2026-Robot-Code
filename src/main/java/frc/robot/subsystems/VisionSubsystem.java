package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera mainCam;
    PhotonPoseEstimator mainCamEstimator;

    PhotonCameraSim mainCamSim;
    VisionSystemSim visionSim;
    
    AprilTagFieldLayout tags;

    public VisionSubsystem() {
        mainCam = new PhotonCamera("mainCam");
        tags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        mainCamEstimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                Units.inchesToMeters(0)
            ),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(0),
                Units.degreesToRadians(0)
            )
        ));

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(tags);
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(1280, 960, Rotation2d.fromDegrees(100));
            cameraProp.setCalibError(0.7, 0.2);
            cameraProp.setFPS(50);
            cameraProp.setAvgLatencyMs(33);
            cameraProp.setLatencyStdDevMs(15);

            mainCamSim = new PhotonCameraSim(mainCam, cameraProp);
            visionSim.addCamera(mainCamSim,  new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(0)
                ),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(0)
                )
            ));
            mainCamSim.enableRawStream(true);
            mainCamSim.enableProcessedStream(true);
            mainCamSim.enableDrawWireframe(true);
        }
    }

    public void updateVisionSim(Pose2d pose) {
        visionSim.update(pose);
    }

    public Optional<EstimatedRobotPose> getEstimatedMainCamPose() {
        var result = mainCam.getLatestResult();
        if (!result.hasTargets()) {
            return Optional.empty();
        }

        if (Robot.isSimulation()) {
            mainCamEstimator.update(result).ifPresentOrElse(
                est ->
                    getSimDebugField()
                        .getObject("VisionEstimation")
                        .setPose(est.estimatedPose.toPose2d()),
                () -> {
                    getSimDebugField().getObject("VisionEstimation").setPoses();
                });
        }

        return mainCamEstimator.update(result);
    }

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}
