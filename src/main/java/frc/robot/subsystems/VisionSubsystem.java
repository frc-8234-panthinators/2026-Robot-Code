package frc.robot.subsystems;

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
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSubsystem extends SubsystemBase {
    // PhotonCamera leftCam;
    PhotonCamera rightCam;
    // PhotonPoseEstimator leftCamEstimator;
    PhotonPoseEstimator rightCamEstimator;

    // PhotonCameraSim leftCamSim;
    PhotonCameraSim rightCamSim;
    VisionSystemSim visionSim;

    AprilTagFieldLayout tags;

    public VisionSubsystem() {
        // leftCam = new PhotonCamera("leftCam");
        rightCam = new PhotonCamera("rightCam");
        tags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        // leftCamEstimator = new PhotonPoseEstimator(
        //         tags,
        //         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        //         new Transform3d(
        //                 new Translation3d(Units.inchesToMeters(-5), Units.inchesToMeters(9),
        // Units.inchesToMeters(24)),
        //                 new Rotation3d(
        //                         Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(0))));
        rightCamEstimator = new PhotonPoseEstimator(
                tags,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(-5), Units.inchesToMeters(-9), Units.inchesToMeters(24)),
                        new Rotation3d(
                                Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(0))));

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(tags);
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(1280, 960, Rotation2d.fromDegrees(70));
            cameraProp.setCalibError(0.7, 0.2);
            cameraProp.setFPS(50);
            cameraProp.setAvgLatencyMs(33);
            cameraProp.setLatencyStdDevMs(15);

            // leftCamSim = new PhotonCameraSim(leftCam, cameraProp);
            // visionSim.addCamera(
            //         leftCamSim,
            //         new Transform3d(
            //                 new Translation3d(
            //                         Units.inchesToMeters(-5), Units.inchesToMeters(9), Units.inchesToMeters(24)),
            //                 new Rotation3d(
            //                         Units.degreesToRadians(0),
            //                         Units.degreesToRadians(-20),
            //                         Units.degreesToRadians(0))));
            // leftCamSim.enableRawStream(true);
            // leftCamSim.enableProcessedStream(true);
            // leftCamSim.enableDrawWireframe(true);
            rightCamSim = new PhotonCameraSim(rightCam, cameraProp);
            visionSim.addCamera(
                    rightCamSim,
                    new Transform3d(
                            new Translation3d(
                                    Units.inchesToMeters(-5), Units.inchesToMeters(9), Units.inchesToMeters(24)),
                            new Rotation3d(
                                    Units.degreesToRadians(0),
                                    Units.degreesToRadians(-20),
                                    Units.degreesToRadians(0))));
            rightCamSim.enableRawStream(true);
            rightCamSim.enableProcessedStream(true);
            rightCamSim.enableDrawWireframe(true);
        }
    }

    public void updateVisionSim(Pose2d pose) {
        visionSim.update(pose);
    }

    // public Optional<EstimatedRobotPose> getEstimatedLeftCamPose() {
    //     var result = leftCam.getLatestResult();
    //     if (!result.hasTargets()) {
    //         return Optional.empty();
    //     }

    //     if (Robot.isSimulation()) {
    //         leftCamEstimator
    //                 .update(result)
    //                 .ifPresentOrElse(
    //                         est -> getSimDebugField()
    //                                 .getObject("VisionEstimation")
    //                                 .setPose(est.estimatedPose.toPose2d()),
    //                         () -> {
    //                             getSimDebugField().getObject("VisionEstimation").setPoses();
    //                         });
    //     }

    //     return leftCamEstimator.update(result);
    // }

    public Optional<EstimatedRobotPose> getEstimatedRightCamPose() {
        var result = rightCam.getLatestResult();
        if (!result.hasTargets()) {
            return Optional.empty();
        }

        if (Robot.isSimulation()) {
            rightCamEstimator
                    .update(result)
                    .ifPresentOrElse(
                            est -> getSimDebugField()
                                    .getObject("VisionEstimation")
                                    .setPose(est.estimatedPose.toPose2d()),
                            () -> {
                                getSimDebugField().getObject("VisionEstimation").setPoses();
                            });
        }

        return rightCamEstimator.update(result);
    }

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}
