package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    // PhotonCamera leftCam;
    private final PhotonCamera rightCam;
    // PhotonPoseEstimator leftCamEstimator;
    private final PhotonPoseEstimator rightCamEstimator;

    // PhotonCameraSim leftCamSim;
    private PhotonCameraSim rightCamSim;
    private VisionSystemSim visionSim;

    private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    private Matrix<N3, N1> curStdDevs;

    private AprilTagFieldLayout tags;

    public VisionSubsystem() {
        // leftCam = new PhotonCamera("leftCam");
        rightCam = new PhotonCamera("rightCam");
        tags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        // leftCamEstimator = new PhotonPoseEstimator(
        //         tags,
        //         new Transform3d(
        //                 new Translation3d(Units.inchesToMeters(-5), Units.inchesToMeters(9),
        // Units.inchesToMeters(24)),
        //                 new Rotation3d(
        //                         Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(0))));
        rightCamEstimator = new PhotonPoseEstimator(
                tags,
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-3.952), Units.inchesToMeters(-8.4), Units.inchesToMeters(24.75)),
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
            //                         Units.inchesToMeters(-3.952), Units.inchesToMeters(8.4),
            // Units.inchesToMeters(24.75)),
            //                 new Rotation3d(
            //                         Units.degreesToRadians(0),
            //                         Units.degreesToRadians(20),
            //                         Units.degreesToRadians(0))));
            // leftCamSim.enableRawStream(true);
            // leftCamSim.enableProcessedStream(true);
            // leftCamSim.enableDrawWireframe(true);
            rightCamSim = new PhotonCameraSim(rightCam, cameraProp);
            visionSim.addCamera(
                    rightCamSim,
                    new Transform3d(
                            new Translation3d(
                                    Units.inchesToMeters(-3.952),
                                    Units.inchesToMeters(-8.4),
                                    Units.inchesToMeters(24.75)),
                            new Rotation3d(
                                    Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(0))));
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
    //                 .estimateCoprocMultiTagPose(result)
    //                 .ifPresentOrElse(
    //                         est -> getSimDebugField()
    //                                 .getObject("VisionEstimation")
    //                                 .setPose(est.estimatedPose.toPose2d()),
    //                         () -> {
    //                             getSimDebugField().getObject("VisionEstimation").setPoses();
    //                         });
    //     }

    //     return leftCamEstimator.estimateCoprocMultiTagPose(result);
    // }

    public void periodic(SwerveSubsystem swerve) {
        var results = rightCam.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
            var visionRightEst = rightCamEstimator.estimateCoprocMultiTagPose(result);

            if (visionRightEst.isEmpty()) {
                visionRightEst = rightCamEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionRightEst, result.getTargets());

            if (Robot.isSimulation()) {
                visionRightEst.ifPresentOrElse(
                        est -> getSimDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }

            visionRightEst.ifPresent(est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = getEstimationStdDevs();

                swerve.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
        }
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = rightCamEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist += tagPose.get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(
                                estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}
