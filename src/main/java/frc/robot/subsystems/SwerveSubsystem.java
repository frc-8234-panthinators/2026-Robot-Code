package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive swerveDrive;
    private boolean align = false;
    private boolean allianceBoolean = false; // TRUE = RED, FALSE = BLUE

    private final Translation2d redHub = new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.85));
    private final Translation2d blueHub = new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.85));

    public SwerveSubsystem() {
        double maximumSpeed = Units.feetToMeters(17.4);
        // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        RobotConfig config;
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
            config = RobotConfig.fromGUISettings();
            var alliance = DriverStation.getAlliance();
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRelative(
                            speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                    // optionally outputs individual module feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                            // for holonomic drive trains
                            new PIDConstants(2, 0.0, 0.01), // Translation PID constants
                            new PIDConstants(1, 0.0, 0.01) // Rotation PID constants
                            ),
                    config, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
                    );
            // swerveDrive.setMotorIdleMode(true);
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    /**
     * swerveDrive uses chassisSpeeds through driveFieldOriented to drive the robot.
     *
     * @param chassisSpeeds Targets speed of the chassis.
     */
    private void driveFieldOriented(ChassisSpeeds chassisSpeeds) {
        swerveDrive.driveFieldOriented(chassisSpeeds);
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier headingX,
            DoubleSupplier headingY) {
        return this.run(() -> {
            Translation2d scaledInputs = SwerveMath.scaleTranslation(
                    new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                    scaledInputs.getX(),
                    scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    public void drive(double translationX, double translationY, double rotation, boolean fieldRelative) {
        Logger.recordOutput("Align", this.align);
        if (!this.align) {
            swerveDrive.drive(
                    new Translation2d(
                            translationX * swerveDrive.getMaximumChassisVelocity(),
                            translationY * swerveDrive.getMaximumChassisVelocity()),
                    (rotation * Math.abs(rotation)) * swerveDrive.getMaximumChassisAngularVelocity(),
                    fieldRelative,
                    false);
        } else {
            Pose2d pose = getPose();
            double angle;
            double driveDirect;
            if (allianceBoolean) {
                driveDirect = -1;
                angle = Math.atan2(
                        (pose.getTranslation().getY() - redHub.getY()),
                        (redHub.getX() - pose.getTranslation().getX()));
            } else {
                driveDirect = 1;
                angle = Math.atan2(
                        (pose.getTranslation().getY() - blueHub.getY()),
                        (blueHub.getX() - pose.getTranslation().getX()));
            }
            swerveDrive.drive(
                    new Translation2d(
                            driveDirect
                                    * ((translationY * Math.sin(angle)) - (translationX * -Math.cos(angle)))
                                    * swerveDrive.getMaximumChassisVelocity(),
                            driveDirect
                                    * ((translationY * Math.cos(angle)) - (translationX * Math.sin(angle)))
                                    * swerveDrive.getMaximumChassisVelocity()),
                    (MathUtil.clamp(
                                    MathUtil.angleModulus(
                                            -pose.getRotation().getRadians() - angle
                                            // + Math.signum(translationY) * 0.2 / getDistanceFromHub()
                                            ),
                                    -0.6,
                                    0.6)
                            * swerveDrive.getMaximumChassisAngularVelocity()),
                    true,
                    false);
        }
    }

    // public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    //     swerveDrive.drive(
    //             translation,
    //             rotation,
    //             fieldRelative,
    //             false); // Open loop is disabled since it shouldn't be used most of the time.
    // }

    public Pose2d getSimulationDriveTrainPose() {
        return swerveDrive.getSimulationDriveTrainPose().get();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public boolean getAllianceBoolean() {
        return allianceBoolean;
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        swerveDrive.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    // This makes controls unusable / unpredictable
    public Command resetHeading() {
        return this.runOnce(() -> {
            swerveDrive.zeroGyro();
        });
    }

    public void quickFixAlliance(boolean alliance) {
        allianceBoolean = alliance;
    }

    public Command autoAlignCommand() {
        return this.runOnce(() -> {
                    align = true;
                })
                .andThen(this.run(() -> {
                            drive(0, 0, 0, true);
                        })
                        .withTimeout(1));
    }

    public Command unalignCommand() {
        return this.runOnce(() -> {
            align = false;
        });
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public void driveRelative(ChassisSpeeds chassis) {
        swerveDrive.drive(chassis);
    }

    public double getDistanceFromHub() {
        return getPose().getTranslation().getDistance((allianceBoolean == true) ? redHub : blueHub);
    }

    public Command alignCommand() {
        return runOnce(() -> {
            align = true;
        });
    }

    public Command stopAlignCommand() {
        return runOnce(() -> {
            align = false;
        });
    }

    public boolean getAlign() {
        return align;
    }

    public void lock() {
        swerveDrive.lockPose();
    }

    // public Command lockCommand() {
    //     return run(() -> {
    //         swerveDrive.lockPose();
    //     });
    // }

    // public Command distanceTestCommand(){
    //     return alignCommand()
    //             .andThen(Commands.waitSeconds(0.5))
    //             .andThen(run(() -> drive(-0.03, 0, 0, true)).withTimeout(0.5))
    //             .andThen(stopAlignCommand());
    // }

}
