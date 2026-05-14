// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
    private SwerveSubsystem swerve;
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private XBoxContainer xbox = new XBoxContainer();
    private VisionSubsystem vision = new VisionSubsystem();
    private Command m_autonomousCommand;
    private int allianceOffset = 1;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private final StringSubscriber nameSub;
    private String autoString;

    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 255);

    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Meters.of(1 / 720.0);

    // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
    // of 1 meter per second.
    private final LEDPattern m_scrollingRainbow =
            m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.01), kLedSpacing);

    private final LEDPattern m_greenPattern = LEDPattern.solid(new Color(0, 255, 0));
    private final LEDPattern m_redPattern = LEDPattern.solid(new Color(255, 0, 0));

    private final RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        Logger.recordMetadata("RobotTeam", "8234"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else {
            // setUseTiming(false); // Run as fast as possible
            // String logPath =
            //        LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            // Logger.addDataReceiver(
            //        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }
        if (isSimulation()) {
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        swerve = new SwerveSubsystem();
        m_robotContainer = new RobotContainer(swerve, vision, xbox);
        CanandEventLoop.getInstance();

        m_led = new AddressableLED(0);

        // // Reuse buffer
        // // Default to a length of 60, start empty output
        // // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(32);
        m_led.setLength(m_ledBuffer.getLength());

        // // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard/Auto Chooser");
        nameSub = table.getStringTopic("active").subscribe("");

        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

        var alliance = DriverStation.getAlliance();
        boolean quickFixBool;
        if (alliance.isPresent()) {
            allianceOffset = (alliance.get() == DriverStation.Alliance.Red ? 1 : -1);
            quickFixBool = (alliance.get() == DriverStation.Alliance.Red);
        } else {
            allianceOffset = -1;
            quickFixBool = false;
        }
        swerve.quickFixAlliance(quickFixBool);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // NOTE: Red & green are swapped because the LEDs are weird.
        if (swerve.getAlign()) {
            if (swerve.getDistanceFromHub() >= 2.0) {
                m_redPattern.applyTo(m_ledBuffer);
            } else {
                m_greenPattern.applyTo(m_ledBuffer);
            }
        } else {
            // Update the buffer with the rainbow animation
            m_scrollingRainbow.applyTo(m_ledBuffer);
        }
        // // Set the LEDs
        m_led.setData(m_ledBuffer);
        // Runs athe Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Correct pose estimate with vision measurements
        vision.periodic(swerve);

        Logger.recordOutput("RobotPose", swerve.getPose());
        Logger.recordOutput("ShooterSpeeds", shooter.getSpeeds());
        Logger.recordOutput("DistanceFromHub", swerve.getDistanceFromHub());
        Logger.recordOutput("ShooterSetSpeed", shooter.getSetSpeed());
        Logger.recordOutput("allianceBoolean", swerve.getAllianceBoolean());
        Logger.recordOutput("EstimatedShooterSpeed", shooter.shootFunction(swerve.getDistanceFromHub()));

        autoString = nameSub.get();
        Logger.recordOutput("AutoNameFromTables", autoString); // DO NOT COMMENT OUT, NEEDED FOR AUTOS
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().schedule(shooter.stopCommand());
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(shooter.distShootCommand());
        m_autonomousCommand = m_robotContainer.getAutonomousCommand(autoString);
        Logger.recordOutput("AutoCommand", m_autonomousCommand == null);
        var alliance = DriverStation.getAlliance();
        boolean quickFixBool;
        if (alliance.isPresent()) {
            allianceOffset = (alliance.get() == DriverStation.Alliance.Red ? 1 : -1);
            quickFixBool = (alliance.get() == DriverStation.Alliance.Red);
        } else {
            allianceOffset = -1;
            quickFixBool = false;
        }
        swerve.quickFixAlliance(quickFixBool);

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }
    ;

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().schedule(shooter.manualShootCommand());
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        swerve.drive(allianceOffset * xbox.driveY(), allianceOffset * xbox.driveX(), -xbox.rotate(), true);
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
        swerve.resetOdometry(new Pose2d(new Translation2d(5, 5), new Rotation2d(0)));
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        Logger.recordOutput("RobotPose", swerve.getSimulationDriveTrainPose());
        vision.updateVisionSim(swerve.getPose());
        // Logger.recordOutput("VisionLeftPose", vision.getEstimatedLeftCamPose().toString());
        // Logger.recordOutput("VisionRightPose", vision.getEstimatedRightCamPose().toString());
    }
}
