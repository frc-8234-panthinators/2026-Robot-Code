// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.text.ParseException;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final XBoxContainer xbox;
    private final SwerveSubsystem swerve;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final ClimberSubsystem climber;
    private final SendableChooser<Command> autoChooser;
    private boolean pathfindAtStart = true;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(SwerveSubsystem swerve, VisionSubsystem vision, XBoxContainer xbox) {
        this.xbox = xbox;
        shooter = new ShooterSubsystem();
        climber = new ClimberSubsystem();
        this.swerve = swerve;
        this.vision = vision;

        // NamedCommands.registerCommand("ResetHeading", swerve.resetHeading());
        NamedCommands.registerCommand("Intake", shooter.intakeCommand());
        NamedCommands.registerCommand("Shoot", shooter.shooterCommand(swerve.getDistanceFromHub()));
        NamedCommands.registerCommand("StopShooter", shooter.stopCommand());
        NamedCommands.registerCommand("Align", swerve.alignCommand());
        NamedCommands.registerCommand("Climb", climber.climbCommand());

        // Configure the trigger bindings
        configureBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");



        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putBoolean("PathfindAtStart", pathfindAtStart);
        SmartDashboard.putString("Auto Name", autoChooser.toString());
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        xbox.runIntake.onTrue(shooter.intakeCommand());
        xbox.runShooter.onTrue(shooter.shooterCommand(swerve.getDistanceFromHub()));
        xbox.distShoot.toggleOnTrue(
                shooter.switchShootType().andThen(shooter.shooterCommand(swerve.getDistanceFromHub())));
        xbox.distShoot.toggleOnFalse(shooter.switchShootType());
        xbox.stopShooter.onTrue(shooter.stopCommand());
        xbox.dpadLeft.onTrue(swerve.resetHeading());
        xbox.align.toggleOnTrue(swerve.alignCommand());
        xbox.align.toggleOnFalse(swerve.stopAlignCommand());
        xbox.dpadDown.onTrue(shooter.nudgeDownCommand());
        xbox.dpadUp.onTrue(shooter.nudgeUpCommand());
        xbox.climb.onTrue(climber.climbCommand());
        xbox.neutral.onTrue(climber.neutralCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(String auto){
        // An example command will be run in autonomous
        try{
            if(pathfindAtStart){
                List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(auto);
                if (!pathGroup.isEmpty()) {
                    var initPath = pathGroup.get(0);
                    Pose2d initPose = new Pose2d(initPath.getAllPathPoints().get(0).position,initPath.getIdealStartingState().rotation());
                    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
                    PathConstraints constraints = new PathConstraints(
                            3.0, 4.0,
                            Units.degreesToRadians(540), Units.degreesToRadians(720));

                    // Since AutoBuilder is configured, we can use it to build pathfinding commands
                    return AutoBuilder.pathfindToPoseFlipped(initPose,constraints,initPath.getIdealStartingState().velocity()).andThen(autoChooser.getSelected());
                }
            }
        } catch (IOException | org.json.simple.parser.ParseException e){
            System.out.println("Error: " + e.getMessage());
        }

        return autoChooser.getSelected();
    }
}
