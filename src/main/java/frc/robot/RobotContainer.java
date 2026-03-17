// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
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

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(SwerveSubsystem swerve, VisionSubsystem vision, XBoxContainer xbox) {
        this.xbox = xbox;
        shooter = new ShooterSubsystem();
        this.swerve = swerve;
        this.vision = vision;

        NamedCommands.registerCommand("ResetHeading", swerve.resetHeading());
        NamedCommands.registerCommand("Intake", shooter.intakeCommand());
        NamedCommands.registerCommand("Shoot", shooter.shooterCommand());
        NamedCommands.registerCommand("StopShooter", shooter.stopCommand());
        NamedCommands.registerCommand("Align", swerve.alignCommand());

        // Configure the trigger bindings
        configureBindings();
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
        xbox.runShooter.onTrue(shooter.shooterCommand());
        xbox.stop.onTrue(shooter.stopCommand().andThen(swerve.stopAlignCommand()));
        xbox.reset.onTrue(swerve.resetHeading());
        xbox.align.onTrue(swerve.alignCommand());
        xbox.dpadDown.onTrue(shooter.nudgeDownCommand());
        xbox.dpadUp.onTrue(shooter.nudgeUpCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new PathPlannerAuto("ShootAuto");
    }
}
