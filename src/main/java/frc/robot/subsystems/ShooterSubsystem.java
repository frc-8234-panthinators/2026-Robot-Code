// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private static final int SHOOTER_CAN_ID = 20;
    private static final int INDEXER_CAN_ID = 21;
    private static final int SHOOTER_2_CAN_ID = 22;
    private static final int INTAKE_CAN_ID = 23;

    private static final double kS = 0.12;
    private static final double kV = 0.15;
    private static final double kA = 0.03;

    private static boolean distanceShoot = true;

    private static double shooterSpeed = 0.75;

    private static final double MAX_VELOCITY = 30;
    private static final double MAX_ACCELERATION = 40;

    private final TalonFX indexerMotor;
    private final TalonFX shooterMotor;
    private final TalonFX shooter2Motor;
    private final TalonFX intakeMotor;

    public ShooterSubsystem() {
        indexerMotor = new TalonFX(INDEXER_CAN_ID);
        shooterMotor = new TalonFX(SHOOTER_CAN_ID);
        shooter2Motor = new TalonFX(SHOOTER_2_CAN_ID);
        intakeMotor = new TalonFX(INTAKE_CAN_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;

        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterMotor.getConfigurator().apply(config);
        shooter2Motor.setControl(new Follower(SHOOTER_CAN_ID, MotorAlignmentValue.Opposed));

        config.Slot0.kS = 0.12;
        config.Slot0.kV = 0.15;
        config.Slot0.kA = 0.03;
        config.CurrentLimits.SupplyCurrentLimit = 40;

        indexerMotor.getConfigurator().apply(config);
        intakeMotor.getConfigurator().apply(config);
    }

    public double shootFunction(double distance) {
        double adjustedDist = distance - 0.5171;
        return 0.6 * Math.pow(Math.tan(1.13446) / adjustedDist - 1.8288 / (adjustedDist * adjustedDist), -0.5);
    }

    public void spinIndexer(double value) {
        indexerMotor.set(value);
    }

    public void stopIndexer() {
        indexerMotor.set(0);
    }

    public void spinShooter(double value) {
        shooterMotor.set(-value);
    }

    public void stopShooter() {
        shooterMotor.set(0);
    }

    public void spinIntake(double value) {
        intakeMotor.set(-value);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public double getSetSpeed() {
        return shooterSpeed;
    }

    public void nudgeUp() {
        distanceShoot = false;
        if (shooterSpeed < 1) {
            shooterSpeed += 0.01;
        }
    }

    public void nudgeDown() {
        distanceShoot = false;
        if (shooterSpeed > 0) {
            shooterSpeed -= 0.01;
        }
    }

    public Command shooterCommand(double distance) {
        return this.runOnce(() -> {
                    stopIntake();
                    spinShooter(shooterSpeed);
                })
                .andThen(Commands.waitSeconds(0.5))
                .andThen(this.runOnce(() -> {
                    shooterSpeed = (distanceShoot) ? shootFunction(distance) : shooterSpeed;
                    spinIndexer(0.5 + 0.5 * shooterSpeed);
                    spinShooter(shooterSpeed);
                    spinIntake(shooterSpeed);
                }));
    }

    public Command intakeCommand() {
        return this.runOnce(() -> {
            stopShooter();
            // spinIndexer(-0.5);
            spinIntake(1);
        });
    }

    public Command stopCommand() {
        return this.runOnce(() -> {
            stopIndexer();
            stopShooter();
            stopIntake();
        });
    }

    public Command nudgeUpCommand() {
        return this.runOnce(() -> {
            nudgeUp();
        });
    }

    public Command nudgeDownCommand() {
        return this.runOnce(() -> {
            nudgeDown();
        });
    }

    public double[] getSpeeds() {
        double[] speeds = new double[3];
        speeds[0] = shooterMotor.getVelocity().getValueAsDouble();
        speeds[1] = indexerMotor.getVelocity().getValueAsDouble();
        speeds[2] = shooter2Motor.getVelocity().getValueAsDouble();
        return speeds;
    }
}
