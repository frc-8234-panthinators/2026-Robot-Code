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

    private static final double kP = 1.75;
    private static final double kI = 0.5;
    private static final double kD = 0.1;
    private static final double kV = 0; // Feed Forward

    private static double shooterSpeed = 0.83;

    private static final double MAX_VELOCITY = 30;
    private static final double MAX_ACCELERATION = 40;

    private final TalonFX indexerMotor;
    private final TalonFX shooterMotor;
    private final TalonFX shooter2Motor;

    public ShooterSubsystem() {
        indexerMotor = new TalonFX(INDEXER_CAN_ID);
        shooterMotor = new TalonFX(SHOOTER_CAN_ID);
        shooter2Motor = new TalonFX(SHOOTER_2_CAN_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;

        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        
        shooterMotor.getConfigurator().apply(config);
        shooter2Motor.setControl(new Follower(SHOOTER_CAN_ID, MotorAlignmentValue.Opposed));
        

        config.Slot0.kP = 0.5;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.1;
        config.Slot0.kV = 0.6;

        indexerMotor.getConfigurator().apply(config);
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

    public void nudgeUp() {
        if (shooterSpeed < 1) {
            shooterSpeed += 0.01;
        }
    }

    public void nudgeDown() {
        if (shooterSpeed > 0) {
            shooterSpeed -= 0.01;
        }
    }

    public Command shooterCommand() {
        return this.runOnce(() -> {
                    spinShooter(shooterSpeed);
                })
                .andThen(Commands.waitSeconds(0.7))
                .andThen(this.runOnce(() -> {
                    spinIndexer(shooterSpeed);
                }));
    }

    public Command intakeCommand() {
        return this.runOnce(() -> {
            spinIndexer(-0.5);
            spinShooter(0.7);
        });
    }

    public Command stopCommand() {
        return this.runOnce(() -> {
            stopIndexer();
            stopShooter();
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
