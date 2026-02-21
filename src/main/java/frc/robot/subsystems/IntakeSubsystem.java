// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private static final int KRAKEN_MOTOR_CAN_ID = 20; // TODO: Change to correct port :)

    private static final double kP = 0.5;
    private static final double kI = 0.0;
    private static final double kD = 0.1;
    private static final double kV = 0.6; // Feed Forward

    private static final double MAX_VELOCITY = 30;
    private static final double MAX_ACCELERATION = 40;

    private final TalonFX krakenMotor;

    public IntakeSubsystem() {
        krakenMotor = new TalonFX(KRAKEN_MOTOR_CAN_ID);

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

        krakenMotor.getConfigurator().apply(config);
    }

    public void intake() {
        krakenMotor.set(0.4);
    }

    public void stop() {
        krakenMotor.set(0);
    }

    public Command intakeCommand() {
        return run(() -> {
            intake();
        });
    }

    public Command stopCommand() {
        return run(() -> {
            stop();
        });
    }
}
