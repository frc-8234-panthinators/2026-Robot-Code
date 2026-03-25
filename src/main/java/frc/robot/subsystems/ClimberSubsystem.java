// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class ClimberSubsystem extends SubsystemBase {
    private static final int CLIMBER_CAN_ID = 24;

    private static final double kP = 0.5;
    private static final double kI = 0.0;
    private static final double kD = 0.1;
    private static final double kV = 0.6; // Feed Forward

    private static final double MAX_VELOCITY = 30;
    private static final double MAX_ACCELERATION = 40;

    private final TalonFX climberMotor;

    private final PositionVoltage positionRequest;

    private double position = 0.0;

    public ClimberSubsystem() {
        climberMotor = new TalonFX(CLIMBER_CAN_ID);

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

        climberMotor.getConfigurator().apply(config);

        positionRequest = new PositionVoltage(0).withSlot(0);

        climberMotor.getConfigurator().setPosition(0.0);
    }

    private void climb(double position) {
        this.position = position;
        climberMotor.setControl(positionRequest.withPosition(position));
    }

    public Command climbCommand() {
        return runOnce(() -> {
                    climb(200.0);
                })
                .andThen(Commands.waitSeconds(3))
                .andThen(run(() -> {
                    climb(-100.0);
                }));
    }

    public Command neutralCommand() {
        return runOnce(() -> {
            climb(0.0);
        });
    }

    // Emergency stop
    public Command stopClimberCommand() {
        return run(() -> {
            climberMotor.stopMotor();
        });
    }

    public double getPosition() {
        return climberMotor.getPosition().getValueAsDouble();
    }
}
