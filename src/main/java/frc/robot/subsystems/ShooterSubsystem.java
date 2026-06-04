// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
    private static final int FLOOR_CAN_ID = 30;

    // private static final double[][] pairsForDist = {{2.24, 0.75}, {2.43, 0.77}, {2.93, 0.813}};

    private static boolean distanceShoot = false;
    private boolean isShooting = false;

    private static double shooterSpeed = 0.75;
    private static double linearBump = 0;

    // create a velocity closed-loop request, voltage output, slot 0 configs
    final VelocityVoltage request_indexer = new VelocityVoltage(0).withSlot(0);
    final VelocityVoltage request_shooter = new VelocityVoltage(0).withSlot(0);
    final VelocityVoltage request_intake = new VelocityVoltage(0).withSlot(0);
    final VelocityVoltage request_floor = new VelocityVoltage(0).withSlot(0);

    private final TalonFX indexerMotor;
    private final TalonFX shooterMotor;
    private final TalonFX shooter2Motor;
    private final TalonFX intakeMotor;
    private final TalonFX floorMotor;

    /**
     * Initialize all of the shooter motors with PID values in slot 0
     */
    public ShooterSubsystem() {
        indexerMotor = new TalonFX(INDEXER_CAN_ID);
        shooterMotor = new TalonFX(SHOOTER_CAN_ID);
        shooter2Motor = new TalonFX(SHOOTER_2_CAN_ID);
        intakeMotor = new TalonFX(INTAKE_CAN_ID);
        floorMotor = new TalonFX(FLOOR_CAN_ID);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        shooterConfig.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
        shooterConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        shooterConfig.Slot0.kP = 0.4; // An error of 1 rps results in 0.4 V output
        shooterConfig.Slot0.kI = 0; // no output for integrated error
        shooterConfig.Slot0.kD = 0; // no output for error derivative

        shooterConfig.CurrentLimits.SupplyCurrentLimit = 40;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        shooterMotor.getConfigurator().apply(shooterConfig);

        shooter2Motor.setControl(new Follower(SHOOTER_CAN_ID, MotorAlignmentValue.Opposed));

        TalonFXConfiguration otherConfig = new TalonFXConfiguration();

        otherConfig.Slot0.kP = 0.2;
        otherConfig.Slot0.kA = 0;
        otherConfig.CurrentLimits.SupplyCurrentLimit = 40;
        otherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        otherConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        indexerMotor.getConfigurator().apply(otherConfig);
        intakeMotor.getConfigurator().apply(otherConfig);

        TalonFXConfiguration floorConfig = new TalonFXConfiguration();

        floorConfig.Slot0.kP = 0;
        floorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        floorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        floorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        floorMotor.getConfigurator().apply(floorConfig);
    }

    /**
     * Convert a distance from the basket to a target speed for shooting into it
     */
    public double shootFunction(double distance) {
        // double adjustedDist = distance - 0.5171;
        // return 0.7 * Math.pow(Math.tan(1.13446) / adjustedDist - 1.8288 / (adjustedDist * adjustedDist), -0.5);
        // if (distance < 2.60) {
        //     return ((2.60 - 2.34) / (0.84 - 0.81)) * (distance - 2.60) + 0.74;
        // }
        // if (distance < 2.96) {
        //     return ((2.96 - 2.60) / (0.905 - 0.84)) * (distance - 2.96) + 0.805;
        // }
        // if (distance < 3.45) {
        //     return ((3.45 - 2.96) / (0.925 - 0.905)) * (distance - 3.45) + 0.825;
        // }
        // if (distance < 3.79) {
        //     return ((3.79 - 3.45) / (0.95 - 0.925)) * (distance - 3.79) + 0.85;
        // }
        // return ((3.92 - 3.79) / (0.97 - 0.95)) * (distance - 3.92) + 0.87;

        // for (int i = 1; i < pairsForDist.length; i++) {
        //     if (distance < pairsForDist[i][0]) {
        //         return ((pairsForDist[i][1] - pairsForDist[i - 1][1]) / (pairsForDist[i][0] - pairsForDist[i -
        // 1][0]))
        //                         * (distance - pairsForDist[i][0])
        //                 + pairsForDist[i][1];
        //     }
        // }
        // return ((pairsForDist[pairsForDist.length - 1][1] - pairsForDist[pairsForDist.length - 2][1])
        //                         / (pairsForDist[pairsForDist.length - 1][0] - pairsForDist[pairsForDist.length -
        // 2][0]))
        //                 * (distance - pairsForDist[pairsForDist.length - 1][0])
        //         + pairsForDist[pairsForDist.length - 1][1];

        return 0.0859544 * distance + 0.498184 + 0.015 + linearBump;
    }

    public void spinIndexer(double rps) {
        indexerMotor.setControl(request_indexer.withVelocity(rps).withFeedForward(0));
    }

    public void stopIndexer() {
        indexerMotor.set(0);
    }

    public void spinShooter(double rps) {
        shooterMotor.setControl(request_shooter.withVelocity(-rps).withFeedForward(0));
    }

    public void stopShooter() {
        shooterMotor.set(0);
    }

    public void spinIntake(double rps) {
        intakeMotor.setControl(request_intake.withVelocity(-rps).withFeedForward(0));
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void spinFloor(double rps) {
        floorMotor.set(2 * rps / 100);
    }

    public void stopFloor() {
        floorMotor.set(0);
    }

    public double getSetSpeed() {
        return shooterSpeed;
    }

    /**
     * Increase the shooter speed offset, used if we are consistenly short of the basket
     */
    public void nudgeUp() {
        if (linearBump < 1) {
            linearBump += 0.001;
        }
    }

    /**
     * Reduce the shooter speed offset, used if we are consistently shooting too far past the basket
     */
    public void nudgeDown() {
        if (linearBump > -1) {
            linearBump -= 0.001;
        }
    }

    public boolean getIsShooting() {
        return isShooting;
    }

    /**
     * Lock the wheels in an X pattern to make it harder for defense bots to push and then start shooting
     */
    public Command shooterCommand(SwerveSubsystem swerve) {
        return this.runOnce(() -> {
                    isShooting = true;
                    stopIntake();
                    spinShooter(shooterSpeed * 100);
                })
                .andThen(Commands.waitSeconds(0.6))
                .andThen(this.runOnce(() -> {
                    double speed = shooterSpeed;
                    spinIndexer(75 + 25 * speed);
                    spinShooter(speed * 100);
                    spinIntake(50 + 50 * speed);
                    spinFloor(speed * 40);
                }));
    }

    public Command distShootCommand() {
        return this.runOnce(() -> {
            distanceShoot = true;
        });
    }

    public Command manualShootCommand() {
        return this.runOnce(() -> {
            distanceShoot = false;
        });
    }

    public Command intakeCommand() {
        return this.runOnce(() -> {
            stopShooter();
            spinIndexer(-120);
            spinIntake(108);
            // spinFloor(-40);
        });
    }

    public Command stopCommand() {
        return this.runOnce(() -> {
            stopIndexer();
            stopShooter();
            stopIntake();
            stopFloor();
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
