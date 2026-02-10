/*
all of this is coded for the basic everybot design, as we change our robot this code will need to be changed
a good bit of the code in here should also be explained by reading the code the everybot comes with
*/
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX leftshooter;
    private TalonFX rightshooter;
    private TalonFX indexer;

    public ShooterSubsystem() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
