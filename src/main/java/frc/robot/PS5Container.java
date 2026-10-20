package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS5Container {
    private PS5Controller controller = new PS5Controller(1);

    public JoystickButton runIntake = new JoystickButton(controller, PS5Controller.Button.kTriangle.value);
    public JoystickButton runShooter = new JoystickButton(controller, PS5Controller.Button.kCircle.value);
    public JoystickButton stopShooter = new JoystickButton(controller, PS5Controller.Button.kSquare.value);
    public JoystickButton reset = new JoystickButton(controller, PS5Controller.Button.kCross.value);
    public JoystickButton backClimb = new JoystickButton(controller, PS5Controller.Button.kL1.value);
    public JoystickButton neutralClimb = new JoystickButton(controller, PS5Controller.Button.kR1.value);
    public JoystickButton align = new JoystickButton(controller, PS5Controller.Button.kL2.value);
    public JoystickButton distShoot = new JoystickButton(controller, PS5Controller.Button.kR2.value);

    public Trigger dpadDown = new Trigger(() -> controller.getPOV() == 180);
    public Trigger dpadUp = new Trigger(() -> controller.getPOV() == 0);
    public Trigger dpadLeft = new Trigger(() -> controller.getPOV() == 270);

    public double driveX() {
        if (Math.abs(controller.getLeftX()) >= 0.1) {
            return Math.signum(controller.getLeftX()) * (Math.abs(controller.getLeftX()) - 0.1) / 0.9;
        } else {
            return (0);
        }
    }

    public double driveY() {
        if (Math.abs(controller.getLeftY()) >= 0.1) {
            return Math.signum(controller.getLeftY()) * (Math.abs(controller.getLeftY()) - 0.1) / 0.9;
        } else {
            return (0);
        }
    }

    public double rotate() {
        if (Math.abs(controller.getRightX()) >= 0.1) {
            return Math.signum(controller.getRightX()) * (Math.abs(controller.getRightX()) - 0.1) / 0.9;
        } else {
            return (0);
        }
    }
}
