package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XBoxContainer {
    private XboxController controller = new XboxController(0);

    public JoystickButton runIntake = new JoystickButton(controller, XboxController.Button.kY.value);
    public JoystickButton runShooter = new JoystickButton(controller, XboxController.Button.kB.value);
    public JoystickButton stop = new JoystickButton(controller, XboxController.Button.kX.value);
    public JoystickButton reset = new JoystickButton(controller, XboxController.Button.kA.value);
    public JoystickButton climb = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
    public JoystickButton neutral = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
    // public Trigger align = new Trigger(() -> controller.getLeftTriggerAxis() > 0.1);

    public Trigger dpadDown = new Trigger(() -> controller.getPOV() == 180);
    public Trigger dpadUp = new Trigger(() -> controller.getPOV() == 0);

    public boolean getControllerXButton() {
        return (controller.getLeftStickButton());
    }

    public double driveX() {
        if (Math.abs(controller.getLeftX()) >= 0.1) {
            return (controller.getLeftX());
        } else {
            return (0);
        }
    }

    public double driveY() {
        if (Math.abs(controller.getLeftY()) >= 0.1) {
            return (controller.getLeftY());
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
