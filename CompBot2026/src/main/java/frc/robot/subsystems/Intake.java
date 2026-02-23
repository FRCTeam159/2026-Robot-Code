
package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;

public class Intake extends SubsystemBase {
    private Motor intake_motor = new Motor(intake_id);

    public Intake() {
        setConfig();
    }

    public void setConfig() {
        intake_motor.setConfig(false, false, 1, 0.001);
    }

    public void intake(double speed) {
        intake_motor.setVelocity(speed);
    }
}
