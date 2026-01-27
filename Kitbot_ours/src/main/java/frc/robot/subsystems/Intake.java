
package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;

public class Intake extends SubsystemBase {
    private Motor intake_motor = new Motor(intake_id, true);

    public Intake() {
        init();
    }

    public void setConfig() {
        intake_motor.setConfig(false, 1);
    }

    public void init() {
        setConfig();
    }

    public void set(double speed) {
        intake_motor.set(speed);
    }
}
