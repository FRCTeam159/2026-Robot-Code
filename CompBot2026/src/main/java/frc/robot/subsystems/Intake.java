
package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;

public class Intake extends SubsystemBase {
    private Motor intake_motor_1 = new Motor(intake_1);
    private Motor intake_motor_2 = new Motor(intake_2);

    public Intake() {
        setConfig();
    }

    public void setConfig() {
        intake_motor_1.setConfig(false, false, 1, 0.001);
        intake_motor_2.setConfig(true, false, 1, 0.001);
    }

    public void intake(double speed) {
        intake_motor_1.setVelocity(speed);
        intake_motor_2.setVelocity(speed);
    }
}
