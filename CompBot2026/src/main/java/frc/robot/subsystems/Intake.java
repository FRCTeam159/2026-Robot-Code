
package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;

public class Intake extends SubsystemBase {
    private Motor intake_motor_1 = new Motor(intake_1);
    //private Motor roller_motor = new Motor(hopper_roller);


    public Intake() {
        setConfig();
    }

    public void setConfig() {
        intake_motor_1.setConfig(false, false, 1, 0.001, 0);
        //roller_motor.setConfig(false, 1);
    }

    public void intake(double speed) {
        intake_motor_1.setVelocity(speed);
        //roller_motor.set(roller_speed);
    }
}
