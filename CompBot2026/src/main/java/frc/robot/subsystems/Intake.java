
package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;

public class Intake extends SubsystemBase {
    private Motor intake_motor_1 = new Motor(intake_1);
    //private Motor roller_motor = new Motor(hopper_roller);

    //horizontal distance from the axis of rotation to the winch perpendicular to the axis of rotation
    private final double diff_x = 0.0;
    //vertical distance from the axis of rotation to the winch
    private final double diff_y = 0.0;
    //horizontal distance from the axis of rotation to the winch parallel to the axis of rotation
    private final double diff_z = 0.0;
    //length of the arm from the axis of rotation to the end of the winch
    private final double arm_length = 0.0;

    public Intake() {
        setConfig();
    }

    public void setConfig() {
        intake_motor_1.setConfig(false, false, 1, 0.001, 0);
    }

    public void intake(double speed) {
        intake_motor_1.setVelocity(speed);
    }

    public void goToAngle(double angle){
        //FIX THIS MATH LATER
        //final double arm_end_x = 
        goToDistance(angle);
    }

    public void goToDistance(double distance){

    }
}