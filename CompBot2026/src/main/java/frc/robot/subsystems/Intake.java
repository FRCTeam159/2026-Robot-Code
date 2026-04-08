
package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;

public class Intake extends SubsystemBase {
    private Motor intake_motor_1 = new Motor(intake_1);
    private Motor winch_motor = new Motor(intake_winch);

    private final double closed_rotations = 0;
    private final double open_rotations = 3.7;
    private final double shooting_rotations = 1.25;

    private final double winch_gear_ratio = 90.0;

    private double target_rotations = 0.0;

    private PIDController winch_pid_controller = new PIDController(0.6, 0, 0);

    public Intake() {
        setConfig();
    }

    public void setConfig() {
        intake_motor_1.setConfig(false, false, 1, 0.001, 0);

        //winch_motor.setLowerLimit(true);
        winch_motor.setConfig(false, true, 1 / winch_gear_ratio);

        winch_motor.reset();
    }

    public void intake(double speed) {
        intake_motor_1.setVelocity(speed);
    }

    public void close_intake(){
        target_rotations = closed_rotations;
    }

    public void open_intake(){
        target_rotations = open_rotations;
    }

    public void goto_shoot_position(){
        target_rotations = shooting_rotations;
    }

    public void spin(){
        double speed = winch_pid_controller.calculate(getPos(), target_rotations);
        winch_motor.set(speed);
    }

    public void stop_spinning(){
        winch_motor.set(0);
    }

    public void change_target(double diff){
        target_rotations += diff;
        target_rotations = Math.min(Math.max(target_rotations, closed_rotations), open_rotations);
    }

    public void runWinch(double speed) {
        winch_motor.set(speed);
    }

    public void armToAngle(double speed, double angle) {

        winch_motor.set(-speed);
    }

    public double getPos() {
        return winch_motor.getPosition();
    }

    public double getTarget() {
        return target_rotations;
    }

}