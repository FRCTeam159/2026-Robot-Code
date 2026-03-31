// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.*;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Test;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.EventMarker;

public class DriveChoreo extends Command {
    Drivetrain m_drive;
    Test m_shooter;

    double more_time;

    boolean shooting = false;

    boolean isFirstPath;

    final PIDController x_PID_controller = new PIDController(1.5, 0, 0);
    final PIDController y_PID_controller = new PIDController(1.5, 0, 0);
    final PIDController r_PID_controller = new PIDController(1.5, 0, 0);

    final double horizontal_coeff = 6.0;
    final double rotation_coeff = 3.0;

    private double start_rotation = 0.0;

    private String file_path;

    Optional<Trajectory<SwerveSample>> trajectory;

    List<EventMarker> events;

    private double previous_sample = -1.0;
    private double previous_rotation = 0.0;

    private double previous_is_finished_sample = 0.0;
    private Pose2d previous_pose = new Pose2d();

    private final double position_threshold = 0.025;
    private final double rotation_threshold = 2;

    private final double max_end_velocity_linear = 0.2;
    private final double max_end_velocity_rotation = 5;

    private Timer m_timer = new Timer();

    public DriveChoreo(Drivetrain drive, Test shooter, String path, double extra_duration, boolean firstPath) {
        m_drive = drive;
        m_shooter = shooter;
        file_path = path;
        more_time = extra_duration;

        isFirstPath = firstPath;

        addRequirements(drive, shooter);
    }

    @Override
    public void initialize() {
        trajectory = Choreo.loadTrajectory(file_path);

        events = trajectory.get().events();

        previous_pose = trajectory.get().sampleAt(0, false).get().getPose();
        if (isFirstPath) {
            m_drive.resetOdometry(previous_pose);
        }
        
        start_rotation = previous_pose.getRotation().getRadians();
        previous_rotation = previous_pose.getRotation().getDegrees();

        m_timer.reset();
        m_timer.start();
    }

    private void proccessEvents(EventMarker event, double sample_time){
        if (previous_sample < event.timestamp && sample_time >= event.timestamp) {
            switch (event.event) {
                default:
                    break;
                case "A":
                    shooting = true;
                    break;
                case "B":
                    shooting = false;
                    break;
            }
        }
    }

    @Override
    public void execute() {
        double sample_time = m_timer.get();
        Optional<SwerveSample> sample = trajectory.get().sampleAt(Math.min(sample_time, trajectory.get().getTotalTime()), false);

        Pose2d target_pose = sample.get().getPose();
        ChassisSpeeds target_speed = sample.get().getChassisSpeeds();

        events.forEach((EventMarker event) -> proccessEvents(event, sample_time));

        m_shooter.shoot(shooting ? 0.5 : 0);

        previous_sample = sample_time;

        Pose2d current_pose = m_drive.getPose();

        double angle_diff = target_pose.getRotation().getDegrees() - previous_rotation;

        //Unwraps the angle when it passes 180 degrees
        double target_angle = previous_rotation + (angle_diff - 360 * Math.round(angle_diff / 360));
        previous_rotation = target_angle;
        target_angle *= Math.PI/180;

        System.out.println(String.format("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", current_pose.getX(), current_pose.getY(), x_PID_controller.calculate(current_pose.getX(), target_pose.getX()), y_PID_controller.calculate(current_pose.getY(), target_pose.getY()), target_pose.getX(), target_pose.getY()));

        double vx = x_PID_controller.calculate(current_pose.getX(), target_pose.getX()) + target_speed.vxMetersPerSecond / horizontal_coeff;
        double vy = y_PID_controller.calculate(current_pose.getY(), target_pose.getY()) + target_speed.vyMetersPerSecond / horizontal_coeff;

        m_drive.drive(
            vx,
            vy,
            r_PID_controller.calculate(m_drive.getRotation2d().getRadians(), target_angle) + target_speed.omegaRadiansPerSecond / rotation_coeff,
            true
        );
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() >= trajectory.get().getTotalTime() + more_time;
        // double time = m_timer.get();
        // double duration = trajectory.get().getTotalTime();

        // Pose2d current_pose = m_drive.getPose();
        // double delta_t = time - previous_is_finished_sample;

        // double delta_x = current_pose.getX() - previous_pose.getX();
        // double delta_y = current_pose.getY() - previous_pose.getY();
        // double delta_r = current_pose.getRotation().getDegrees() - previous_pose.getRotation().getDegrees();

        // double x_velocity = delta_x / delta_t;
        // double y_velocity = delta_y / delta_t;
        // double r_velocity = delta_r / delta_t;

        // previous_is_finished_sample = time;
        // previous_pose = current_pose;

        // Optional<SwerveSample> sample = trajectory.get().sampleAt(trajectory.get().getTotalTime(), false);
        // Pose2d target_pose = sample.get().getPose();

        // double diff_x = target_pose.getX() - current_pose.getX();
        // double diff_y = target_pose.getY() - current_pose.getY();
        // double diff_r = target_pose.getRotation().getDegrees() - current_pose.getRotation().getDegrees();

        // //System.out.println(String.format("VX: %.2f, VY: %.2f, VR: %.2f", x_velocity, y_velocity, r_velocity));
        // //System.out.println(String.format("X: %.2f, Y: %.2f, R: %.2f", diff_x, diff_y, diff_r));

        // if (time >= duration + more_time ){
        //     System.out.println("Terminated choreo sequence: Out of time");
        // } else if(time >= duration &&
        // (Math.abs(x_velocity) < max_end_velocity_linear) &&
        // (Math.abs(y_velocity) < max_end_velocity_linear) &&
        // (Math.abs(r_velocity) < max_end_velocity_rotation) &&
        // (Math.abs(diff_x) < position_threshold) &&
        // (Math.abs(diff_y) < position_threshold) &&
        // (Math.abs(diff_r) < rotation_threshold)
        // ){
        //     System.out.println("Terminated choreo sequence: At Target Position");
        // }

        // return (time >= duration + more_time || (time >= duration &&
        // (Math.abs(x_velocity) < max_end_velocity_linear) &&
        // (Math.abs(y_velocity) < max_end_velocity_linear) &&
        // (Math.abs(r_velocity) < max_end_velocity_rotation) &&
        // (Math.abs(diff_x) < position_threshold) &&
        // (Math.abs(diff_y) < position_threshold) &&
        // (Math.abs(diff_r) < rotation_threshold)
        // ));
    }
}
