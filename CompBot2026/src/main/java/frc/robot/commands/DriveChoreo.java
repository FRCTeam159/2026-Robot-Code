// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.*;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.EventMarker;

public class DriveChoreo extends Command {
    Drivetrain m_drive;
    Shooter m_shooter;
    Intake m_intake;

    double more_time;

    boolean intaking = false;
    boolean shooting = false;

    final PIDController x_PID_controller = new PIDController(2.5, 0, 0);
    final PIDController y_PID_controller = new PIDController(2.5, 0, 0);
    final PIDController r_PID_controller = new PIDController(3, 0, 0);

    final double horizontal_coeff = 1.0;
    final double rotation_coeff = 1.0;

    private String file_path;

    Optional<Trajectory<SwerveSample>> trajectory;

    List<EventMarker> events;

    private double previous_sample = -1.0;

    private Timer m_timer = new Timer();

    public DriveChoreo(Drivetrain drive, Shooter shooter, Intake intake, String path, double extra_duration) {
        m_drive = drive;
        m_shooter = shooter;
        m_intake = intake;
        file_path = path;
        more_time = extra_duration;

        addRequirements(drive, shooter, intake);
    }

    @Override
    public void initialize() {
        trajectory = Choreo.loadTrajectory(file_path);

        events = trajectory.get().events();

        m_drive.resetOdometry(trajectory.get().sampleAt(0, false).get().getPose());
        System.out.println(m_drive.getPose());
        
        m_timer.reset();
        m_timer.start();
    }

    private void proccessEvents(EventMarker event, double sample_time){
        if (previous_sample < event.timestamp && sample_time >= event.timestamp) {
            switch (event.event) {
                default:
                    break;
                case "Intake_Start":
                    intaking = true;
                    break;
                case "Intake_End":
                    intaking = false;
                    break;
                case "Shoot_Start":
                    shooting = true;
                    break;
                case "Shoot_End":
                    shooting = false;
                    break;
            }
        }
    }

    @Override
    public void execute() {
        double sample_time = m_timer.get();
        Optional<SwerveSample> sample = trajectory.get().sampleAt(sample_time, false);

        Pose2d target_pose = sample.get().getPose();
        ChassisSpeeds target_speed = sample.get().getChassisSpeeds();

        events.forEach((EventMarker event) -> proccessEvents(event, sample_time));

        m_intake.intake(intaking ? 50 : 0);

        previous_sample = sample_time;

        Pose2d current_pose = m_drive.getPose();

        m_drive.drive(
            x_PID_controller.calculate(current_pose.getX(), target_pose.getX()) + target_speed.vxMetersPerSecond / 2,
            y_PID_controller.calculate(current_pose.getY(), target_pose.getY()) + target_speed.vyMetersPerSecond / 2,
            r_PID_controller.calculate(current_pose.getRotation().getRadians(), target_pose.getRotation().getRadians()) + target_speed.omegaRadiansPerSecond / 3,
            true
        );
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() >= trajectory.get().getTotalTime() + more_time;
    }
}
