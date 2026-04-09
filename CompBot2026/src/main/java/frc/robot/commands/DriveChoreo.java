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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.EventMarker;

public class DriveChoreo extends Command {
    Drivetrain m_drive;
    Shooter m_shooter;
    Intake m_intake;

    double more_time;

    private double rotation_offset;

    boolean intaking = false;
    boolean shooting = false;
    boolean deploying = false;

    boolean isFirstPath;

    final PIDController x_PID_controller = new PIDController(2, 0, 0);
    final PIDController y_PID_controller = new PIDController(2, 0, 0);
    final PIDController r_PID_controller = new PIDController(2, 0, 0);

    double top_speed = 5600;
    double bottom_speed = -100;
    double intake_speed = 3000;

    final double horizontal_coeff = 6;
    final double rotation_coeff = 3;

    private String file_path;

    Optional<Trajectory<SwerveSample>> trajectory;

    List<EventMarker> events;

    private double previous_sample = -1.0;
    private double previous_rotation = 0.0;

    private double targetAngle;

    private Timer m_timer = new Timer();

    public DriveChoreo(Drivetrain drive, Shooter shooter, Intake intake, String path, double extra_duration, boolean firstPath) {
        m_drive = drive;
        m_shooter = shooter;
        m_intake = intake;
        file_path = path;
        more_time = extra_duration;

        isFirstPath = firstPath;

        addRequirements(drive, shooter, intake);
    }

    @Override
    public void initialize() {
        trajectory = Choreo.loadTrajectory(file_path);

        events = trajectory.get().events();

        Pose2d start_pose = trajectory.get().sampleAt(0, false).get().getPose();
        
        if (isFirstPath) {
            m_drive.resetOdometry(start_pose);
        }

        previous_rotation = start_pose.getRotation().getRadians();
        
        rotation_offset = start_pose.getRotation().getRadians();

        m_timer.reset();
        m_timer.start();
    }

    private void proccessEvents(EventMarker event, double sample_time){
        if (previous_sample < event.timestamp && sample_time >= event.timestamp) {
            switch (event.event) {
                default:
                    break;
                case "Deploy_Intake" :
                    m_intake.open_intake();
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

        top_speed = SmartDashboard.getNumber("Top Shoot Speed", top_speed) / 60;
        bottom_speed = SmartDashboard.getNumber("Bottom Shoot Speed", bottom_speed) / 60;
        intake_speed = SmartDashboard.getNumber("Intake Speed", intake_speed) / 60;

        if (shooting) {
            m_shooter.shoot(top_speed / 60, bottom_speed / 60, 0, 0);
        }
        m_intake.intake(intaking ? intake_speed : 0);

        m_intake.spin();

        previous_sample = sample_time;

        Pose2d current_pose = m_drive.getPose();

        double angle_diff = target_pose.getRotation().getDegrees() - previous_rotation;

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
    }

    @Override
    public void end(boolean interrupted){
        m_drive.drive(0, 0, 0, false);
        m_intake.stop_spinning();
    }
}
