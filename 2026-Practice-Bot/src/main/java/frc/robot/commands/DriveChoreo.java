// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.*;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drivetrain;
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

public class DriveChoreo extends Command {
    Drivetrain m_drive;

    final PIDController x_PID_controller = new PIDController(1.5, 0, 0);
    final PIDController y_PID_controller = new PIDController(1.5, 0, 0);
    final PIDController r_PID_controller = new PIDController(1.5, 0, 0);

    final double horizontal_coeff = 1.0;
    final double rotation_coeff = 1.0;

    private String file_path;

    Optional<Trajectory<SwerveSample>> trajectory;

    private Timer m_timer = new Timer();

    public DriveChoreo(Drivetrain drive, String path) {
        m_drive = drive;
        file_path = path;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        trajectory = Choreo.loadTrajectory(file_path);

        m_drive.resetOdometry(trajectory.get().sampleAt(0, false).get().getPose());
        
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        Optional<SwerveSample> sample = trajectory.get().sampleAt(m_timer.get(), false);
        
        Pose2d target_pose = sample.get().getPose();
        ChassisSpeeds target_speed = sample.get().getChassisSpeeds();

        Pose2d current_pose = m_drive.getPose();

        m_drive.drive(
            x_PID_controller.calculate(current_pose.getX(), target_pose.getX()) + target_speed.vxMetersPerSecond / 2.5,
            y_PID_controller.calculate(current_pose.getY(), target_pose.getY()) + target_speed.vyMetersPerSecond / 2.5,
            r_PID_controller.calculate(current_pose.getRotation().getRadians(), target_pose.getRotation().getRadians()) + target_speed.omegaRadiansPerSecond / 2,
            true
        );
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() >= trajectory.get().getTotalTime();
    }
}
