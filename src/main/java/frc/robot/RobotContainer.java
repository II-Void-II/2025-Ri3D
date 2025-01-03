// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;
import com.techhounds.houndutil.houndlib.subsystems.BaseDifferentialDrive.DifferentialDriveMode;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.SendableLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    CommandXboxController controller = new CommandXboxController(0);
    CommandVirpilJoystick joystick = new CommandVirpilJoystick(0);

    @Log
    Drivetrain drivetrain = new Drivetrain();

    @SendableLog
    CommandScheduler scheduler = CommandScheduler.getInstance();

    public RobotContainer() {
        configureBindings();
        configureAuto();
        LoggingManager.getInstance().registerObject(this);
        URCL.start();
    }

    private void configureBindings() {
        // xbox
        // drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(() ->
        // -controller.getLeftY(),
        // () -> -controller.getRightY(), () -> -controller.getRightX(), () ->
        // DifferentialDriveMode.ARCADE));

        // virpil
        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(
                () -> joystick.getY(),
                // https://www.chiefdelphi.com/t/single-joystick-tank-drive/109331/7
                () -> joystick.getX() * (joystick.getY() > 0 ? 1 : -1),
                () -> -controller.getRightX(),
                () -> DifferentialDriveMode.ARCADE));

        controller.x().whileTrue(drivetrain.driveDistanceCommand(() -> 5));
        controller.y().whileTrue(drivetrain.rotateToAngle(() -> new Rotation2d(Math.PI)));
        controller.a().whileTrue(drivetrain.sysidDynamicForwardCommand());
        controller.b().whileTrue(drivetrain.sysidDynamicReverseCommand());
    }

    public void configureAuto() {
        AutoManager.getInstance().addRoutine(Autos.testPath(drivetrain));
    }
}
