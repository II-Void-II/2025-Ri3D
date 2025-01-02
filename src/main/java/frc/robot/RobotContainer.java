// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.subsystems.BaseDifferentialDrive.DifferentialDriveMode;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.SendableLog;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    CommandXboxController controller = new CommandXboxController(0);

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
        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(() -> -controller.getLeftY(),
                () -> -controller.getRightY(), () -> -controller.getRightX(), () -> DifferentialDriveMode.ARCADE));

        controller.x().whileTrue(drivetrain.sysidQuasistaticForwardCommand());
        controller.y().whileTrue(drivetrain.sysidQuasistaticReverseCommand());
        controller.a().whileTrue(drivetrain.sysidDynamicForwardCommand());
        controller.b().whileTrue(drivetrain.sysidDynamicReverseCommand());
    }

    public void configureAuto() {
        AutoManager.getInstance().addRoutine(Autos.testPath(drivetrain));
    }
}
