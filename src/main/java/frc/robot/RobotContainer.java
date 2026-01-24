// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

// Utilities
import static edu.wpi.first.units.Units.*;


import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  // --- SUBSYSTEMS ---
 private final VisionSubsystem vision = new VisionSubsystem();
private final SwerveSubsystem drivebase =
    new SwerveSubsystem(
        new File(Filesystem.getDeployDirectory(), "swerve"),
        vision
    );

  private final ShooterSubsystem shooter = new ShooterSubsystem();
  
  // Vision subsystem for AprilTag detection

  private final TurretSubsystem turret =
    new TurretSubsystem(vision, drivebase, shooter);

  

             
  // --- CONTROLLER ---
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController turretController =
    new CommandXboxController(1);


  public RobotContainer() {
    configureBindings();

  }


  private void configureBindings() {

  

    // ---------------- DRIVE ----------------
    drivebase.setDefaultCommand(
        drivebase.driveCommand(
            () -> -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
            () -> -MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
            () -> -driverController.getRightX()
        )
    );

    // ---------------- RESET GYRO ----------------
    driverController.a().onTrue(
        Commands.runOnce(drivebase::zeroGyro)
    );

     // ---------------- AUTO SHOOT ----------------
    driverController.rightTrigger()
        .whileTrue(
            Commands.run(
                () -> turret.enableAutoShoot(true),
                turret
            )
        )
        .onFalse(
            Commands.runOnce(
                () -> turret.enableAutoShoot(false),
                turret
            )
        );
    
}

    public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public SwerveSubsystem getDrivebase() {
  return drivebase;
}

public VisionSubsystem getVision() {
  return vision;
}

}