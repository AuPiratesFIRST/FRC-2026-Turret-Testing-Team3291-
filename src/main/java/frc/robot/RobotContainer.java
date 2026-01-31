package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
// import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.commands.AimShooterFromVision;

import static edu.wpi.first.units.Units.*;

/**
 * RobotContainer
 * -----------------------------
 * Central wiring point for the robot.
 */
public class RobotContainer {

        // ---------------- SUBSYSTEMS ----------------

        private final VisionSubsystem vision = new VisionSubsystem();
        private final HoodSubsystem hood = new HoodSubsystem();
        private final ShooterSubsystem shooter = new ShooterSubsystem();

        // ---------------- CONTROLLERS ----------------

        private final CommandXboxController operator = new CommandXboxController(0);

        // ---------------- AUTO ----------------

        private final SendableChooser<Command> autoChooser = new SendableChooser<>();

        public RobotContainer() {

                configureBindings();

                // ---------------- DEFAULT COMMANDS ----------------
                hood.setDefaultCommand(hood.hold());
                shooter.setDefaultCommand(shooter.stop());

                // ---------------- PATHPLANNER NAMED COMMANDS ----------------
                NamedCommands.registerCommand(
                                "StopShooter",
                                shooter.stop());

                autoChooser.setDefaultOption("Do Nothing", Commands.none());
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        // --------------------------------------------------
        // CONTROLLER BINDINGS
        // --------------------------------------------------
        private void configureBindings() {

                // ================= HOOD =================
                operator.povUp().onTrue(
                                hood.setAngle(
                                                hood.getAngle().plus(Degrees.of(10))));

                operator.povDown().onTrue(
                                hood.setAngle(
                                                hood.getAngle().minus(Degrees.of(50))));

                // ================= SHOOTER =================
                operator.rightTrigger(0.2).whileTrue(
                                new AimShooterFromVision(shooter, hood, vision));
                // Manual shooter test (no vision)

                operator.a().whileTrue(
                                Commands.parallel(
                                                shooter.setRPM(3000),
                                                hood.setAngle(Degrees.of(35))));

        }

        // ================= AUTO =================
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        // ---------------- ACCESSORS ----------------
        public VisionSubsystem getVision() {
                return vision;
        }
}
