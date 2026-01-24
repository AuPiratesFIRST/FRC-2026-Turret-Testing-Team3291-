package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import yams.gearing.*;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.*;
import yams.motorcontrollers.remote.TalonFXSWrapper;

import edu.wpi.first.math.system.plant.DCMotor;

public class TurretSubsystem extends SubsystemBase {

    private static final Translation2d TURRET_OFFSET =
        new Translation2d(0.35, 0.0);

    private final VisionSubsystem vision;
    private final SwerveSubsystem swerve;
    private final ShooterSubsystem shooter;

    private final Pivot turretPivot;
    private final TurretVisualizer visualizer;

    // 🔥 SHOOT STATE
    private boolean shootingEnabled = false;

    public TurretSubsystem(
        VisionSubsystem vision,
        SwerveSubsystem swerve,
        ShooterSubsystem shooter
    ) {
        this.vision = vision;
        this.swerve = swerve;
        this.shooter = shooter;

        visualizer =
            new TurretVisualizer(
                () -> new Pose3d(swerve.getPose()),
                swerve::getFieldVelocity,
                () -> DriverStation.getAlliance()
                        .orElse(DriverStation.Alliance.Blue)
                        == DriverStation.Alliance.Blue
            );

        TalonFXS motor = new TalonFXS(1);

        SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                .withControlMode(
                    SmartMotorControllerConfig.ControlMode.CLOSED_LOOP
                )
                .withClosedLoopController(
                    1.8, 0, 0.25,
                    DegreesPerSecond.of(120),
                    DegreesPerSecondPerSecond.of(240)
                )
                .withIdleMode(
                    SmartMotorControllerConfig.MotorMode.BRAKE
                )
                .withStatorCurrentLimit(Amps.of(40))
                .withGearing(
                    new MechanismGearing(
                        GearBox.fromReductionStages(3, 4)
                    )
                )
                .withTelemetry(
                    "TurretMotor",
                    SmartMotorControllerConfig.TelemetryVerbosity.HIGH
                );

        TalonFXSWrapper smc =
            new TalonFXSWrapper(
                motor,
                DCMotor.getNEO(1),
                motorConfig
            );

        PivotConfig pivotConfig =
            new PivotConfig(smc)
                .withStartingPosition(Degrees.of(0))
                .withHardLimit(Degrees.of(0), Degrees.of(720))
                .withWrapping(Degrees.of(0), Degrees.of(360))
                .withMOI(Meters.of(0.25), Pounds.of(4));

        turretPivot = new Pivot(pivotConfig);
    }

    // ------------------------------------------------
    // PERIODIC
    // ------------------------------------------------

    @Override
    public void periodic() {
        updateControl();
        updateVisualization();
    }

    // ------------------------------------------------
    // CONTROL
    // ------------------------------------------------

    private void updateControl() {

        Translation3d hub =
            DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Blue
                    ? Constants.FieldConstants.HUB_BLUE
                    : Constants.FieldConstants.HUB_RED;

        Pose2d robotPose = swerve.getPose();
        Rotation2d robotYaw = robotPose.getRotation();

        Translation2d turretXY =
            robotPose.getTranslation()
                .plus(TURRET_OFFSET.rotateBy(robotYaw));

        Translation2d toHub =
            hub.toTranslation2d().minus(turretXY);

        Rotation2d desiredFieldHeading =
            new Rotation2d(
                Math.atan2(toHub.getY(), toHub.getX())
            );

        Rotation2d desiredTurretAngle =
            desiredFieldHeading.minus(robotYaw);

        turretPivot.setAngle(
            Degrees.of(desiredTurretAngle.getDegrees())
        );

        double distance =
            hub.toTranslation2d().getDistance(turretXY);

        // 🔥 SHOOT GATE
        if (shootingEnabled) {
            shooter.setAutoVelocityForDistance(distance);
        } else {
            shooter.setAutoVelocityForDistance(0.0);
        }

        turretPivot.updateTelemetry();
    }

    // ------------------------------------------------
    // VISUALIZATION
    // ------------------------------------------------

    private void updateVisualization() {
        visualizer.update(
            shooter.getLinearVelocity(),
            shooter.getHoodAngle()
        );
    }

    // ------------------------------------------------
    // SHOOT CONTROL API
    // ------------------------------------------------

    public void enableAutoShoot(boolean enabled) {
        shootingEnabled = enabled;
    }
}
