package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import yams.gearing.*;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.*;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase {

    // ---------------- CONSTANTS ----------------
    private static final double MAX_RPM = 6000.0;
    private static final double WHEEL_RADIUS_METERS =
        Inches.of(2).in(Meters);

    // FIXED HOOD ANGLE (keep visualizer behavior)
    private static final Angle HOOD_ANGLE = Degrees.of(35);

    // HEIGHTS
    private static final double SHOOTER_HEIGHT = 0.50; // meters
    private static final double TARGET_HEIGHT  = 2.64; // meters (hub)

    private static final double GRAVITY = 9.81;

    private final FlyWheel flywheel;

    public ShooterSubsystem() {

        SparkMax motor = new SparkMax(26, MotorType.kBrushless);

        MechanismGearing gearing =
            new MechanismGearing(GearBox.fromReductionStages(1, 1));

        SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                .withControlMode(
                    SmartMotorControllerConfig.ControlMode.CLOSED_LOOP
                )
                .withClosedLoopController(
                    0.00015, 0, 0,
                    RPM.of(MAX_RPM),
                    RotationsPerSecondPerSecond.of(1200)
                )
                .withIdleMode(
                    SmartMotorControllerConfig.MotorMode.COAST
                )
                .withTelemetry(
                    "ShooterMotor",
                    SmartMotorControllerConfig.TelemetryVerbosity.LOW
                )
                .withStatorCurrentLimit(Amps.of(40))
                .withGearing(gearing);

        SparkWrapper smc =
            new SparkWrapper(
                motor,
                DCMotor.getNEO(1),
                motorConfig
            );

        FlyWheelConfig flywheelConfig =
            new FlyWheelConfig(smc)
                .withDiameter(Inches.of(4))
                .withMass(Pounds.of(1))
                .withUpperSoftLimit(RPM.of(MAX_RPM))
                .withTelemetry(
                    "Flywheel",
                    SmartMotorControllerConfig.TelemetryVerbosity.HIGH
                );

        flywheel = new FlyWheel(flywheelConfig);
    }

    // -------------------------------------------------
    // BALLISTIC SOLVER (FIXED ANGLE, SOLVE VELOCITY)
    // -------------------------------------------------

    public LinearVelocity getRequiredVelocityForDistance(
        double distanceMeters
    ) {
        double theta = HOOD_ANGLE.in(Radians);
        double h = TARGET_HEIGHT - SHOOTER_HEIGHT;
        double d = distanceMeters;

        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        double denom = 2 * cos * cos * (h - d * tan);

        // Safety clamp
        if (denom >= 0) {
            return MetersPerSecond.of(0);
        }

        double v =
            Math.sqrt(
                (GRAVITY * d * d) / (-denom)
            );

        return MetersPerSecond.of(v);
    }

    public AngularVelocity velocityToRPM(
        LinearVelocity velocity
    ) {
        double omega =
            velocity.in(MetersPerSecond) / WHEEL_RADIUS_METERS;

        return RadiansPerSecond.of(omega)
            .times(60.0 / (2 * Math.PI));
    }

    // -------------------------------------------------
    // DIRECT CONTROL (NO COMMAND SPAM)
    // -------------------------------------------------

    public void setAutoVelocityForDistance(double distanceMeters) {

        LinearVelocity v =
            getRequiredVelocityForDistance(distanceMeters);

        AngularVelocity rpm =
            velocityToRPM(v);

        flywheel.setSpeed(
            RPM.of(
                Math.min(rpm.in(RPM), MAX_RPM)
            )
        );
    }

    public LinearVelocity getLinearVelocity() {
        return MetersPerSecond.of(
            flywheel.getSpeed().in(RadiansPerSecond)
                * WHEEL_RADIUS_METERS
        );
    }

    public Angle getHoodAngle() {
        return HOOD_ANGLE;
    }

    @Override
    public void periodic() {
        flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }
}
