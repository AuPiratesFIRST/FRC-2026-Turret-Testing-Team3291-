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

    private static final double MAX_RPM = 6000.0;
    private static final double WHEEL_RADIUS_METERS =
        Inches.of(2).in(Meters);

    private static final double SHOOTER_HEIGHT = 0.50;
    private static final double TARGET_HEIGHT  = 2.64;
    private static final double GRAVITY = 9.81;

    private final FlyWheel flywheel;
    private final HoodSubsystem hood;

    public ShooterSubsystem(HoodSubsystem hood) {
        this.hood = hood;

        SparkMax motor =
            new SparkMax(26, MotorType.kBrushless);

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
                .withStatorCurrentLimit(Amps.of(40))
                .withTelemetry(
                    "ShooterMotor",
                    SmartMotorControllerConfig.TelemetryVerbosity.LOW
                );

        SparkWrapper smc =
            new SparkWrapper(
                motor,
                DCMotor.getNEO(1),
                motorConfig
            );

        flywheel =
            new FlyWheel(
                new FlyWheelConfig(smc)
                    .withDiameter(Inches.of(4))
                    .withUpperSoftLimit(RPM.of(MAX_RPM))
            );
    }

    public LinearVelocity getRequiredVelocityForDistance(
        double distanceMeters
    ) {
        Angle hoodAngle =
            hood.getAngleForDistance(distanceMeters);

        double theta = hoodAngle.in(Radians);
        double h = TARGET_HEIGHT - SHOOTER_HEIGHT;
        double d = distanceMeters;

        double denom =
            2 * Math.pow(Math.cos(theta), 2)
            * (h - d * Math.tan(theta));

        if (denom >= 0) {
            return MetersPerSecond.of(0);
        }

        return MetersPerSecond.of(
            Math.sqrt(
                (GRAVITY * d * d) / (-denom)
            )
        );
    }

    public void setVelocityForDistance(double distanceMeters) {
        LinearVelocity v =
            getRequiredVelocityForDistance(distanceMeters);

        double rpm =
            (v.in(MetersPerSecond) / WHEEL_RADIUS_METERS)
            * 60.0 / (2 * Math.PI);

        flywheel.setSpeed(
            RPM.of(Math.min(rpm, MAX_RPM))
        );
    }

    public void stop() {
        flywheel.setSpeed(RPM.of(0));
    }

    // ---------------- BACKWARD COMPATIBILITY ----------------
    
    public void setAutoVelocityForDistance(double distanceMeters) {
        setVelocityForDistance(distanceMeters);
    }

    public LinearVelocity getLinearVelocity() {
        return MetersPerSecond.of(
            flywheel.getSpeed().in(RadiansPerSecond)
                * WHEEL_RADIUS_METERS
        );
    }

    public Angle getHoodAngle() {
        return hood.getAngle();
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
