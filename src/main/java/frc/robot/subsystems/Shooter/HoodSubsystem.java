package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class HoodSubsystem extends SubsystemBase {

    // ------------------------------------------------
    // MOTOR
    // ------------------------------------------------

    private final SparkMax hoodMotor =
        new SparkMax(2, MotorType.kBrushless);

    // ------------------------------------------------
    // SMART MOTOR CONTROLLER (YAMS)
    // ------------------------------------------------

    private final SmartMotorController hoodSMC =
        new SparkWrapper(
            hoodMotor,
            DCMotor.getNeo550(1),
            new SmartMotorControllerConfig(this)
                // REQUIRED
                .withControlMode(ControlMode.CLOSED_LOOP)

                // PID (position based arm)
                .withClosedLoopController(
                    50.0, 0.0, 0.0,
                    DegreesPerSecond.of(180),
                    DegreesPerSecondPerSecond.of(360)
                )
                .withSimClosedLoopController(
                    50.0, 0.0, 0.0,
                    DegreesPerSecond.of(180),
                    DegreesPerSecondPerSecond.of(360)
                )

                // Feedforward
                .withFeedforward(new ArmFeedforward(0.0, 0.3, 0.02))
                .withSimFeedforward(new ArmFeedforward(0.0, 0.3, 0.02))

                // Gearing
                .withGearing(
                    new MechanismGearing(
                        GearBox.fromReductionStages(3, 4)
                    )
                )

                // SAFETY (NEO 550 REQUIRES THIS)
                .withStatorCurrentLimit(Amps.of(25))
                .withSupplyCurrentLimit(Amps.of(20))

                // Motor behavior
                .withIdleMode(MotorMode.BRAKE)
                .withMotorInverted(false)

                // Telemetry
                .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
        );

    // ------------------------------------------------
    // ARM MECHANISM
    // ------------------------------------------------

    private final Arm hood =
        new Arm(
            new ArmConfig(hoodSMC)
                .withStartingPosition(Degrees.of(20))
                .withSoftLimits(Degrees.of(5), Degrees.of(100))
                .withHardLimit(Degrees.of(0), Degrees.of(120))
                .withLength(Meters.of(0.30))
                .withMass(Kilograms.of(2.0))
                .withTelemetry("Hood", TelemetryVerbosity.HIGH)
        );

    // ------------------------------------------------
    // DISTANCE → ANGLE INTERPOLATION
    // ------------------------------------------------

    private final InterpolatingDoubleTreeMap hoodMap =
        new InterpolatingDoubleTreeMap();

    private Angle lastTarget = Degrees.of(20);
    private boolean zeroed = false;

    public HoodSubsystem() {
        hoodMap.put(1.5, 25.0);
        hoodMap.put(2.0, 30.0);
        hoodMap.put(2.5, 35.0);
        hoodMap.put(3.0, 40.0);
        hoodMap.put(3.5, 45.0);
    }

    // ------------------------------------------------
    // ZEROING
    // ------------------------------------------------

    private void autoZeroIfNeeded() {
        if (!zeroed && DriverStation.isEnabled()) {
            hoodSMC.setPosition(Degrees.zero());
            zeroed = true;
        }
    }

    // ------------------------------------------------
    // PUBLIC API
    // ------------------------------------------------

    public Angle getAngleForDistance(double meters) {
        return Degrees.of(hoodMap.get(meters));
    }

    public void setTargetAngle(Angle angle) {
        lastTarget = angle;
        hood.setAngle(angle);
    }

    public Angle getAngle() {
        return hood.getAngle();
    }

    // ------------------------------------------------
    // PERIODIC
    // ------------------------------------------------

    @Override
    public void periodic() {
        autoZeroIfNeeded();

        hood.updateTelemetry();

        Logger.recordOutput("Hood/TargetDeg", lastTarget.in(Degrees));
        Logger.recordOutput("Hood/ActualDeg", hood.getAngle().in(Degrees));
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
