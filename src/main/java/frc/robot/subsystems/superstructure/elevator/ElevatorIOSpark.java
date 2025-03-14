package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.lib.util.SparkUtil.ifOk;
import static frc.lib.util.SparkUtil.sparkStickyFault;
import static frc.lib.util.SparkUtil.tryUntilOk;

// import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.GenericSparkIO.GenericSparkIOConfig;
import frc.lib.util.TunableValue;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.claw.ClawConstants;
import frc.robot.subsystems.superstructure.claw.ClawIO.ClawIOInputs;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSpark implements ElevatorIO {

    // Turn motor variables
    /**
     * The motor.
     */
    private final SparkMax motor;

    /**
     * The relative encoder.
     */
    private final RelativeEncoder encoder;

    /**
     * The closed loop controller.
     */
    private final SparkClosedLoopController closedLoopController;

    /**
     * A debouncer for the connected state, to prevent flickering.
     */
    private final Debouncer connectedDebouncer = new Debouncer(0.5);

    /**
     * The setpoint of the elevator in radians.
     */
    private double radianSetpoint = 0.0;

    /**
     * The configuration for the elevator motor.
     */
    private SparkMaxConfig config;

    /**
     * The config for the outtake motor.
     */
    private static class MotorConfig extends GenericSparkIOConfig {

        public MotorConfig() {
            super();
            // Override the default config
            this.idleMode = SparkBaseConfig.IdleMode.kBrake;
            this.inverted = ElevatorConstants.ELEVATOR_MOTOR_INVERTED;
            this.smartCurrentLimit = (int) ElevatorConstants.ELEVATOR_MOTOR_CURRENT_LIMIT.in(Amps);
            this.positionConversionFactor = ElevatorConstants.ELEVATOR_MOTOR_POSITION_FACTOR;
        }
    }

    public ElevatorIOSpark() {
        // Create the motor and configure it
        motor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        closedLoopController = motor.getClosedLoopController();

        config = new MotorConfig().getConfig();

        config.closedLoop
            .pidf(
                ElevatorConstants.ELEVATOR_KP.get(),
                ElevatorConstants.ELEVATOR_KI.get(),
                ElevatorConstants.ELEVATOR_KD.get(),
                ElevatorConstants.ELEVATOR_KF.get()
            )
            .outputRange(-ElevatorConstants.ELEVATOR_MAX_OUTPUT, ElevatorConstants.ELEVATOR_MAX_OUTPUT);

        config.closedLoop.maxMotion
            .maxVelocity(ElevatorConstants.ELEVATOR_MAX_ANGULAR_VELOCITY.in(RadiansPerSecond))
            .maxAcceleration(ElevatorConstants.ELEVATOR_MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));

        // Apply the config
        tryUntilOk(motor, 5, () ->
            motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        );

        // Reset the encoder
        tryUntilOk(motor, 5, () -> encoder.setPosition(0.0));

        TunableValue.addRefreshConfigConsumer(this::refreshConfig);
    }

    @Override
    public void refreshConfig() {
        // Refresh the config for the motor
        config.closedLoop.pidf(
            ElevatorConstants.ELEVATOR_KP.get(),
            ElevatorConstants.ELEVATOR_KI.get(),
            ElevatorConstants.ELEVATOR_KD.get(),
            ElevatorConstants.ELEVATOR_KF.get()
        );
        // .outputRange(-ElevatorConstants.ELEVATOR_MAX_OUTPUT, ElevatorConstants.ELEVATOR_MAX_OUTPUT);

        tryUntilOk(motor, 5, () ->
            motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        );
    }

    @Override
    public void runMotor(double motorInput) {
        double positionCurrent = encoder.getPosition();

        // If the position is above the max, stop

        double percentOutput = motorInput;

        // TODO:
        // If it is at the top, slow
        if (positionCurrent > ElevatorConstants.ELEVATOR_MAX_POSITION.in(Radians)) {
            percentOutput *= 0.05;
        } else if (positionCurrent > ElevatorConstants.ELEVATOR_TOP_THRESHOLD.in(Radians)) {
            percentOutput *= ElevatorConstants.ELEVATOR_TOP_INPUT;
        } else {
            percentOutput *= ElevatorConstants.ELEVATOR_MAX_OUTPUT;
        }

        motor.set(percentOutput);

        // Log
        Logger.recordOutput("Elevator/motorInput", motorInput);
        Logger.recordOutput("Elevator/percentOutput", percentOutput);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void zeroEncoder(double value) {
        encoder.setPosition(value);
        radianSetpoint = 0.0;
    }

    @Override
    public void setPosition(Distance position) {
        // Set the position of the turn motor
        radianSetpoint = ElevatorConstants.getMotorPositionFromHeight(position);
    }

    @Override
    public void setPosition(SuperstructureConstants.Level level) {
        radianSetpoint = level.getElevatorRadian();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Clamp
        radianSetpoint = MathUtil.clamp(
            radianSetpoint,
            ElevatorConstants.ELEVATOR_MIN_POSITION.in(Radians),
            ElevatorConstants.ELEVATOR_MAX_POSITION.in(Radians)
        );

        inputs.setpoint = radianSetpoint;

        // TODO: PID
        // closedLoopController.setReference(radianSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        // Reset spark sticky fault
        sparkStickyFault = false;

        // Update inputs for the turn motor

        // Set the position and velocity
        ifOk(motor, encoder::getPosition, position -> inputs.positionRad = position);
        ifOk(motor, encoder::getVelocity, velocity -> inputs.velocityRadPerSec = velocity);
        ifOk(motor, encoder::getPosition, position ->
            inputs.height = position * ElevatorConstants.ELEVATOR_RADIANS_TO_METERS
        );

        // Set the supply current based on the bus voltage multiplied by the applied output
        ifOk(motor, new DoubleSupplier[] { motor::getBusVoltage, motor::getAppliedOutput }, x ->
            inputs.appliedVolts = x[0] * x[1]
        );

        // Set the torque current
        ifOk(motor, motor::getOutputCurrent, current -> inputs.torqueCurrentAmps = current);

        // Set the connected state with a debouncer
        inputs.connected = connectedDebouncer.calculate(!sparkStickyFault);

        // Set the temperature
        ifOk(motor, motor::getMotorTemperature, temp -> inputs.tempCelsius = temp);
    }
}
