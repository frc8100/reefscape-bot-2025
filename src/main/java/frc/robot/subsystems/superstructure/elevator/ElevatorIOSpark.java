package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Amps;
import static frc.lib.util.SparkUtil.ifOk;
import static frc.lib.util.SparkUtil.sparkStickyFault;
import static frc.lib.util.SparkUtil.tryUntilOk;

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

        SparkMaxConfig motorConfig = new MotorConfig().getConfig();

        // TODO: Apply PID config for the angle motor
        motorConfig.closedLoop
            .pidf(
                ElevatorConstants.ELEVATOR_KP,
                ElevatorConstants.ELEVATOR_KI,
                ElevatorConstants.ELEVATOR_KD,
                ElevatorConstants.ELEVATOR_KF
            )
            .outputRange(-ElevatorConstants.ELEVATOR_MAX_OUTPUT, ElevatorConstants.ELEVATOR_MAX_OUTPUT);

        // Apply the config
        tryUntilOk(motor, 5, () ->
            motor.configure(
                motorConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
        );

        // Reset the encoder
        tryUntilOk(motor, 5, () -> encoder.setPosition(0.0));
    }

    @Override
    public void runMotor(double motorInput) {
        // Apply deadband
        // motorInput = MathUtil.applyDeadband(motorInput, ClawConstants.ARM_CONTROLLER_DEADBAND);

        // Run the motor
        double percentOutput = ElevatorConstants.ELEVATOR_MAX_OUTPUT * motorInput;
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
    public void setPosition(Distance position) {
        // TODO
        // Set the position of the turn motor
        double setPointRadians = ElevatorConstants.getMotorPositionFromHeight(position);

        Logger.recordOutput("Elevator/SetPoint", setPointRadians);
        // angleClosedLoopController.setReference(rotation.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
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
