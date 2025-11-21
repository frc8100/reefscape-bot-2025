package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.util.CoupledYAMSSubsystemIO;
import org.littletonrobotics.junction.AutoLog;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class TestArmSubsystemIO implements CoupledYAMSSubsystemIO {

    @AutoLog
    public static class TestArmSubsystemIOInputs {

        public CoupledYAMSSubsystemIO.SparkMotorControllerData motorData = CoupledYAMSSubsystemIO.defaultControllerData;
    }

    private final SparkMax spark = new SparkMax(4, MotorType.kBrushless);
    private final SparkWrapper sparkSmartMotorController;

    public TestArmSubsystemIO(SmartMotorControllerConfig motorConfig) {
        sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), motorConfig);
    }

    @Override
    public SmartMotorController getMotorController() {
        return sparkSmartMotorController;
    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(TestArmSubsystemIOInputs inputs) {
        inputs.motorData = CoupledYAMSSubsystemIO.getDataFromMotorController(sparkSmartMotorController);
    }
}
