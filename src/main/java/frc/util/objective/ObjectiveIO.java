package frc.util.objective;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectiveIO {
    @AutoLog
    public static class ObjectiveIOInputs {

        /**
         * The objective that was sent to the dashboard.
         */
        // TODO: Way to change this string to an enum or object
        public String sentObjective = "";
    }

    public default void updateInputs(ObjectiveIOInputs inputs) {}

    public enum InputtedObjective {}
}
