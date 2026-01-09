package frc.util.objective;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;

/**
 * Gets objective data from the dashboard.
 */
public class ObjectiveIODashboard implements ObjectiveIO {

    // Dashboard keys
    public static final String OBJECTIVE_DASHBOARD_KEY = "/ObjectiveTracker";
    public static final String DASHBOARD_PUBLISH_OBJECTIVE_KEY = "PublishObjective";

    private final NetworkTable objectiveTable;

    // Subscribers/publishers
    private final StringSubscriber publishObjectiveSubscriber;

    public ObjectiveIODashboard() {
        // Initialize tables
        objectiveTable = NetworkTableInstance.getDefault().getTable(OBJECTIVE_DASHBOARD_KEY);

        publishObjectiveSubscriber = objectiveTable.getStringTopic(DASHBOARD_PUBLISH_OBJECTIVE_KEY).subscribe("");
    }

    @Override
    public void updateInputs(ObjectiveIOInputs inputs) {
        if (publishObjectiveSubscriber.readQueue().length > 0) {
            inputs.sentObjective = publishObjectiveSubscriber.get();
        }
    }
}
