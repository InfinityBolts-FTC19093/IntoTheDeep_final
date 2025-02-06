package org.firstinspires.ftc.teamcode.utils.ToutchPad;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;

public class PurePursuitController {
    private final MecanumDrive drive;
    private final List<Waypoint> path;
    private final double lookaheadDistance;
    private int currentWaypoint = 0;

    public PurePursuitController(MecanumDrive drive, List<Waypoint> path, double lookaheadDistance) {
        this.drive = drive;
        this.path = path;
        this.lookaheadDistance = lookaheadDistance;
    }

    public void update() {
        if (path == null || path.isEmpty()) {
            RobotLog.e("ERROR: No waypoints in path!");
            return;
        }

        // Get current robot position
        Pose2d robotPose = drive.getPoseEstimate();
        Waypoint targetWaypoint = findLookaheadPoint(robotPose);

        if (targetWaypoint == null) {
            RobotLog.w("No valid lookahead point found!");
            return;
        }

        // Calculate direction and speed
        double dx = targetWaypoint.x - robotPose.position.x;
        double dy = targetWaypoint.y - robotPose.position.y;
        double distance = Math.hypot(dx, dy);

        double targetAngle = Math.atan2(dy, dx);
        double headingError = targetAngle - robotPose.heading.toDouble();

        // Set drive motor power
        double speed = Math.min(distance * 0.5, 0.8);  // Scale speed
        drive.setDrivePowers(speed, headingError);
    }

    private Waypoint findLookaheadPoint(Pose2d robotPose) {
        for (int i = currentWaypoint; i < path.size(); i++) {
            Waypoint waypoint = path.get(i);
            double distance = Math.hypot(waypoint.x - robotPose.position.x, waypoint.y - robotPose.position.y);

            if (distance >= lookaheadDistance) {
                currentWaypoint = i;
                return waypoint;
            }
        }
        return null;
    }
}
