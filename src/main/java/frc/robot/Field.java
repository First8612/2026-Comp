package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Field {
    private static final Rotation2d noRotation = new Rotation2d();
    private static final Distance noDist = Inches.of(0);
    // referencing page 3 of
    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    private static final Distance fieldLength = Inches.of(651.22);
    private static final Distance fieldWidth = Inches.of(317.69);
    private static final Distance allianceZoneDepth = Inches.of(158.6);

    private static final Distance towerAlignRightY = Meters.of(3.52);
    private static final Distance towerAlignLeftY = Meters.of(3.90);
    private static final Distance towerAlignX = Meters.of(1.8);

    // using tag 28 from page 11
    private static final Distance trenchYOffset = Inches.of(25);
    private static final Distance trenchXOffset = Inches.of(180);
    

    private static final Pose2d blueOrigin = new Pose2d();
    private static final Pose2d redOrigin = new Pose2d(fieldLength, fieldWidth, new Rotation2d(Degree.of(180)));

    // public
    public static final ByAlliance blueAlliance = new ByAlliance("Blue", blueOrigin, new Rotation2d());
    public static final ByAlliance redAlliance = new ByAlliance("Red", redOrigin, new Rotation2d(Degree.of(180)));
    public static final ByAlliance getMyAlliance() {
        if(DriverStation.getAlliance().isPresent()) {
            return DriverStation.getAlliance().get() == Alliance.Blue
                ? blueAlliance
                : redAlliance;
        }

        return blueAlliance;
    }

    public static class ByAlliance {
        public ByAlliance(String name, Pose2d origin, Rotation2d perspectiveRotation) {
            super();

            var hubFromOrigin = new Translation2d(Inches.of(182.11), Inches.of(158.84));

            this.name = name;
            hub = origin.plus(new Transform2d(hubFromOrigin, perspectiveRotation));
            zone = new Rectangle2d(origin, allianceZoneDepth.times(2), fieldWidth)
                .transformBy(new Transform2d(
                    new Translation2d(allianceZoneDepth.times(0), fieldWidth.div(2)),
                    noRotation
                ));

            var passingTargetYOffset = Inches.of(72);
            var passingTargetXOffset = Inches.of(50);

            passingTargetRight = origin.plus(
                new Transform2d(passingTargetXOffset, passingTargetYOffset, noRotation)
            );

            passingTargetLeft = origin.plus(
                new Transform2d(passingTargetXOffset, fieldWidth.minus(passingTargetYOffset), noRotation)
            );

            // trenches
            this.trenchRight = new Trench(
                origin.transformBy(new Transform2d(trenchXOffset, trenchYOffset, noRotation))
            );
            this.trenchLeft = new Trench(
                origin.transformBy(new Transform2d(trenchXOffset, fieldWidth.minus(trenchYOffset), noRotation))
            );

            // tower align points
            this.towerAlignRight = origin.plus(
                new Transform2d(towerAlignX, towerAlignRightY, noRotation)
            );
            this.towerAlignLeft = origin.plus(
                new Transform2d(towerAlignX, towerAlignLeftY, noRotation)
            );
    }

        public final String name;
        public final Pose2d hub;
        public final Rectangle2d zone;
        public final Pose2d passingTargetRight;
        public final Pose2d passingTargetLeft;
        public final Trench trenchRight;
        public final Trench trenchLeft;
        public final Pose2d towerAlignRight;
        public final Pose2d towerAlignLeft;
    }

    public static class Trench {
        public Trench(Pose2d location) {
            var trenchZoneWidth = Inches.of(72);

            this.location = location;
            // entrace is the side closest to alliance origin
            this.entrance = location.transformBy(new Transform2d(trenchZoneWidth.div(2).times(-1), noDist, noRotation));
            this.exit = location.transformBy(new Transform2d(trenchZoneWidth.div(2), noDist, noRotation));

            this.zone = new Rectangle2d(
                location,
                trenchZoneWidth,
                trenchYOffset.times(2)
            );
        }

        public Pose2d location;
        public Pose2d entrance;
        public Pose2d exit;
        public Rectangle2d zone;
    }

    public static boolean inTrenchZone(Pose2d pose) {
        if (blueAlliance.trenchRight.zone.contains(pose.getTranslation())) return true;
        if (blueAlliance.trenchLeft.zone.contains(pose.getTranslation())) return true;
        if (redAlliance.trenchRight.zone.contains(pose.getTranslation())) return true;
        if (redAlliance.trenchLeft.zone.contains(pose.getTranslation())) return true;

        return false;
    }

    public static void writeOnceToNT() {
        var nt = NetworkTableInstance.getDefault();
        writeAllianceOnceToNT(nt, "blue", Field.blueAlliance);
        writeAllianceOnceToNT(nt, "red", Field.redAlliance);
    }

    private static void writeAllianceOnceToNT(NetworkTableInstance nt, String name, ByAlliance alliance) {
        nt.getStructTopic("Field/" + name + "/zone", Rectangle2d.struct).publish().set(alliance.zone);
        nt.getStructTopic("Field/" + name + "/hub", Pose2d.struct).publish().set(alliance.hub);
        nt.getStructTopic("Field/" + name + "/passingTargetRight", Pose2d.struct).publish().set(alliance.passingTargetRight);
        nt.getStructTopic("Field/" + name + "/passingTargetLeft", Pose2d.struct).publish().set(alliance.passingTargetLeft);
        nt.getStructTopic("Field/" + name + "/trenchRight/location", Pose2d.struct).publish().set(alliance.trenchRight.location);
        nt.getStructTopic("Field/" + name + "/trenchRight/entrance", Pose2d.struct).publish().set(alliance.trenchRight.entrance);
        nt.getStructTopic("Field/" + name + "/trenchRight/exit", Pose2d.struct).publish().set(alliance.trenchRight.exit);
        nt.getStructTopic("Field/" + name + "/trenchRight/zone", Rectangle2d.struct).publish().set(alliance.trenchRight.zone);
        nt.getStructTopic("Field/" + name + "/trenchLeft/location", Pose2d.struct).publish().set(alliance.trenchLeft.location);
        nt.getStructTopic("Field/" + name + "/trenchLeft/entrance", Pose2d.struct).publish().set(alliance.trenchLeft.entrance);
        nt.getStructTopic("Field/" + name + "/trenchLeft/exit", Pose2d.struct).publish().set(alliance.trenchLeft.exit);
        nt.getStructTopic("Field/" + name + "/trenchLeft/zone", Rectangle2d.struct).publish().set(alliance.trenchLeft.zone);
        nt.getStructTopic("Field/" + name + "/tower/towerAlignRight", Pose2d.struct).publish().set(alliance.towerAlignRight);
        nt.getStructTopic("Field/" + name + "/tower/towerAlignLeft", Pose2d.struct).publish().set(alliance.towerAlignLeft);
    }
    
}
