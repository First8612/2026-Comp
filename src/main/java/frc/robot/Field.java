package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;

public class Field {
    private static final Rotation2d noRotation = new Rotation2d();
    private static final Distance noDist = Inches.of(0);
    // referencing page 3 of
    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    private static final Distance fieldLength = Inches.of(651.22);
    private static final Distance fieldWidth = Inches.of(317.69);
    private static final Distance allianceZoneDepth = Inches.of(158.6);

    // using tag 28 from page 11
    private static final Distance trenchYOffset = Inches.of(25);
    private static final Distance trenchXOffset = Inches.of(180);
    

    private static final Pose2d blueOrigin = new Pose2d();
    private static final Pose2d redOrigin = new Pose2d(fieldLength, fieldWidth, new Rotation2d(Degree.of(180)));

    // public
    public static final ByAlliance blueAlliance = new ByAlliance("Blue", blueOrigin, new Rotation2d());
    public static final ByAlliance redAlliance = new ByAlliance("Red", redOrigin, new Rotation2d(Degree.of(180)));

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

            var passingTargetYOffset = Inches.of(24);
            var passingTargetXOffset = Inches.of(24);

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
    }

        public final String name;
        public final Pose2d hub;
        public final Rectangle2d zone;
        public final Pose2d passingTargetRight;
        public final Pose2d passingTargetLeft;
        public final Trench trenchRight;
        public final Trench trenchLeft;
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
        nt.getStructTopic("Field/blue/hub", Pose2d.struct).publish().set(blueAlliance.hub);
        nt.getStructTopic("Field/blue/zone", Rectangle2d.struct).publish().set(blueAlliance.zone);
        nt.getStructTopic("Field/blue/passingTargetRight", Pose2d.struct).publish().set(blueAlliance.passingTargetRight);
        nt.getStructTopic("Field/blue/passingTargetLeft", Pose2d.struct).publish().set(blueAlliance.passingTargetLeft);
        nt.getStructTopic("Field/blue/trenchRight/location", Pose2d.struct).publish().set(blueAlliance.trenchRight.location);
        nt.getStructTopic("Field/blue/trenchRight/entrance", Pose2d.struct).publish().set(blueAlliance.trenchRight.entrance);
        nt.getStructTopic("Field/blue/trenchRight/exit", Pose2d.struct).publish().set(blueAlliance.trenchRight.exit);
        nt.getStructTopic("Field/blue/trenchRight/zone", Rectangle2d.struct).publish().set(blueAlliance.trenchRight.zone);
        nt.getStructTopic("Field/blue/trenchLeft/location", Pose2d.struct).publish().set(blueAlliance.trenchLeft.location);
        nt.getStructTopic("Field/blue/trenchLeft/entrance", Pose2d.struct).publish().set(blueAlliance.trenchLeft.entrance);
        nt.getStructTopic("Field/blue/trenchLeft/exit", Pose2d.struct).publish().set(blueAlliance.trenchLeft.exit);
        nt.getStructTopic("Field/blue/trenchLeft/zone", Rectangle2d.struct).publish().set(blueAlliance.trenchLeft.zone);
        nt.getStructTopic("Field/red/zone", Rectangle2d.struct).publish().set(redAlliance.zone);
        nt.getStructTopic("Field/red/hub", Pose2d.struct).publish().set(redAlliance.hub);
        nt.getStructTopic("Field/red/passingTargetRight", Pose2d.struct).publish().set(redAlliance.passingTargetRight);
        nt.getStructTopic("Field/red/passingTargetLeft", Pose2d.struct).publish().set(redAlliance.passingTargetLeft);
        nt.getStructTopic("Field/red/trenchRight/location", Pose2d.struct).publish().set(redAlliance.trenchRight.location);
        nt.getStructTopic("Field/red/trenchRight/entrance", Pose2d.struct).publish().set(redAlliance.trenchRight.entrance);
        nt.getStructTopic("Field/red/trenchRight/exit", Pose2d.struct).publish().set(redAlliance.trenchRight.exit);
        nt.getStructTopic("Field/red/trenchRight/zone", Rectangle2d.struct).publish().set(redAlliance.trenchRight.zone);
        nt.getStructTopic("Field/red/trenchLeft/location", Pose2d.struct).publish().set(redAlliance.trenchLeft.location);
        nt.getStructTopic("Field/red/trenchLeft/entrance", Pose2d.struct).publish().set(redAlliance.trenchLeft.entrance);
        nt.getStructTopic("Field/red/trenchLeft/exit", Pose2d.struct).publish().set(redAlliance.trenchLeft.exit);
        nt.getStructTopic("Field/red/trenchLeft/zone", Rectangle2d.struct).publish().set(redAlliance.trenchLeft.zone);
    }
}
