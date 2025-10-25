package org.firstinspires.ftc.teamcode;

public enum AprilTagId {
    BLUE_TAG(20), RED_TAG(24),
    GPP_TAG(21), PGP_TAG(22), PPG_TAG(23);

    private final int id;
    AprilTagId(int id) {
        this.id = id;
    }

    public int getId() {
        return id;
    }

    public static AprilTagId getTagFromId(int id) {
        for (AprilTagId tag : values()) {
            if (tag.getId() == id) {
                return tag;
            }
        }
        return null;
    }
}
