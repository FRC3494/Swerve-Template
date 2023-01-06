package com.swervedrivespecialties.swervelib.ctre;

public class CanCoderAbsoluteConfiguration {
    private final int id;
    private final double offset;
    private final boolean reverse;

    public CanCoderAbsoluteConfiguration(int id, double offset, boolean reverse) {
        this.id = id;
        this.offset = offset;
        this.reverse = reverse;
    }

    public int getId() {
        return id;
    }

    public double getOffset() {
        return offset;
    }

    public boolean getReversed() {
        return reverse;
    }
}
