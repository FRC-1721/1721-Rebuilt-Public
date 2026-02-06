package org.tidalforce.frc2026.subsystems.navigation.planner;

public class Node {
    public final int x, y;
    public double g = Double.POSITIVE_INFINITY;
    public double rhs = Double.POSITIVE_INFINITY;

    public Node(int x, int y) {
        this.x = x;
        this.y = y;
    }
}
