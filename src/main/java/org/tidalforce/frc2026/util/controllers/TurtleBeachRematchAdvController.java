package org.tidalforce.frc2026.util.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Custom wrapper for the Turtle Beach Rematch Advanced controller.
 * Treats back paddles as triggers.
 */
public class TurtleBeachRematchAdvController extends CommandXboxController {

    public TurtleBeachRematchAdvController(int port) {
        super(port);
    }

    /**
     * Trigger representing the left paddle.
     * Maps to the left stick press (L3) by default.
     */
    public Trigger LeftPaddle() {
        return leftStick(); // Use L3 as the paddle
    }

    /**
     * Trigger representing the right paddle.
     * Maps to the right stick press (R3) by default.
     */
    public Trigger RightPaddle() {
        return rightStick(); // Use R3 as the paddle
    }
}
