package org.pwrup.napoleon;

import org.pwrup.napoleon.bridge.AStarPathfinder;

public class Main {
  public static void main(String[] args) {
    System.out.println("Hello from Java!");

    // Example: Call a native method (if using Rust)
    AStarPathfinder pathfinder = new AStarPathfinder();
    int[] result = pathfinder.calculate(
        new int[] { 1, 2, 3, 4 }, // Static obstacles
        new int[] { 10, 10 }, // Grid size
        new int[] { 5, 5 }, // Center
        1.0f, // Square size in meters
        new float[] { 1.0f, 1.0f }, // Robot velocity
        new int[] { 0, 0 }, // Start position
        new int[] { 9, 9 }, // End position
        new float[] { 2.0f, 2.0f } // Finder relative size
    );

    System.out.println("Pathfinding result: " + java.util.Arrays.toString(result));
  }
}
