package org.pwrup.napoleon;

import javax.swing.SwingUtilities;
import org.pwrup.napoleon.bridge.HybridGrid;
import org.pwrup.napoleon.visualization.PathfinderVisualization;

public class Napoleon {

  // Hardcoded default grid file path
  private static final String DEFAULT_GRID_PATH = "blue_a_annotations.json";

  public static void launchPathfinderVisualization() {
    SwingUtilities.invokeLater(() -> {
      try {
        // Load the HybridGrid from file
        HybridGrid grid = new HybridGrid(DEFAULT_GRID_PATH);

        // Create and display visualization with loaded grid
        new PathfinderVisualization(grid).setVisible(true);
      } catch (Exception e) {
        System.err.println(
          "Error loading grid configuration: " + e.getMessage()
        );
        e.printStackTrace();

        // Fallback to default visualization if file loading fails
        new PathfinderVisualization().setVisible(true);
      }
    });
  }

  public static void main(String[] args) {
    System.out.println("Creating sample grid and launching visualization...");
    launchPathfinderVisualization();
  }
}
