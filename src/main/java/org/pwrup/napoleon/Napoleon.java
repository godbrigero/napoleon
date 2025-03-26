package org.pwrup.napoleon;

import javax.swing.SwingUtilities;
import org.pwrup.napoleon.visualization.PathfinderVisualization;

public class Napoleon {

  public static void launchPathfinderVisualization() {
    SwingUtilities.invokeLater(() -> {
      new PathfinderVisualization().setVisible(true);
    });
  }

  public static void main(String[] args) {
    System.out.println("Launching A* Pathfinder Visualization...");
    launchPathfinderVisualization();
  }
}
