package org.pwrup.napoleon.visualization;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.SwingUtilities;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import org.pwrup.napoleon.bridge.AStarPathfinder;
import org.pwrup.napoleon.bridge.HybridGrid;
import org.pwrup.napoleon.bridge.NodePickStyle;

public class PathfinderVisualization extends JFrame {

  private GridPanel gridPanel;
  private AStarPathfinder pathfinder;
  private HybridGrid hybridGrid;

  private int[] startPoint = new int[] { 10, 10 };
  private int[] endPoint = new int[] { 40, 40 };
  private int[] path;

  private float finderRelativeW = 1.0f;
  private float finderRelativeH = 1.0f;
  private boolean doAbsoluteDiscard = false;
  private float avgDistanceMinDiscardThreshold = 0.0f;
  private float avgDistanceCost = 0.0f;
  private NodePickStyle nodePickStyle = NodePickStyle.ALL;

  private List<int[]> obstacles = new ArrayList<>();
  private List<float[]> hybridObstacles = new ArrayList<>();
  private List<UncertaintyField> uncertaintyFields = new ArrayList<>();
  private long lastCalculationTime = 0;
  private JLabel timeLabel;

  // Current interaction mode
  private InteractionMode currentMode = InteractionMode.OBSTACLE;

  // Enum for different mouse interaction modes
  private enum InteractionMode {
    OBSTACLE,
    START_END,
    HYBRID_OBSTACLE,
    UNCERTAINTY_FIELD,
  }

  // Class to store uncertainty field data
  private static class UncertaintyField {

    float[] center; // x, y
    float radius;
    float intensity;

    public UncertaintyField(float[] center, float radius, float intensity) {
      this.center = center;
      this.radius = radius;
      this.intensity = intensity;
    }
  }

  /**
   * Constructor that accepts a pre-configured HybridGrid
   *
   * @param grid The HybridGrid configuration to use
   */
  public PathfinderVisualization(HybridGrid grid) {
    setTitle("A* Pathfinder Visualization - Loaded from File");
    setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    setSize(1000, 700);

    this.hybridGrid = grid;

    // Extract obstacles from the grid
    int[] staticObstacles = grid.getStaticObstacles();
    obstacles.clear();
    for (int i = 0; i < staticObstacles.length; i += 2) {
      if (i + 1 < staticObstacles.length) {
        obstacles.add(new int[] { staticObstacles[i], staticObstacles[i + 1] });
      }
    }

    initComponents();
    createPathfinderWithGrid();
    calculatePath();
  }

  public PathfinderVisualization() {
    setTitle("A* Pathfinder Visualization");
    setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    setSize(1000, 700);

    initComponents();
    createPathfinder();
    calculatePath();
  }

  private void initComponents() {
    JPanel mainPanel = new JPanel(new BorderLayout());

    gridPanel = new GridPanel();
    mainPanel.add(gridPanel, BorderLayout.CENTER);

    JPanel controlPanel = createControlPanel();
    mainPanel.add(controlPanel, BorderLayout.EAST);

    add(mainPanel);
  }

  private JPanel createControlPanel() {
    JPanel controlPanel = new JPanel();
    controlPanel.setLayout(new BoxLayout(controlPanel, BoxLayout.Y_AXIS));
    controlPanel.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
    controlPanel.setPreferredSize(new Dimension(300, 600));

    JLabel titleLabel = new JLabel("Pathfinder Controls");
    titleLabel.setFont(new Font("Arial", Font.BOLD, 16));
    titleLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
    controlPanel.add(titleLabel);
    controlPanel.add(Box.createVerticalStrut(20));

    timeLabel = new JLabel("Calculation time: 0 ms");
    timeLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
    timeLabel.setFont(new Font("Arial", Font.BOLD, 12));
    controlPanel.add(timeLabel);
    controlPanel.add(Box.createVerticalStrut(10));

    JLabel nodePickStyleLabel = new JLabel("Node Pick Style:");
    nodePickStyleLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
    controlPanel.add(nodePickStyleLabel);

    JComboBox<String> nodePickStyleCombo = new JComboBox<>(
      new String[] { "ALL", "SIDES" }
    );
    nodePickStyleCombo.setMaximumSize(new Dimension(300, 30));
    nodePickStyleCombo.setAlignmentX(Component.LEFT_ALIGNMENT);
    nodePickStyleCombo.addActionListener(e -> {
      nodePickStyle =
        nodePickStyleCombo.getSelectedIndex() == 0
          ? NodePickStyle.ALL
          : NodePickStyle.SIDES;
      createPathfinder();
      calculatePath();
      gridPanel.repaint();
    });
    controlPanel.add(nodePickStyleCombo);
    controlPanel.add(Box.createVerticalStrut(10));

    controlPanel.add(
      createSliderPanel(
        "Finder Relative Width",
        0,
        20,
        10,
        value -> {
          finderRelativeW = value / 10.0f;
          createPathfinder();
          calculatePath();
          gridPanel.repaint();
        }
      )
    );

    controlPanel.add(
      createSliderPanel(
        "Finder Relative Height",
        0,
        20,
        10,
        value -> {
          finderRelativeH = value / 10.0f;
          createPathfinder();
          calculatePath();
          gridPanel.repaint();
        }
      )
    );

    JCheckBox discardCheckbox = new JCheckBox("Do Absolute Discard");
    discardCheckbox.setAlignmentX(Component.LEFT_ALIGNMENT);
    discardCheckbox.addActionListener(e -> {
      doAbsoluteDiscard = discardCheckbox.isSelected();
      createPathfinder();
      calculatePath();
      gridPanel.repaint();
    });
    controlPanel.add(discardCheckbox);
    controlPanel.add(Box.createVerticalStrut(10));

    controlPanel.add(
      createSliderPanel(
        "Min Discard Threshold",
        0,
        100,
        0,
        value -> {
          avgDistanceMinDiscardThreshold = value / 10.0f;
          createPathfinder();
          calculatePath();
          gridPanel.repaint();
        }
      )
    );

    controlPanel.add(
      createSliderPanel(
        "Avg Distance Cost",
        0,
        100,
        0,
        value -> {
          avgDistanceCost = value / 10.0f;
          createPathfinder();
          calculatePath();
          gridPanel.repaint();
        }
      )
    );

    // Interaction mode selection
    JLabel modeLabel = new JLabel("Interaction Mode:");
    modeLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
    controlPanel.add(modeLabel);

    JComboBox<String> modeCombo = new JComboBox<>(
      new String[] {
        "Static Obstacles",
        "Start/End Points",
        "Hybrid Obstacles",
        "Uncertainty Fields",
      }
    );
    modeCombo.setMaximumSize(new Dimension(300, 30));
    modeCombo.setAlignmentX(Component.LEFT_ALIGNMENT);
    modeCombo.addActionListener(e -> {
      switch (modeCombo.getSelectedIndex()) {
        case 0:
          currentMode = InteractionMode.OBSTACLE;
          break;
        case 1:
          currentMode = InteractionMode.START_END;
          break;
        case 2:
          currentMode = InteractionMode.HYBRID_OBSTACLE;
          break;
        case 3:
          currentMode = InteractionMode.UNCERTAINTY_FIELD;
          break;
      }
    });
    controlPanel.add(modeCombo);
    controlPanel.add(Box.createVerticalStrut(15));

    // Hybrid obstacle controls
    JLabel hybridObstacleLabel = new JLabel("Hybrid Obstacles:");
    hybridObstacleLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
    hybridObstacleLabel.setFont(new Font("Arial", Font.BOLD, 12));
    controlPanel.add(hybridObstacleLabel);

    JButton addHybridObstacleButton = new JButton("Add Hybrid Obstacle");
    addHybridObstacleButton.setAlignmentX(Component.LEFT_ALIGNMENT);
    addHybridObstacleButton.addActionListener(e -> {
      String input = JOptionPane.showInputDialog(
        this,
        "Enter hybrid obstacle data (x,y,width,height):",
        "Add Hybrid Obstacle",
        JOptionPane.PLAIN_MESSAGE
      );

      if (input != null && !input.trim().isEmpty()) {
        try {
          String[] parts = input.split(",");
          if (parts.length == 4) {
            float[] obstacle = new float[4];
            for (int i = 0; i < 4; i++) {
              obstacle[i] = Float.parseFloat(parts[i].trim());
            }
            hybridObstacles.add(obstacle);
            updatePathfinderWithDynamicObjects();
            calculatePath();
            gridPanel.repaint();
          }
        } catch (NumberFormatException ex) {
          JOptionPane.showMessageDialog(
            this,
            "Invalid input format. Please use x,y,width,height format with numeric values.",
            "Error",
            JOptionPane.ERROR_MESSAGE
          );
        }
      }
    });
    controlPanel.add(addHybridObstacleButton);

    JButton clearHybridObstaclesButton = new JButton("Clear Hybrid Obstacles");
    clearHybridObstaclesButton.setAlignmentX(Component.LEFT_ALIGNMENT);
    clearHybridObstaclesButton.addActionListener(e -> {
      hybridObstacles.clear();
      updatePathfinderWithDynamicObjects();
      calculatePath();
      gridPanel.repaint();
    });
    controlPanel.add(clearHybridObstaclesButton);
    controlPanel.add(Box.createVerticalStrut(15));

    // Uncertainty field controls
    JLabel uncertaintyFieldLabel = new JLabel("Uncertainty Fields:");
    uncertaintyFieldLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
    uncertaintyFieldLabel.setFont(new Font("Arial", Font.BOLD, 12));
    controlPanel.add(uncertaintyFieldLabel);

    JButton addUncertaintyFieldButton = new JButton("Add Uncertainty Field");
    addUncertaintyFieldButton.setAlignmentX(Component.LEFT_ALIGNMENT);
    addUncertaintyFieldButton.addActionListener(e -> {
      String input = JOptionPane.showInputDialog(
        this,
        "Enter uncertainty field data (centerX,centerY,radius,intensity):",
        "Add Uncertainty Field",
        JOptionPane.PLAIN_MESSAGE
      );

      if (input != null && !input.trim().isEmpty()) {
        try {
          String[] parts = input.split(",");
          if (parts.length == 4) {
            float[] center = new float[2];
            center[0] = Float.parseFloat(parts[0].trim());
            center[1] = Float.parseFloat(parts[1].trim());
            float radius = Float.parseFloat(parts[2].trim());
            float intensity = Float.parseFloat(parts[3].trim());

            uncertaintyFields.add(
              new UncertaintyField(center, radius, intensity)
            );
            updatePathfinderWithDynamicObjects();
            calculatePath();
            gridPanel.repaint();
          }
        } catch (NumberFormatException ex) {
          JOptionPane.showMessageDialog(
            this,
            "Invalid input format. Please use centerX,centerY,radius,intensity format with numeric values.",
            "Error",
            JOptionPane.ERROR_MESSAGE
          );
        }
      }
    });
    controlPanel.add(addUncertaintyFieldButton);

    JButton clearUncertaintyFieldsButton = new JButton(
      "Clear Uncertainty Fields"
    );
    clearUncertaintyFieldsButton.setAlignmentX(Component.LEFT_ALIGNMENT);
    clearUncertaintyFieldsButton.addActionListener(e -> {
      uncertaintyFields.clear();
      updatePathfinderWithDynamicObjects();
      calculatePath();
      gridPanel.repaint();
    });
    controlPanel.add(clearUncertaintyFieldsButton);
    controlPanel.add(Box.createVerticalStrut(15));

    JButton clearObstaclesButton = new JButton("Clear Static Obstacles");
    clearObstaclesButton.setAlignmentX(Component.LEFT_ALIGNMENT);
    clearObstaclesButton.addActionListener(e -> {
      obstacles.clear();
      createPathfinder();
      calculatePath();
      gridPanel.repaint();
    });
    controlPanel.add(clearObstaclesButton);
    controlPanel.add(Box.createVerticalStrut(10));

    JButton recalculateButton = new JButton("Recalculate Path");
    recalculateButton.setAlignmentX(Component.LEFT_ALIGNMENT);
    recalculateButton.addActionListener(e -> {
      calculatePath();
      gridPanel.repaint();
    });
    controlPanel.add(recalculateButton);
    controlPanel.add(Box.createVerticalStrut(10));

    JLabel instructionsLabel = new JLabel(
      "<html>Select interaction mode above.<br>" +
      "In Static Obstacles mode: Left click to place/remove obstacles<br>" +
      "In Start/End mode: Right click to set start, Middle click to set end<br>" +
      "In Hybrid/Uncertainty modes: Use buttons to add objects</html>"
    );
    instructionsLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
    controlPanel.add(instructionsLabel);

    return controlPanel;
  }

  private JPanel createSliderPanel(
    String labelText,
    int min,
    int max,
    int initial,
    SliderCallback callback
  ) {
    JPanel panel = new JPanel();
    panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
    panel.setAlignmentX(Component.LEFT_ALIGNMENT);
    panel.setMaximumSize(new Dimension(300, 60));

    JLabel label = new JLabel(labelText);
    label.setAlignmentX(Component.LEFT_ALIGNMENT);
    panel.add(label);

    JSlider slider = new JSlider(min, max, initial);
    slider.setMajorTickSpacing((max - min) / 5);
    slider.setPaintTicks(true);
    slider.setPaintLabels(true);
    slider.setAlignmentX(Component.LEFT_ALIGNMENT);

    JLabel valueLabel = new JLabel(String.valueOf(initial / 10.0f));
    valueLabel.setAlignmentX(Component.LEFT_ALIGNMENT);

    slider.addChangeListener(
      new ChangeListener() {
        @Override
        public void stateChanged(ChangeEvent e) {
          int value = slider.getValue();
          valueLabel.setText(String.valueOf(value / 10.0f));
          callback.onValueChanged(value);
        }
      }
    );

    panel.add(slider);
    panel.add(valueLabel);
    panel.add(Box.createVerticalStrut(10));

    return panel;
  }

  private void createPathfinder() {
    int gridSize = 50;
    int[][] staticObstacles = new int[obstacles.size()][];
    for (int i = 0; i < obstacles.size(); i++) {
      staticObstacles[i] = obstacles.get(i);
    }

    if (hybridGrid != null) {
      hybridGrid = null;
    }

    hybridGrid =
      new HybridGrid(
        gridSize,
        gridSize,
        gridSize / 2,
        gridSize / 2,
        1.0f,
        staticObstacles
      );

    if (pathfinder != null) {
      pathfinder.close();
      pathfinder = null;
    }

    pathfinder =
      new AStarPathfinder(
        hybridGrid,
        nodePickStyle,
        finderRelativeW,
        finderRelativeH,
        doAbsoluteDiscard,
        avgDistanceMinDiscardThreshold,
        avgDistanceCost
      );

    // Add dynamic objects to the newly created pathfinder
    updatePathfinderWithDynamicObjects();
  }

  // Method to create pathfinder with existing grid
  private void createPathfinderWithGrid() {
    if (pathfinder != null) {
      pathfinder.close();
      pathfinder = null;
    }

    pathfinder =
      new AStarPathfinder(
        hybridGrid,
        nodePickStyle,
        finderRelativeW,
        finderRelativeH,
        doAbsoluteDiscard,
        avgDistanceMinDiscardThreshold,
        avgDistanceCost
      );

    // Add dynamic objects to the newly created pathfinder
    updatePathfinderWithDynamicObjects();
  }

  private void updatePathfinderWithDynamicObjects() {
    if (pathfinder != null) {
      // Clear existing dynamic objects
      pathfinder.clearHybridObjects();
      pathfinder.clearUncertentyFields();

      // Add hybrid obstacles
      if (!hybridObstacles.isEmpty()) {
        float[] allHybridObstacles = new float[hybridObstacles.size() * 4];
        int index = 0;
        for (float[] obstacle : hybridObstacles) {
          System.arraycopy(obstacle, 0, allHybridObstacles, index, 4);
          index += 4;
        }
        pathfinder.addHybridObjects(allHybridObstacles);
      }

      // Add uncertainty fields
      for (UncertaintyField field : uncertaintyFields) {
        pathfinder.addUncertentyField(
          field.center,
          field.radius,
          field.intensity
        );
      }
    }
  }

  private void calculatePath() {
    try {
      long startTime = System.nanoTime();
      path = pathfinder.calculate(startPoint, endPoint);
      long endTime = System.nanoTime();

      lastCalculationTime = (endTime - startTime) / 1_000_000; // Convert to milliseconds
      DecimalFormat df = new DecimalFormat("#,###.##");
      timeLabel.setText(
        "Calculation time: " + df.format(lastCalculationTime) + " ms"
      );

      if (lastCalculationTime < 1) {
        // If time is less than 1ms, show in microseconds
        long microseconds = (endTime - startTime) / 1_000;
        timeLabel.setText(
          "Calculation time: " + df.format(microseconds) + " μs"
        );
      }
    } catch (Exception e) {
      System.err.println("Error calculating path: " + e.getMessage());
      path = new int[0];
      timeLabel.setText("Calculation failed!");
    }
  }

  private interface SliderCallback {
    void onValueChanged(int value);
  }

  private class GridPanel extends JPanel {

    private static final int CELL_SIZE = 12;
    private static final int GRID_OFFSET_X = 50;
    private static final int GRID_OFFSET_Y = 50;

    public GridPanel() {
      setBackground(Color.WHITE);

      addMouseListener(
        new MouseAdapter() {
          @Override
          public void mousePressed(MouseEvent e) {
            int gridX = (e.getX() - GRID_OFFSET_X) / CELL_SIZE;
            int gridY = (e.getY() - GRID_OFFSET_Y) / CELL_SIZE;

            if (gridX >= 0 && gridX < 50 && gridY >= 0 && gridY < 50) {
              switch (currentMode) {
                case OBSTACLE:
                  if (e.getButton() == MouseEvent.BUTTON1) {
                    boolean removed = false;
                    for (int i = 0; i < obstacles.size(); i++) {
                      if (
                        obstacles.get(i)[0] == gridX &&
                        obstacles.get(i)[1] == gridY
                      ) {
                        obstacles.remove(i);
                        removed = true;
                        break;
                      }
                    }

                    if (!removed) {
                      obstacles.add(new int[] { gridX, gridY });
                    }

                    createPathfinder();
                    calculatePath();
                    repaint();
                  }
                  break;
                case START_END:
                  if (e.getButton() == MouseEvent.BUTTON3) {
                    startPoint = new int[] { gridX, gridY };
                    calculatePath();
                    repaint();
                  } else if (e.getButton() == MouseEvent.BUTTON2) {
                    endPoint = new int[] { gridX, gridY };
                    calculatePath();
                    repaint();
                  }
                  break;
                case HYBRID_OBSTACLE:
                  if (e.getButton() == MouseEvent.BUTTON1) {
                    // Show dialog to get hybrid obstacle size
                    String input = JOptionPane.showInputDialog(
                      PathfinderVisualization.this,
                      "Enter hybrid obstacle width,height:",
                      "2.0,2.0"
                    );
                    if (input != null) {
                      try {
                        String[] parts = input.split(",");
                        float width = Float.parseFloat(parts[0].trim());
                        float height = Float.parseFloat(parts[1].trim());
                        float[] obstacle = new float[] {
                          gridX,
                          gridY,
                          width,
                          height,
                        };
                        hybridObstacles.add(obstacle);
                        updatePathfinderWithDynamicObjects();
                        calculatePath();
                        repaint();
                      } catch (Exception ex) {
                        JOptionPane.showMessageDialog(
                          PathfinderVisualization.this,
                          "Invalid input format"
                        );
                      }
                    }
                  }
                  break;
                case UNCERTAINTY_FIELD:
                  if (e.getButton() == MouseEvent.BUTTON1) {
                    // Show dialog to get uncertainty field parameters
                    String input = JOptionPane.showInputDialog(
                      PathfinderVisualization.this,
                      "Enter radius,intensity:",
                      "5.0,0.8"
                    );
                    if (input != null) {
                      try {
                        String[] parts = input.split(",");
                        float radius = Float.parseFloat(parts[0].trim());
                        float intensity = Float.parseFloat(parts[1].trim());
                        UncertaintyField field = new UncertaintyField(
                          new float[] { gridX, gridY },
                          radius,
                          intensity
                        );
                        uncertaintyFields.add(field);
                        updatePathfinderWithDynamicObjects();
                        calculatePath();
                        repaint();
                      } catch (Exception ex) {
                        JOptionPane.showMessageDialog(
                          PathfinderVisualization.this,
                          "Invalid input format"
                        );
                      }
                    }
                  }
                  break;
              }
            }
          }
        }
      );
    }

    @Override
    protected void paintComponent(Graphics g) {
      super.paintComponent(g);
      Graphics2D g2d = (Graphics2D) g;
      g2d.setRenderingHint(
        RenderingHints.KEY_ANTIALIASING,
        RenderingHints.VALUE_ANTIALIAS_ON
      );

      int gridSize = 50;

      g2d.setColor(Color.LIGHT_GRAY);
      for (int i = 0; i <= gridSize; i++) {
        g2d.drawLine(
          GRID_OFFSET_X,
          GRID_OFFSET_Y + i * CELL_SIZE,
          GRID_OFFSET_X + gridSize * CELL_SIZE,
          GRID_OFFSET_Y + i * CELL_SIZE
        );
        g2d.drawLine(
          GRID_OFFSET_X + i * CELL_SIZE,
          GRID_OFFSET_Y,
          GRID_OFFSET_X + i * CELL_SIZE,
          GRID_OFFSET_Y + gridSize * CELL_SIZE
        );
      }

      // Draw static obstacles
      for (int[] obstacle : obstacles) {
        g2d.setColor(Color.BLACK);
        g2d.fillRect(
          GRID_OFFSET_X + obstacle[0] * CELL_SIZE,
          GRID_OFFSET_Y + obstacle[1] * CELL_SIZE,
          CELL_SIZE,
          CELL_SIZE
        );
      }

      // Draw hybrid obstacles
      for (float[] obstacle : hybridObstacles) {
        g2d.setColor(new Color(139, 69, 19, 128)); // Semi-transparent brown
        int x = Math.round(obstacle[0]);
        int y = Math.round(obstacle[1]);
        int width = Math.round(obstacle[2]);
        int height = Math.round(obstacle[3]);
        g2d.fillRect(
          GRID_OFFSET_X + x * CELL_SIZE,
          GRID_OFFSET_Y + y * CELL_SIZE,
          width * CELL_SIZE,
          height * CELL_SIZE
        );
        g2d.setColor(new Color(139, 69, 19));
        g2d.drawRect(
          GRID_OFFSET_X + x * CELL_SIZE,
          GRID_OFFSET_Y + y * CELL_SIZE,
          width * CELL_SIZE,
          height * CELL_SIZE
        );
      }

      // Draw uncertainty fields
      for (UncertaintyField field : uncertaintyFields) {
        float[] center = field.center;
        float radius = field.radius;
        float intensity = field.intensity;

        // Create a gradient color based on intensity (more red = higher intensity)
        Color fieldColor = new Color(
          Math.min(1.0f, intensity),
          Math.max(0.0f, 1.0f - intensity),
          0.0f,
          0.3f
        );
        g2d.setColor(fieldColor);

        int x = Math.round(center[0]);
        int y = Math.round(center[1]);
        int diameterCells = Math.round(radius * 2);

        g2d.fillOval(
          GRID_OFFSET_X + (x - Math.round(radius)) * CELL_SIZE,
          GRID_OFFSET_Y + (y - Math.round(radius)) * CELL_SIZE,
          diameterCells * CELL_SIZE,
          diameterCells * CELL_SIZE
        );

        // Draw outline
        g2d.setColor(
          new Color(
            Math.min(1.0f, intensity),
            Math.max(0.0f, 1.0f - intensity),
            0.0f,
            0.7f
          )
        );
        g2d.drawOval(
          GRID_OFFSET_X + (x - Math.round(radius)) * CELL_SIZE,
          GRID_OFFSET_Y + (y - Math.round(radius)) * CELL_SIZE,
          diameterCells * CELL_SIZE,
          diameterCells * CELL_SIZE
        );
      }

      if (path != null && path.length > 0) {
        g2d.setColor(Color.BLUE);
        for (int i = 0; i < path.length; i += 2) {
          if (i + 1 < path.length) {
            int x = path[i];
            int y = path[i + 1];
            g2d.fillOval(
              GRID_OFFSET_X + x * CELL_SIZE + CELL_SIZE / 4,
              GRID_OFFSET_Y + y * CELL_SIZE + CELL_SIZE / 4,
              CELL_SIZE / 2,
              CELL_SIZE / 2
            );
          }
        }
      }

      g2d.setColor(Color.GREEN);
      g2d.fillOval(
        GRID_OFFSET_X + startPoint[0] * CELL_SIZE + CELL_SIZE / 4,
        GRID_OFFSET_Y + startPoint[1] * CELL_SIZE + CELL_SIZE / 4,
        CELL_SIZE / 2,
        CELL_SIZE / 2
      );

      g2d.setColor(Color.RED);
      g2d.fillOval(
        GRID_OFFSET_X + endPoint[0] * CELL_SIZE + CELL_SIZE / 4,
        GRID_OFFSET_Y + endPoint[1] * CELL_SIZE + CELL_SIZE / 4,
        CELL_SIZE / 2,
        CELL_SIZE / 2
      );

      g2d.setColor(Color.BLACK);
      g2d.drawString(
        "Start",
        GRID_OFFSET_X + startPoint[0] * CELL_SIZE - 15,
        GRID_OFFSET_Y + startPoint[1] * CELL_SIZE - 5
      );
      g2d.drawString(
        "End",
        GRID_OFFSET_X + endPoint[0] * CELL_SIZE - 10,
        GRID_OFFSET_Y + endPoint[1] * CELL_SIZE - 5
      );

      g2d.drawString("Grid Size: 50x50", GRID_OFFSET_X, GRID_OFFSET_Y - 20);

      if (path != null) {
        int pathLength = path.length / 2;
        g2d.drawString(
          "Path length: " + pathLength + " nodes",
          GRID_OFFSET_X,
          GRID_OFFSET_Y - 5
        );
      }
    }
  }

  public static void main(String[] args) {
    SwingUtilities.invokeLater(() -> {
      new PathfinderVisualization().setVisible(true);
    });
  }
}
