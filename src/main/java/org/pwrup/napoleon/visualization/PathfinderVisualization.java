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
  private long lastCalculationTime = 0;
  private JLabel timeLabel;

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

    JButton clearObstaclesButton = new JButton("Clear Obstacles");
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
      "<html>Left click to place/remove obstacles<br>Right click to set start point<br>Middle click to set end point</html>"
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
              if (e.getButton() == MouseEvent.BUTTON1) {
                boolean removed = false;
                for (int i = 0; i < obstacles.size(); i++) {
                  if (
                    obstacles.get(i)[0] == gridX && obstacles.get(i)[1] == gridY
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
              } else if (e.getButton() == MouseEvent.BUTTON3) {
                startPoint = new int[] { gridX, gridY };
                calculatePath();
                repaint();
              } else if (e.getButton() == MouseEvent.BUTTON2) {
                endPoint = new int[] { gridX, gridY };
                calculatePath();
                repaint();
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

      for (int[] obstacle : obstacles) {
        g2d.setColor(Color.BLACK);
        g2d.fillRect(
          GRID_OFFSET_X + obstacle[0] * CELL_SIZE,
          GRID_OFFSET_Y + obstacle[1] * CELL_SIZE,
          CELL_SIZE,
          CELL_SIZE
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
