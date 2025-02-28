package org.pwrup.napoleon.bridge;

import java.io.File;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;

public class AStarPathfinder {
  static {
    try {
      // Map the correct library name
      String libName = System.mapLibraryName("napoleon_core"); // Not "napoleon"!

      // Extract from JAR to a temp file
      InputStream in = AStarPathfinder.class.getResourceAsStream("/native/" + libName);
      if (in == null)
        throw new RuntimeException("Native library not found in JAR!");

      File tempFile = File.createTempFile(libName, "");
      Files.copy(in, tempFile.toPath(), StandardCopyOption.REPLACE_EXISTING);
      System.load(tempFile.getAbsolutePath());
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Rust native library", e);
    }
  }

  public native int[] calculate(int[] static_obstacles, int[] size_x_y, int[] center_x_y, float square_size_meters,
      float[] robot_avg_velocity_x_m_s, int[] start_x_y, int[] end_x_y, float[] finder_relative_w_h);
}
