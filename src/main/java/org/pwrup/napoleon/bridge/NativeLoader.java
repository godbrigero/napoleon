package org.pwrup.napoleon.bridge;

import java.io.*;
import java.nio.file.*;

public class NativeLoader {
  static {
    try {
      String libName = System.mapLibraryName("napoleon_core");
      System.out.println("Looking for native library: " + libName);

      InputStream in =
        NativeLoader.class.getResourceAsStream("/native/" + libName);
      if (in == null) {
        System.out.println(
          "Failed to find native library in JAR at /native/" + libName
        );
        throw new RuntimeException("Native library not found in JAR!");
      }
      System.out.println("Found native library in JAR");

      File tempFile = File.createTempFile(libName, "");
      System.out.println(
        "Extracting to temp file: " + tempFile.getAbsolutePath()
      );
      Files.copy(in, tempFile.toPath(), StandardCopyOption.REPLACE_EXISTING);

      System.out.println(
        "Loading native library from: " + tempFile.getAbsolutePath()
      );
      System.load(tempFile.getAbsolutePath());
      System.out.println("Successfully loaded native library");
    } catch (Exception e) {
      System.err.println("Error loading native library: " + e.getMessage());
      e.printStackTrace();
      throw new RuntimeException("Failed to load Rust native library", e);
    }
  }

  public static void init() {
    // Empty method to trigger static block
  }
}
