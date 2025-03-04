package org.pwrup.napoleon.bridge;

import java.io.*;
import java.nio.file.*;

public class NativeLoader {
  static {
    try {
      String libName = System.mapLibraryName("napoleon_core");
      InputStream in =
        NativeLoader.class.getResourceAsStream("/native/" + libName);
      if (in == null) throw new RuntimeException(
        "Native library not found in JAR!"
      );

      File tempFile = File.createTempFile(libName, "");
      Files.copy(in, tempFile.toPath(), StandardCopyOption.REPLACE_EXISTING);
      System.load(tempFile.getAbsolutePath());
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Rust native library", e);
    }
  }

  public static void init() {
    // Empty method to trigger static block
  }
}
