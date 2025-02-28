import sys
import os
import subprocess
import shutil
import glob
import argparse
import configparser
import logging
import stat


def check_folders():
    if (
        not os.path.exists("lib")
        or not os.path.isdir("lib/build")
        or not os.path.isdir("lib/vendor")
    ):
        os.makedirs("lib/build", exist_ok=True)
        os.makedirs("lib/vendor", exist_ok=True)


def delete_folder(folder_name):
    if os.path.exists(folder_name):
        shutil.rmtree(folder_name)


def main():
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )

    parser = argparse.ArgumentParser(description="Clone and build GitHub libraries")
    parser.add_argument("--config-file-path", type=str, help="Path to the config file")
    args = parser.parse_args()

    config = configparser.ConfigParser()
    config.read(args.config_file_path)

    check_folders()

    vendor_path = os.path.abspath("lib/vendor")
    os.chdir(vendor_path)

    for section in config.sections():
        if config.getboolean(section, "build_dynamically"):
            logging.info(f"Building {section}")
        else:
            logging.info(f"Skipping {section}")
            continue

        github_url = config.get(section, "github")
        use_branch = config.has_option(section, "branch")

        os.chdir(vendor_path)

        if config.getboolean(section, "force_clone") and os.path.exists(section):
            delete_folder(section)

        if not os.path.exists(section):
            print(f"Cloning {section}")
            subprocess.run(
                ["git", "clone", github_url, "--single-branch", section]
                + (
                    []
                    if not use_branch
                    else ["--branch", config.get(section, "branch")]
                )
            )
        else:
            logging.info(f"{section} already exists. Building from existing repo.")

        repo_path = os.path.join(vendor_path, section)
        os.chdir(repo_path)

        out = subprocess.run(["./gradlew", "build"], capture_output=True, text=True)
        print(out.stdout)
        if out.returncode != 0:
            logging.error(f"Failed to build {section}:\n{out.stderr}")
            exit(1)
        else:
            logging.info(f"Successfully built {section}")

        build_libs_path = os.path.join("build", "libs")
        for jar_file in glob.glob(os.path.join(build_libs_path, "*.jar")):
            dest_path = os.path.abspath(os.path.join(vendor_path, "..", "build"))
            shutil.copy(jar_file, dest_path)


if __name__ == "__main__":
    main()
    exit(0)
