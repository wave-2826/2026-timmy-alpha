#!/usr/bin/env python3
import os
import shutil
import sys
import tarfile
import tempfile
import urllib.request
from pathlib import Path

HOTSWAP_URL = "https://github.com/HotswapProjects/HotswapAgent/releases/download/RELEASE-2.0.3/hotswap-agent-2.0.3.jar"

# JetBrains runtimes
JBR_LINUX_URL = "https://cache-redirector.jetbrains.com/intellij-jbr/jbr_jcef-17.0.11-linux-x64-b1312.2.tar.gz"
JBR_WINDOWS_URL = "https://cache-redirector.jetbrains.com/intellij-jbr/jbr_jcef-17.0.11-windows-x64-b1312.2.tar.gz"

def download(url: str, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    print(f"downloading {url} -> {dest}")
    urllib.request.urlretrieve(url, str(dest))

def extract_tar_gz(archive: Path, dest_dir: Path) -> Path:
    print(f"extracting {archive} -> {dest_dir}")
    with tarfile.open(archive, "r:gz") as tf:
        tf.extractall(path=str(dest_dir))

    # find the first top-level directory created by the tarball
    for child in dest_dir.iterdir():
        if child.is_dir():
            return child
    
    return dest_dir

def main():
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent.parent

    libs_dir = repo_root / "libs"
    libs_dir.mkdir(parents=True, exist_ok=True)

    # Download HotswapAgent jar into libs/
    hotswap_dest = libs_dir / "hotswap-agent.jar"
    try:
        download(HOTSWAP_URL, hotswap_dest)
    except Exception as e:
        print(f"Failed to download HotswapAgent: {e}")
        sys.exit(1)

    # Prepare tmp files for JBR download/extract
    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)

        # pick URL based on platform
        if sys.platform.startswith("win"):
            jbr_url = JBR_WINDOWS_URL
            archive_name = "jbr-windows.tar.gz"
        else:
            jbr_url = JBR_LINUX_URL
            archive_name = "jbr-linux.tar.gz"

        jbr_archive = tmpdir / archive_name
        try:
            download(jbr_url, jbr_archive)
        except Exception as e:
            print(f"Failed to download JBR archive: {e}")
            print("HotswapAgent has been installed to:", hotswap_dest)
            sys.exit(1)

        extracted = None
        try:
            extracted = extract_tar_gz(jbr_archive, tmpdir / "extracted")
        except Exception as e:
            print(f"Failed to extract JBR archive: {e}")
            print("HotswapAgent has been installed to:", hotswap_dest)
            sys.exit(1)

        # choose destination path depending on platform
        home = Path(os.path.expanduser("~"))
        jbr_folder_name = "jbr-jcef-17"
        if sys.platform.startswith("win"):
            program_files = os.environ.get("ProgramFiles", r"C:\\Program Files")
            system_dest = Path(program_files) / "Java" / jbr_folder_name
            user_dest = home / jbr_folder_name
        else:
            system_dest = Path("/usr/lib/jvm") / jbr_folder_name
            user_dest = home / f".{jbr_folder_name}"

        dest = system_dest

        # fallback if not writable
        if not os.access(system_dest.parent, os.W_OK):
            dest = user_dest

        try:
            # move/copy extracted runtime to destination
            if dest.exists():
                # remove if empty or create if not exist
                if not any(dest.iterdir()):
                    pass
            else:
                dest.parent.mkdir(parents=True, exist_ok=True)

            print(f"Installing JetBrains JBR to: {dest}")

            if dest.exists():
                # if destination exists and is empty, remove it then move
                if not any(dest.iterdir()):
                    shutil.rmtree(dest)

            # extracted may point at a top-level directory; move it into place
            shutil.move(str(extracted), str(dest))
        except Exception as e:
            print(f"Failed to install JBR runtime: {e}")
            print("HotswapAgent is installed, but JBR was not installed.")
            sys.exit(1)

        java_bin = dest / "bin" / ("java.exe" if sys.platform.startswith("win") else "java")
        print("\nInstallation complete.")
        print("HotswapAgent jar:", hotswap_dest)
        print("JBR runtime installed to:", dest)
        print("Use the java binary at:", java_bin)

if __name__ == "__main__":
    main()

