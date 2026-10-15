"""Verify and extract the pinned official Isaac Sim workstation distribution."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[2]
ARCHIVE = ROOT / "runtime/downloads/isaac-sim-standalone-6.1.0-linux-x86_64.zip"
DESTINATION = ROOT / "runtime/isaac-sim-6.1.0"
EXPECTED_MD5 = "b471383a51f259e0af22541f05c34b0e"


def main():
    with ARCHIVE.open("rb") as stream:
        checksum = hashlib.file_digest(stream, "md5").hexdigest()
    if checksum != EXPECTED_MD5:
        raise RuntimeError(f"Incomplete or corrupted download: {checksum}")
    with zipfile.ZipFile(ARCHIVE) as archive:
        files = archive.infolist()
        unpacked_bytes = sum(info.file_size for info in files)
        for info in files:
            path = Path(info.filename)
            if path.is_absolute() or ".." in path.parts:
                raise ValueError(f"Unexpected archive path: {info.filename}")
    if DESTINATION.exists() and any(DESTINATION.iterdir()):
        raise RuntimeError("Runtime directory is not empty; inspect existing installation before extracting")
    available = shutil.disk_usage(ROOT).free
    assert available > unpacked_bytes + 5 * 1024**3, "Insufficient free disk space"
    print(f"Checksum verified; extracting {len(files)} entries ({unpacked_bytes / 2**30:.2f} GiB)", flush=True)
    DESTINATION.mkdir(parents=True, exist_ok=True)
    subprocess.run(["unzip", "-q", str(ARCHIVE), "-d", str(DESTINATION)], check=True)
    assert (DESTINATION / "isaac-sim.sh").is_file()
    assert (DESTINATION / "python.sh").is_file()
    record = {"version": "6.1.0", "method": "official workstation zip", "path": str(DESTINATION),
              "archive_md5": checksum, "archive_entries": len(files), "unpacked_bytes": unpacked_bytes,
              "extracted": True, "runtime_verified": False}
    (ROOT / "reports/installation.json").write_text(json.dumps(record, indent=2) + "\n")
    print("ISAAC_SIM_EXTRACTED " + str(DESTINATION), flush=True)


if __name__ == "__main__":
    main()
