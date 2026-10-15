"""Download the official workstation archive, with resumable HTTP ranges."""
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import os
from pathlib import Path
import time
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[2]
URL = "https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.1.0-linux-x86_64.zip"
MD5 = "b471383a51f259e0af22541f05c34b0e"
DEST = ROOT / "runtime/downloads/isaac-sim-standalone-6.1.0-linux-x86_64.zip"
STATE = DEST.with_suffix(".progress.json")
CHUNK = 64 * 1024 * 1024


def main():
    DEST.parent.mkdir(parents=True, exist_ok=True)
    with urlopen(Request(URL, method="HEAD"), timeout=60) as response:
        total = int(response.headers["Content-Length"])
    if STATE.exists():
        state = json.loads(STATE.read_text())
        assert state["url"] == URL and state["total"] == total
    else:
        prefix = DEST.stat().st_size if DEST.exists() else 0
        assert prefix <= total
        state = {"url": URL, "total": total, "prefix": prefix, "done": []}
    ranges = [(i, min(i + CHUNK, total) - 1) for i in range(state["prefix"], total, CHUNK)]
    done = set(state["done"])
    fd = os.open(DEST, os.O_RDWR | os.O_CREAT, 0o644)
    os.ftruncate(fd, total)

    def save():
        state["done"] = sorted(done)
        temp = STATE.with_suffix(".tmp")
        temp.write_text(json.dumps(state, indent=2) + "\n")
        temp.replace(STATE)

    save()

    def fetch(bounds):
        start, end = bounds
        for attempt in range(4):
            try:
                req = Request(URL, headers={"Range": f"bytes={start}-{end}"})
                with urlopen(req, timeout=90) as response:
                    assert response.status == 206, "Server did not honor Range"
                    assert response.headers["Content-Range"] == f"bytes {start}-{end}/{total}"
                    pos = start
                    while data := response.read(1024 * 1024):
                        assert pos + len(data) <= end + 1
                        offset = 0
                        while offset < len(data):
                            offset += os.pwrite(fd, data[offset:], pos + offset)
                        pos += len(data)
                    assert pos == end + 1, "Incomplete HTTP range"
                return start
            except Exception:
                if attempt == 3:
                    raise
                time.sleep(2 * (attempt + 1))

    started = time.monotonic()
    pending = [bounds for bounds in ranges if bounds[0] not in done]
    print(f"Downloading {total / 2**30:.2f} GiB, {len(pending)} ranges remaining", flush=True)
    try:
        with ThreadPoolExecutor(max_workers=8) as pool:
            for future in as_completed([pool.submit(fetch, bounds) for bounds in pending]):
                done.add(future.result())
                os.fsync(fd)
                save()
                completed = state["prefix"] + sum(b - a + 1 for a, b in ranges if a in done)
                print(f"{completed / total:.1%} ({completed / 2**30:.2f}/{total / 2**30:.2f} GiB), elapsed {time.monotonic() - started:.0f}s", flush=True)
    finally:
        os.close(fd)
    with DEST.open("rb") as archive:
        actual = hashlib.file_digest(archive, "md5").hexdigest()
    assert actual == MD5, f"Archive checksum mismatch: {actual} != {MD5}"
    ROOT.joinpath("reports/download_verified.json").write_text(json.dumps({"url": URL, "bytes": total, "md5": actual, "verified": True}, indent=2) + "\n")
    print(f"Verified official checksum: {actual}", flush=True)


if __name__ == "__main__":
    main()
