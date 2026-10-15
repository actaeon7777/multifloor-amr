#!/usr/bin/env python3
"""Create once without displaying a token; preserve and validate existing files."""
import argparse
import os
from pathlib import Path
import secrets
import stat


def prepare(filename):
    filename = filename.expanduser().absolute()
    new_directory = not filename.parent.exists()
    filename.parent.mkdir(parents=True, mode=0o700, exist_ok=True)
    if filename.parent.is_symlink():
        raise ValueError('Token directory must not be a symbolic link')
    if new_directory or filename.parent.name == 'amr_remote_bridge':
        os.chmod(filename.parent, 0o700)
    elif stat.S_IMODE(filename.parent.stat().st_mode) & 0o077:
        raise ValueError('Use a private token directory with mode 700')
    try:
        descriptor = os.open(filename, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    except FileExistsError:
        if filename.is_symlink() or not filename.is_file():
            raise ValueError('Token path must be a regular file') from None
        token = filename.read_text(encoding='utf-8').strip()
        if len(token) < 32 or any(c.isspace() for c in token):
            raise ValueError('Existing token is invalid; it was not overwritten') from None
        os.chmod(filename, stat.S_IRUSR | stat.S_IWUSR)
        return filename, False
    with os.fdopen(descriptor, 'w', encoding='utf-8') as output:
        output.write(secrets.token_hex(32) + '\n')
    return filename, True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--token-file', type=Path, default=Path.home() / '.config/amr_remote_bridge/token')
    args = parser.parse_args()
    try:
        filename, created = prepare(args.token_file)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print(('Created: ' if created else 'Reused: ') + str(filename))


if __name__ == '__main__':
    main()
