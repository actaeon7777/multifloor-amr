#!/usr/bin/env bash
# Select or roll back a validated release. Does not start robot processes.
set -euo pipefail
release=$(realpath -- "${1:?Usage: select_release.sh RELEASE_DIRECTORY CURRENT_SYMLINK}")
current=${2:?Supply the current-release symlink path}
[[ -f $release/RELEASE_VALIDATED && -f $release/RELEASE.json ]] || {
  echo 'A validated extracted source release is required' >&2; exit 1;
}
[[ ! -e $current || -L $current ]] || { echo 'Refusing to replace a real directory/file' >&2; exit 1; }
previous=$(readlink -f -- "$current" || true)
link_tmp="${current}.pending.$$"
trap 'rm -f -- "$link_tmp"' EXIT
ln -s -- "$release" "$link_tmp"
mv -Tf -- "$link_tmp" "$current"
printf 'Selected: %s\nPrevious: %s\nNo robot process was started.\n' "$release" "$previous"
