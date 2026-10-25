#!/usr/bin/env bash
# Builds the firmware on the desktop PC (which has the ESP32 toolchain) and
# copies the binaries to build/flash on this machine, ready for upload_firmware.py.
# The current local branch must be pushed: the desktop builds origin/<branch>.
set -euo pipefail

remote_host=desktop
remote_repo='~/Dokumente/Bastelei/roboter/Navbot-ES02'

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
branch="$(git -C "$repo_root" branch --show-current)"
[[ -n "$branch" ]] || { echo "error: detached HEAD, check out a branch first." >&2; exit 1; }

git -C "$repo_root" fetch --quiet origin "$branch"
if [[ "$(git -C "$repo_root" rev-parse HEAD)" != "$(git -C "$repo_root" rev-parse "origin/$branch")" ]]; then
  echo "error: $branch differs from origin/$branch; push your commits first." >&2
  exit 1
fi

echo "Building $branch on $remote_host"
ssh "$remote_host" bash -l -s -- "$branch" <<REMOTE
set -euo pipefail
cd $remote_repo
git fetch --quiet origin "\$1"
git checkout --quiet --force -B "\$1" "origin/\$1"
python3 scripts/build_firmware.py
REMOTE

mkdir -p "$repo_root/build/flash"
rsync -a --delete "$remote_host:$remote_repo/build/flash/" "$repo_root/build/flash/"
echo "Firmware delivered to $repo_root/build/flash"
