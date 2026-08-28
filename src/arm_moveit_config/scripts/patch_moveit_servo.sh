#!/usr/bin/env bash
# Vendors moveit_servo 2.12.4 into src/ with nix/moveit-servo-underactuated-svd.patch
# applied, so colcon overlays the stock package.
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
dest="$repo_root/src/moveit_servo"

[ -d "$dest" ] && { echo "$dest already exists; delete it to re-vendor." >&2; exit 1; }

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
git clone --depth 1 --branch 2.12.4 https://github.com/moveit/moveit2 "$tmp/moveit2"
cp -r "$tmp/moveit2/moveit_ros/moveit_servo" "$dest"
patch -d "$dest" -p1 < "$repo_root/nix/moveit-servo-underactuated-svd.patch"

echo
echo "Vendored patched moveit_servo into $dest"
echo "  rosdep install --from-paths src --ignore-src -y"
echo "  colcon build"
