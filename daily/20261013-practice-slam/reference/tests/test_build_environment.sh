#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
script=${1:-"$root/build_lyrical.sh"}
test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT
cp "$script" "$test_dir/build_lyrical.sh"
: > "$test_dir/sources.repos"
mkdir "$test_dir/bin"
for tool in vcs rosdep colcon git; do
  cat > "$test_dir/bin/$tool" <<'SH'
#!/usr/bin/env bash
echo invoked >> "$BUILD_TEST_TRACE"
SH
  chmod +x "$test_dir/bin/$tool"
done
export PATH="$test_dir/bin:$PATH"
export BUILD_TEST_TRACE="$test_dir/trace"

if ROS_DISTRO=jazzy bash "$test_dir/build_lyrical.sh" > "$test_dir/wrong.log" 2>&1; then
  echo 'FAIL: another ROS distribution was accepted' >&2
  exit 1
fi
grep -q 'Lyrical is required' "$test_dir/wrong.log"
test ! -f "$BUILD_TEST_TRACE"

ROS_DISTRO=lyrical bash "$test_dir/build_lyrical.sh" > "$test_dir/ready.log" 2>&1
grep -q 'Next: source install/setup.bash' "$test_dir/ready.log"
test -s "$BUILD_TEST_TRACE"
rm "$BUILD_TEST_TRACE"

if [[ -f /opt/ros/lyrical/setup.bash ]]; then
  env -u ROS_DISTRO bash "$test_dir/build_lyrical.sh" > "$test_dir/empty.log" 2>&1
  grep -q 'Next: source install/setup.bash' "$test_dir/empty.log"
  test -s "$BUILD_TEST_TRACE"
  echo 'PASS: empty environment loads installed Lyrical'
else
  if env -u ROS_DISTRO bash "$test_dir/build_lyrical.sh" > "$test_dir/missing.log" 2>&1; then
    echo 'FAIL: missing Lyrical installation was accepted' >&2
    exit 1
  fi
  grep -q 'NVIDIA Buildfarm' "$test_dir/missing.log"
  test ! -f "$BUILD_TEST_TRACE"
  echo 'PASS: missing Lyrical gives installation instructions'
fi
echo 'PASS: Lyrical accepted; other distributions rejected before build tools run'
