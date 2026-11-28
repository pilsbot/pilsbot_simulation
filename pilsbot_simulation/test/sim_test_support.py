# Shared by the headless Gazebo launch tests (test_sim_*.launch.py): whether
# this machine can run them, the run's own Gazebo partition, and the
# processes it leaves behind.
import os
from pathlib import Path
import shutil
import signal
import time

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.actions import SetEnvironmentVariable

# Keeps this run's Gazebo transport apart from any other Gazebo on the machine
# (ROS_DOMAIN_ID is isolated by the CMake test runner), and finds this run's
# processes after the launch.
PARTITION = f'pilsbot_sim_test_{os.getpid()}'


def gazebo_unavailable():
    """Why this machine cannot run the test, or None."""
    if os.environ.get('PILSBOT_SKIP_GAZEBO_TESTS', '') not in ('', '0'):
        return 'PILSBOT_SKIP_GAZEBO_TESTS is set'
    if shutil.which('ign') is None:
        return 'Gazebo Fortress (ign) is not installed'
    for package in ['ros_gz_sim', 'ros_gz_bridge', 'gz_ros2_control']:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            return f'{package} is not installed'
    return None


def sim_launch_file():
    return Path(get_package_share_directory('pilsbot_simulation')) / 'launch' / 'sim.launch.py'


def partition_actions():
    """Launch actions that put the run's Gazebo into its own partition."""
    return [SetEnvironmentVariable('IGN_PARTITION', PARTITION),
            SetEnvironmentVariable('GZ_PARTITION', PARTITION)]


def processes_in_partition():
    """Pids of the processes started with this run's IGN_PARTITION."""
    marker = f'IGN_PARTITION={PARTITION}'.encode()
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            if marker in (entry / 'environ').read_bytes().split(b'\0'):
                found.append(int(entry.name))
        except OSError:
            pass  # gone, or not ours
    return found


def stop_processes(pids):
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 10.0
    while processes_in_partition() and time.monotonic() < deadline:
        time.sleep(0.2)
    for pid in processes_in_partition():
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def command_line(pid):
    try:
        return (Path('/proc') / str(pid) / 'cmdline').read_bytes() \
            .replace(b'\0', b' ').decode(errors='replace').strip()
    except OSError:
        return '(gone)'


class NothingLeftBehind:
    """
    Post-shutdown check of a launch test: the launch stopped all its processes.

    Mix it into a unittest.TestCase decorated with
    launch_testing.post_shutdown_test().
    """

    def tearDown(self):
        # Cleanup, not a check: whatever the test below finds is stopped, so
        # a failed run does not leave a gz server running on the machine.
        stop_processes(processes_in_partition())

    def test_the_launch_stopped_all_its_processes(self):
        # launch_testing ends the run with a signal to the launch's processes
        # only, as systemd or `kill <pid>` would, not to the process group.
        # The gz server must stop with them (sim.launch.py execs it). A few
        # seconds' grace for anything a launched process forked.
        deadline = time.monotonic() + 5.0
        while processes_in_partition() and time.monotonic() < deadline:
            time.sleep(0.2)
        leftovers = processes_in_partition()
        self.assertEqual(leftovers, [], 'processes still running after the launch ended: ' +
                         ', '.join(f'{pid} {command_line(pid)}' for pid in leftovers))
