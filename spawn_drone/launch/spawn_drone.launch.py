"""Launch PX4 SITL (x500 with gimbal) in the custom industrial_zone world,
along with the MicroXRCE Agent.  Any leftover sim processes from a previous
run are killed first, then the Gazebo world (server + GUI) is pre-warmed so
that PX4 spawns the drone onto an idle, fully loaded server -- required for
worlds that pull many models from Gazebo Fuel.

Usage:
    ros2 launch spawn_drone spawn_drone.launch.py
"""


"""
param set SYS_FAILURE_EN 1
param set CA_FAILURE_MODE 1
failure motor off -i 1
"""

import glob
import os
import re
import shutil
import signal
import subprocess
import time
import urllib.parse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction)
from launch.substitutions import LaunchConfiguration


FUEL_CACHE = os.path.expanduser('~/.gz/fuel/fuel.gazebosim.org')
FUEL_LINKS = os.path.expanduser('~/.gz/argus_fuel_links')

# Substrings identifying a simulation process we own.  Anything matching
# these and not part of our own process tree is a leftover from a previous
# run and is killed before we start.
SIM_PROCESS_MARKERS = (
    'gz sim',
    'bin/px4',
    'MicroXRCEAgent',
    'parameter_bridge',
    'prewarm_gz_world',
    'ros2 launch',
)


def _ancestors():
    """PIDs of this process and all its parents (never kill ourselves)."""
    pids = set()
    pid = os.getpid()
    while pid > 1 and pid not in pids:
        pids.add(pid)
        try:
            with open(f'/proc/{pid}/stat') as fh:
                # "pid (comm) state ppid ..." -- comm may contain spaces
                pid = int(fh.read().rsplit(')', 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            break
    return pids


def _find_sim_processes():
    """Leftover simulation processes from a previous run."""
    keep = _ancestors()
    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in keep:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as fh:
                cmdline = fh.read().replace(b'\0', b' ').decode(
                    'utf-8', 'ignore').strip()
        except OSError:
            continue
        if not cmdline:
            continue
        if any(marker in cmdline for marker in SIM_PROCESS_MARKERS):
            found.append((pid, cmdline))
    return found


def _alive(pid):
    """True if pid exists and is not a reaped-but-unwaited zombie."""
    try:
        with open(f'/proc/{pid}/stat') as fh:
            return fh.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except OSError:
        return False


def _cleanup_stale_processes(timeout=10.0):
    """Kill leftover sim processes and wait for them to exit.

    Re-launching without this hits "bind error port: 8888, errno 98"
    from a stale MicroXRCEAgent, and prewarm_gz_world.sh refuses to start
    because a Gazebo world is already running.
    """
    victims = _find_sim_processes()
    if not victims:
        return

    print('[spawn_drone] cleaning up leftovers from a previous run:')
    for pid, cmdline in victims:
        print(f'  pid {pid}: {cmdline[:90]}')

    for pid, _ in victims:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass

    deadline = time.time() + timeout
    while time.time() < deadline and any(_alive(p) for p, _ in victims):
        time.sleep(0.3)

    for pid, _ in victims:
        if _alive(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass

    # let the OS release the sockets (udp 8888 / 14550) we just freed
    time.sleep(1.0)
    print('[spawn_drone] cleanup done')


def _prefetch_fuel_models(world_file):
    """Download any Gazebo Fuel models referenced by the world that are not
    cached yet.  PX4 only waits ~30 s for the Gazebo world, so first-time
    downloads must happen before PX4 is started."""
    try:
        with open(world_file) as fh:
            uris = sorted(set(re.findall(r'<uri>(https://fuel\.gazebosim\.org/[^<]+)</uri>', fh.read())))
    except OSError:
        return False
    for uri in uris:
        m = re.match(r'https://fuel\.gazebosim\.org/1\.0/([^/]+)/models/([^/]+)', uri)
        if not m:
            continue
        owner, name = m.group(1).lower(), m.group(2)
        cached = any(os.path.isdir(os.path.join(FUEL_CACHE, owner, 'models', n))
                     for n in {name.lower(), urllib.parse.unquote(name).lower()})
        if cached:
            continue
        model_uri = f'https://fuel.gazebosim.org/1.0/{m.group(1)}/models/{name}'
        print(f'[spawn_drone] Downloading Fuel model {urllib.parse.unquote(name)} ...')
        try:
            subprocess.run(['gz', 'fuel', 'download', '-u', model_uri], check=False, timeout=600)
        except Exception as exc:  # noqa: BLE001
            print(f'[spawn_drone] WARNING: could not download {model_uri}: {exc}')
    return bool(uris)


def _fuel_resource_paths():
    """Many older Fuel models reference textures by bare file name or as
    model://<name>/materials/..., which Gazebo Harmonic cannot resolve from the
    versioned Fuel cache.  Expose the cache through GZ_SIM_RESOURCE_PATH:
    a folder of <name> -> <latest version> symlinks plus every texture dir."""
    paths = []
    if not os.path.isdir(FUEL_CACHE):
        return paths
    os.makedirs(FUEL_LINKS, exist_ok=True)
    for model_dir in glob.glob(os.path.join(FUEL_CACHE, '*', 'models', '*')):
        versions = [v for v in os.listdir(model_dir) if v.isdigit()]
        if not versions:
            continue
        latest = os.path.join(model_dir, max(versions, key=int))
        name = urllib.parse.unquote(os.path.basename(model_dir)).lower().replace(' ', '_')
        link = os.path.join(FUEL_LINKS, name)
        try:
            if os.path.islink(link) and os.readlink(link) != latest:
                os.remove(link)
            if not os.path.exists(link):
                os.symlink(latest, link)
        except OSError:
            pass
        for sub in ('materials/textures', 'meshes'):
            d = os.path.join(latest, sub)
            if os.path.isdir(d):
                paths.append(d)
    return [FUEL_LINKS] + paths


def _launch_setup(context, *args, **kwargs):
    px4_dir = LaunchConfiguration('px4_dir').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    model_target = LaunchConfiguration('model_target').perform(context)
    gcs_host = LaunchConfiguration('gcs_host').perform(context)
    do_cleanup = LaunchConfiguration('cleanup').perform(context).lower() in (
        '1', 'true', 'yes')
    # Runs here, synchronously, so no ExecuteProcess has started yet.
    if do_cleanup:
        _cleanup_stale_processes()
    # PX4's mavlink module only accepts dotted-quad IPs in its -t
    # argument (inet_aton), so resolve hostnames before passing on.
    try:
        import socket
        gcs_host = socket.gethostbyname(gcs_host)
    except Exception:
        pass

    # Copy our custom world into the PX4 worlds directory so that
    # PX4_GZ_WORLD=<world_name> can find it (${PX4_GZ_WORLDS}/<world>.sdf).
    worlds_share = os.path.join(
        get_package_share_directory('spawn_drone'), 'worlds')
    px4_worlds_dir = os.path.join(
        px4_dir, 'Tools', 'simulation', 'gz', 'worlds')
    src_world = os.path.join(worlds_share, f'{world_name}.sdf')
    dst_world = os.path.join(px4_worlds_dir, f'{world_name}.sdf')
    if os.path.exists(src_world) and px4_worlds_dir and os.path.isdir(px4_worlds_dir):
        try:
            shutil.copyfile(src_world, dst_world)
        except PermissionError:
            pass

    px4_env = dict(os.environ)
    px4_env['PX4_GZ_WORLD'] = world_name

    # Worlds that pull models from Gazebo Fuel (e.g. industrial_zone_real):
    # pre-download them and make their textures resolvable.
    uses_fuel = _prefetch_fuel_models(dst_world if os.path.exists(dst_world) else src_world)
    fuel_paths = _fuel_resource_paths() if uses_fuel else []
    if fuel_paths:
        existing = px4_env.get('GZ_SIM_RESOURCE_PATH', '')
        px4_env['GZ_SIM_RESOURCE_PATH'] = ':'.join(fuel_paths + ([existing] if existing else []))
    px4_env['GZ_IP'] = '127.0.0.1'
    px4_env['PX4_GCS_HOST'] = gcs_host

    # Leave the Gazebo GUI camera under user control.  px4-rc.gzsim otherwise
    # publishes a one-shot /gui/track FOLLOW message that pins the view to
    # the drone for the whole session, so you cannot fly the camera around.
    px4_env['PX4_GZ_NO_FOLLOW'] = '1'

    # Spawn the drone just above the helipad: the x500's
    # landing gear reaches ~0.227 m below the model origin
    # and the apron surface sits at z~0.03, so 0.28 m
    # puts the gear a couple of centimetres above the pad
    # (it then settles gently onto it).
    px4_env['PX4_GZ_MODEL_POSE'] = '0,0,0.28,0,0,0'

    # Pre-warm the Gazebo world (server + GUI) and run PX4
    # against it.  px4-rc.gzsim spawns the drone with a 5 s
    # EntityFactory call whose result it ignores and sends
    # the camera-follow message exactly once, so a world
    # that is still loading its Fuel models swallows both
    # and the drone never appears.  prewarm_gz_world.sh
    # starts the world, lets the server and GUI finish
    # loading every mesh, and only then runs
    # `make px4_sitl <model_target>`; PX4 finds the
    # already-running world and spawns onto an idle server.
    sim_process = ExecuteProcess(
        cmd=['bash', os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'prewarm_gz_world.sh'),
             dst_world if os.path.exists(dst_world) else src_world,
             world_name, model_target, px4_dir],
        cwd=px4_dir,
        env=px4_env,
        output='screen',
    )

    xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
    )

    return [sim_process, xrce_agent]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir',
            default_value='/home/ubuntu/Desktop/argus/PX4-Autopilot',
            description='Path to the PX4-Autopilot directory'),
        DeclareLaunchArgument(
            'world',
            default_value='industrial_zone_real',
            description='Gazebo world name (without .sdf).  Default is the '
                        'Fuel-based industrial_zone_real; use '
                        'industrial_zone for the lightweight one'),
        DeclareLaunchArgument(
            'model_target',
            default_value='gz_x500_gimbal',
            description='PX4 SITL gz target, e.g. gz_x500_gimbal'),
        DeclareLaunchArgument(
            'gcs_host',
            default_value='host.docker.internal',
            description='IP/hostname of the machine running QGC (GCS partner for MAVLink)'),
        DeclareLaunchArgument(
            'cleanup',
            default_value='true',
            description='Kill leftover sim processes from a previous run before '
                        'starting (avoids the udp 8888 bind error and the '
                        '"world already running" abort)'),
        OpaqueFunction(function=_launch_setup),
    ])