"""TETRA 브링업 공용: ttyS 포트를 스캔해 구동보드·전원보드·IMU 가 꽂힌 포트를 찾는다.

로봇마다 보드 배선이 달라(예: MH 표준은 구동 ttyS0·전원 ttyS1 인데 어떤 MH 는 구동 ttyS3·전원 ttyS2)
포트를 고정하면 드라이버가 엉뚱한 장치를 연다. 그래서 드라이버를 띄우기 전에 각 포트에 읽기 전용 명령을 보내
응답으로 장치를 가린다. 보내는 것은 버전 조회(VR)와 오일러각 조회(e)뿐이라 장치 상태를 바꾸지 않는다.

  - 구동·전원 보드: STX 'V' 'R' ETX LRC → STX + 8글자 ASCII("보드종류 YYMMDD 버전") + ETX + LRC.
    보드종류 첫 글자가 '1' 이면 구동보드, 다른 숫자면 전원보드 (실측: 구동 "12605130", 전원 "42609190", 예전 전원 "2…").
  - IMU(iAHRS): "e\\n" → "e=<roll>,<pitch>,<yaw>\\r\\n" (오류면 '!' 로 시작).

런치 인자 wheel_port / power_port / imu_port 의 기본값은 'auto' 이다. 값을 직접 주면 그 장치는 스캔하지 않고 그대로 쓴다.
"""

import os
import select
import termios
import time
from typing import Dict, List, Optional

from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration

AUTO = 'auto'
DEVICE_ROLES = ('wheel', 'power', 'imu')
DEFAULT_SCAN_PORTS = ','.join(f'/dev/ttyS{index}' for index in range(6))

STX = 0x02
ETX = 0x03
VERSION_REQUEST = bytes([STX, ord('V'), ord('R'), ETX, ord('V') ^ ord('R') ^ ETX])
VERSION_LENGTH = 8
DRIVE_BOARD_TYPE = '1'
IMU_EULER_REQUEST = b'e\n'

# 보드는 수 ms 안에 답한다. 빈 포트는 이 시간을 다 기다리므로 6개 포트 전체가 몇 초 안에 끝나게 짧게 둔다
BOARD_REPLY_TIMEOUT_SEC = 0.25
BOARD_REQUEST_ATTEMPTS = 2
IMU_REPLY_TIMEOUT_SEC = 0.2


def _open_raw_115200(port_path: str) -> int:
    port_fd = os.open(port_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attributes = termios.tcgetattr(port_fd)
    attributes[0] = 0  # iflag: 소프트웨어 흐름 제어·변환 없음
    attributes[1] = 0  # oflag
    attributes[2] = termios.B115200 | termios.CS8 | termios.CREAD | termios.CLOCAL
    attributes[3] = 0  # lflag: raw
    attributes[4] = termios.B115200
    attributes[5] = termios.B115200
    termios.tcsetattr(port_fd, termios.TCSANOW, attributes)
    termios.tcflush(port_fd, termios.TCIOFLUSH)
    return port_fd


def _exchange(port_fd: int, request: bytes, timeout_sec: float, is_complete) -> bytes:
    termios.tcflush(port_fd, termios.TCIFLUSH)
    os.write(port_fd, request)
    reply = b''
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and not is_complete(reply):
        readable, _, _ = select.select([port_fd], [], [], 0.02)
        if not readable:
            continue
        try:
            reply += os.read(port_fd, 256)
        except BlockingIOError:
            pass
    return reply


def _board_reply_complete(reply: bytes) -> bool:
    etx_index = reply.find(bytes([ETX]), reply.find(bytes([STX])) + 1)
    return reply.find(bytes([STX])) >= 0 and etx_index >= 0 and len(reply) > etx_index + 1


def parse_board_version(reply: bytes) -> Optional[str]:
    """VR 응답에서 8글자 버전 문자열을 꺼낸다. 형식이나 LRC 가 틀리면 None."""
    stx_index = reply.find(bytes([STX]))
    if stx_index < 0:
        return None
    etx_index = reply.find(bytes([ETX]), stx_index + 1)
    if etx_index < 0 or len(reply) <= etx_index + 1:
        return None
    payload = reply[stx_index + 1:etx_index]
    expected_lrc = 0
    for byte in payload + bytes([ETX]):
        expected_lrc ^= byte
    if reply[etx_index + 1] != expected_lrc or len(payload) != VERSION_LENGTH:
        return None
    version = payload.decode('ascii', errors='replace')
    return version if version.isdigit() else None


def is_imu_reply(reply: bytes) -> bool:
    return reply.startswith(b'e=') or reply.startswith(b'!')


def identify_device(port_path: str) -> Optional[str]:
    """포트에 붙은 장치를 'wheel' / 'power' / 'imu' 로 가린다. 응답이 없거나 모르는 장치면 None."""
    try:
        port_fd = _open_raw_115200(port_path)
    except OSError:
        return None
    try:
        for _ in range(BOARD_REQUEST_ATTEMPTS):
            version = parse_board_version(
                _exchange(port_fd, VERSION_REQUEST, BOARD_REPLY_TIMEOUT_SEC, _board_reply_complete))
            if version:
                return 'wheel' if version[0] == DRIVE_BOARD_TYPE else 'power'
        imu_reply = _exchange(port_fd, IMU_EULER_REQUEST, IMU_REPLY_TIMEOUT_SEC,
                              lambda reply: reply.endswith(b'\n') or reply.endswith(b'\r'))
        return 'imu' if is_imu_reply(imu_reply) else None
    except OSError:
        return None
    finally:
        os.close(port_fd)


def _ports_opened_by_other_processes(port_paths: List[str]) -> set:
    """이미 다른 프로세스(예: 남아 있는 드라이버)가 연 포트. 여기에 명령을 보내면 그 통신이 깨지므로 건너뛴다."""
    wanted = {os.path.realpath(path) for path in port_paths}
    opened = set()
    for pid in filter(str.isdigit, os.listdir('/proc')):
        if int(pid) == os.getpid():
            continue
        fd_dir = f'/proc/{pid}/fd'
        try:
            for fd_name in os.listdir(fd_dir):
                try:
                    target = os.readlink(os.path.join(fd_dir, fd_name))
                except OSError:
                    continue
                if target in wanted:
                    opened.add(target)
        except OSError:
            continue
    return opened


def scan_serial_devices(port_paths: List[str], roles: List[str]) -> Dict[str, str]:
    """port_paths 를 순서대로 검사해 roles 각각이 처음 발견된 포트를 돌려준다."""
    busy_ports = _ports_opened_by_other_processes(port_paths)
    found: Dict[str, str] = {}
    for port_path in port_paths:
        if len(found) == len(roles):
            break
        if not os.path.exists(port_path) or os.path.realpath(port_path) in busy_ports:
            continue
        role = identify_device(port_path)
        if role in roles and role not in found:
            found[role] = port_path
    return found


def _resolve_ports(context: LaunchContext, default_ports: Dict[str, str]) -> list:
    requested = {role: LaunchConfiguration(f'{role}_port').perform(context) for role in DEVICE_ROLES}
    auto_roles = [role for role in DEVICE_ROLES if requested[role] == AUTO]
    if not auto_roles:
        return []

    fixed_ports = {port for role, port in requested.items() if port != AUTO}
    scan_ports = [port.strip() for port in LaunchConfiguration('serial_scan_ports').perform(context).split(',')
                  if port.strip() and port.strip() not in fixed_ports]
    found = scan_serial_devices(scan_ports, auto_roles)

    actions = []
    for role in auto_roles:
        port = found.get(role, default_ports[role])
        source = 'scan' if role in found else 'not found, model default'
        actions.append(SetLaunchConfiguration(f'{role}_port', port))
        actions.append(LogInfo(msg=f'[serial scan] {role} port: {port} ({source})'))
    return actions


def serial_port_actions(default_ports: Dict[str, str]) -> list:
    """구동·전원·IMU 포트 인자 선언과 스캔 액션. 드라이버 노드보다 앞에 넣어야 노드가 스캔 결과를 받는다.

    default_ports: 스캔에서 못 찾았을 때 쓸 모델 기본 포트 {'wheel': ..., 'power': ..., 'imu': ...}
    """
    return [
        DeclareLaunchArgument('wheel_port', default_value=AUTO,
                              description="구동보드 포트. 'auto' 면 serial_scan_ports 를 스캔"),
        DeclareLaunchArgument('power_port', default_value=AUTO,
                              description="전원보드 포트. 'auto' 면 serial_scan_ports 를 스캔"),
        DeclareLaunchArgument('imu_port', default_value=AUTO,
                              description="IMU(iAHRS) 포트. 'auto' 면 serial_scan_ports 를 스캔"),
        DeclareLaunchArgument('serial_scan_ports', default_value=DEFAULT_SCAN_PORTS,
                              description='스캔할 포트 목록 (쉼표 구분)'),
        OpaqueFunction(function=_resolve_ports, kwargs={'default_ports': default_ports}),
    ]
