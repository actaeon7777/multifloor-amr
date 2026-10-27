"""Foreground stdlib server used by scripts/run_server.sh; no ROS required."""
import argparse
from dataclasses import replace
import ipaddress
import logging
import os
from pathlib import Path
import signal
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/amr_system/amr_remote_bridge'))
from amr_remote_bridge.server import Registry, create_server
from amr_remote_bridge.slack_notify import SlackNotifier
from .config import Settings


def main():
    parser = argparse.ArgumentParser(description='AMR lightweight central console')
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, default=int(os.environ.get('AMR_PORT', '8081')))
    parser.add_argument('--slack-env', type=Path)
    parser.add_argument('--slack-rooms', type=Path)
    args = parser.parse_args()
    notifier = None
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    try:
        address = ipaddress.ip_address(args.host)
        if not (address.is_loopback or address in ipaddress.ip_network('100.64.0.0/10')):
            raise ValueError('Bind to loopback or the actual host Tailscale IPv4 address')
        if not 1 <= args.port <= 65535:
            raise ValueError('Port must be between 1 and 65535')
        settings = Settings.from_env()
        if args.slack_env:
            settings = replace(settings, slack_env=args.slack_env.expanduser())
        if args.slack_rooms:
            settings = replace(settings, slack_rooms=args.slack_rooms.expanduser())
        token = settings.validate()
        notifier = SlackNotifier(slack_env=settings.slack_env, slack_rooms=settings.slack_rooms)
        registry = Registry(settings.stale_after, settings.command_timeout, settings.lease_timeout,
                            settings.max_robots, notifier=notifier)
        server = create_server(registry, token, args.host, args.port, settings.robot_id,
                               body_timeout=settings.body_timeout)
    except (ValueError, OSError) as exc:
        if notifier:
            notifier.close()
        parser.error(str(exc))
    def terminate(*_args):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    print(f'AMR central host: http://{args.host}:{args.port}/ (operator login required)', flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        notifier.close()


if __name__ == '__main__':
    main()
