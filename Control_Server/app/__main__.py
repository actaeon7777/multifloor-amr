"""Single-worker foreground entry point used by scripts/run_server.sh."""
import argparse
import ipaddress
import os

import uvicorn

from .config import Settings
from .main import create_app


def main():
    parser = argparse.ArgumentParser(description='AMR FastAPI central host')
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, default=int(os.environ.get('AMR_PORT', '8081')))
    args = parser.parse_args()
    try:
        address = ipaddress.ip_address(args.host)
        if not (address.is_loopback or address in ipaddress.ip_network('100.64.0.0/10')):
            raise ValueError('Bind to loopback or the actual host Tailscale IPv4 address')
        if not 1 <= args.port <= 65535:
            raise ValueError('Port must be between 1 and 65535')
        settings = Settings.from_env()
        settings.validate()
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print(f'AMR central host: http://{args.host}:{args.port}/ (operator login required)', flush=True)
    uvicorn.run(create_app(settings), host=args.host, port=args.port, workers=1,
                access_log=False, proxy_headers=False)


if __name__ == '__main__':
    main()

