import argparse
from .app import DashboardApp
from .client import GatewayClient
from .config import DashboardConfig
from powertrain_observability.client import ObservabilityClient
from powertrain_observability.protocol import STATUS_SOCKET

def main():
    parser=argparse.ArgumentParser(description="Socket-only L515 Gateway Dashboard")
    parser.add_argument("--socket",default=DashboardConfig().socket_path)
    parser.add_argument("--observability-socket",default=STATUS_SOCKET)
    args=parser.parse_args()
    DashboardApp(
        GatewayClient(args.socket),
        observability_client=ObservabilityClient(args.observability_socket),
    ).run()

if __name__ == "__main__": main()
