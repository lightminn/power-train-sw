import argparse
from .app import DashboardApp
from .client import GatewayClient
from .config import DashboardConfig

def main():
    parser=argparse.ArgumentParser(description="Socket-only L515 Gateway Dashboard")
    parser.add_argument("--socket",default=DashboardConfig().socket_path)
    args=parser.parse_args(); DashboardApp(GatewayClient(args.socket)).run()

if __name__ == "__main__": main()
