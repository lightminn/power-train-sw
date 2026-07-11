"""Real-process fake used to prove Dashboard/Gateway ownership boundaries."""
import os
import socket
import subprocess
import sys

from l515_dashboard.protocol import decode_request, encode_message, response

def main():
    path,pid_path=sys.argv[1:3]
    child=subprocess.Popen([sys.executable,"-c","import time; time.sleep(300)"])
    open(pid_path,"w",encoding="ascii").write(str(child.pid))
    sock=socket.socket(socket.AF_UNIX); sock.bind(path); sock.listen(); stopping=False
    try:
        while not stopping:
            conn,_=sock.accept()
            with conn:
                req=decode_request(conn.makefile("rb").readline().rstrip(b"\n"),65536)
                stopping=req["type"]=="stop_gateway"
                payload={"accepted":True} if stopping else {"state":"RUNNING"}
                conn.sendall(encode_message(response(req["request_id"],payload),65536))
    finally:
        sock.close()
        try: os.unlink(path)
        except FileNotFoundError: pass
        child.terminate(); child.wait(timeout=2)

if __name__=="__main__": main()
