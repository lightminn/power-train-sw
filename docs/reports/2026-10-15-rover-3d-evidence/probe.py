"""Temporary CAD/GTK feasibility probe. No hardware, telemetry, or ops connections."""
from pathlib import Path
from functools import partial
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
import json
import threading
import time
import gi

gi.require_version('Gtk', '3.0')
gi.require_version('WebKit2', '4.1')
from gi.repository import Gtk, WebKit2, GLib

ROOT = Path(__file__).resolve().parent
events = []
started = time.monotonic()
class Handler(SimpleHTTPRequestHandler):
    def log_message(self, *_args):
        pass
server = ThreadingHTTPServer(('127.0.0.1', 0), partial(Handler, directory=str(ROOT)))
threading.Thread(target=server.serve_forever, daemon=True).start()
context = WebKit2.WebContext.new_ephemeral()
view = WebKit2.WebView.new_with_context(context)
manager = view.get_user_content_manager()
manager.register_script_message_handler('probe')
window = Gtk.Window(title='JETIN 3D 표시 검토 — 임시 시험')
window.set_default_size(1100, 760)
window.add(view)
finished = False

def record(value):
    value['elapsed_s'] = round(time.monotonic() - started, 3)
    events.append(value)
    print(json.dumps(value, ensure_ascii=False), flush=True)

def stop():
    global finished
    if finished:
        return False
    finished = True
    (ROOT / 'probe-results.json').write_text(json.dumps(events, ensure_ascii=False, indent=2))
    window.destroy()
    Gtk.main_quit()
    return False

def screenshot(name, continuation):
    def complete(webview, result, _data):
        try:
            surface = webview.get_snapshot_finish(result)
            surface.write_to_png(str(ROOT / name))
            record({'type': 'snapshot', 'path': str(ROOT / name)})
            continuation()
        except Exception as exc:
            record({'type': 'snapshot-error', 'error': str(exc)})
            stop()
    view.get_snapshot(WebKit2.SnapshotRegion.VISIBLE, WebKit2.SnapshotOptions.NONE,
                      None, complete, None)

def change_pose():
    script = "JSON.stringify({type:'motion',...window.review.pose(true),pixelProof:window.review.pixelProof()})"
    def complete(webview, result, _data):
        try:
            value = webview.evaluate_javascript_finish(result)
            record(json.loads(value.to_string()))
            webview.evaluate_javascript("window.webkit.messageHandlers.probe.postMessage(JSON.stringify({type:'click-test',...window.review.click('front_left')}))", -1, None, None, None, None, None)
            GLib.timeout_add(500, lambda: (screenshot('articulated.png', stop), False)[1])
        except Exception as exc:
            record({'type': 'motion-error', 'error': str(exc)})
            stop()
    view.evaluate_javascript(script, -1, None, None, None, complete, None)

def receive(_manager, message):
    value = json.loads(message.get_js_value().to_string())
    record(value)
    if value['type'] == 'ready':
        GLib.timeout_add(500, lambda: (screenshot('neutral.png', change_pose), False)[1])
    elif value['type'] == 'error':
        stop()

manager.connect('script-message-received::probe', receive)
window.connect('delete-event', lambda *_args: stop())
window.show_all()
view.load_uri(f'http://127.0.0.1:{server.server_port}/index.html')
GLib.timeout_add_seconds(90, lambda: (record({'type': 'timeout'}), stop(), False)[2])
Gtk.main()
server.shutdown()
