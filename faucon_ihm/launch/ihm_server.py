#!/usr/bin/env python3
"""
launch/ihm_server.py
────────────────────
Serveur HTTP minimaliste pour servir le build statique de l'IHM React.
Lancé automatiquement par ihm_launch.py.

Usage direct :
  python3 ihm_server.py 0.0.0.0 3000 /path/to/ihm/dist
"""

import sys
import os
import signal
from http.server import HTTPServer, SimpleHTTPRequestHandler
from functools import partial


class SPAHandler(SimpleHTTPRequestHandler):
    """
    Sert les fichiers statiques du build React.
    Redirige toutes les routes inconnues vers index.html (SPA routing).
    """

    def __init__(self, *args, directory=None, **kwargs):
        super().__init__(*args, directory=directory, **kwargs)

    def do_GET(self):
        
        requested = self.translate_path(self.path)
        if not os.path.exists(requested) or os.path.isdir(requested):
            # SPA fallback → index.html
            self.path = "/index.html"
        super().do_GET()

    def log_message(self, format, *args):
        
        if not self.path.endswith((".js", ".css", ".ico", ".png", ".woff2")):
            print(f"[IHM] {self.address_string()} → {args[0]}", flush=True)


def main():
    host    = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port    = int(sys.argv[2]) if len(sys.argv) > 2 else 3000
    ihm_dir = sys.argv[3] if len(sys.argv) > 3 else os.getcwd()

    if not os.path.isdir(ihm_dir):
        print(f"[IHM] ⚠  Répertoire IHM introuvable : {ihm_dir}", flush=True)
        print(f"[IHM] ⚠  Lancez d'abord : cd ihm && npm install && npm run build", flush=True)
        
        signal.pause()
        return

    handler = partial(SPAHandler, directory=ihm_dir)

    # SO_REUSEADDR : permet de relancer immédiatement sans attendre le timeout TCP
    HTTPServer.allow_reuse_address = True
    try:
        server = HTTPServer((host, port), handler)
    except OSError as e:
        if e.errno == 98:  # Address already in use
            # On ne tue JAMAIS le processus qui occupe le port (ce pourrait être
            # n'importe quel autre service) : on laisse l'utilisateur décider.
            print(f"[IHM] ✖  Port {port} déjà utilisé : serveur IHM non démarré.", flush=True)
            print(f"[IHM]    Identifier le processus : ss -ltnp 'sport = :{port}'", flush=True)
            print(f"[IHM]    Ou relancer sur un autre port : ihm_port:=<port>", flush=True)
            sys.exit(1)
        raise

    print(f"\n[IHM] ✔  Serveur démarré", flush=True)
    print(f"[IHM]    Local   → http://localhost:{port}", flush=True)
    if host in ("127.0.0.1", "localhost", "::1"):
        print(f"[IHM]    Réseau  → désactivé (accès local uniquement, ihm_lan:=true pour l'ouvrir)", flush=True)
    else:
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
        except Exception:
            local_ip = "<ip-de-la-machine>"
        print(f"[IHM]    Réseau  → http://{local_ip}:{port}  (⚠ sans authentification)", flush=True)
    print(f"[IHM]    Dossier → {ihm_dir}\n", flush=True)

    def shutdown(sig, frame):
        print("\n[IHM] Arrêt du serveur...", flush=True)
        # server.shutdown() attend la fin de serve_forever() : l'appeler depuis le
        # gestionnaire de signal (meme thread) bloquait indefiniment, et le port
        # restait occupe. On le lance dans un thread separe.
        import threading
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    server.serve_forever()
    server.server_close()
    print("[IHM] Serveur arrêté.", flush=True)


if __name__ == "__main__":
    main()
