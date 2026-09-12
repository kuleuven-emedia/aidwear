from pathlib import Path
import shutil
import threading
import time
from typing import Optional
import yaml

from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler




class ConfigManager:
    def __init__(self, config_path: Optional[str], output_dir: str):
        self.output_dir = Path(output_dir).resolve()

        if config_path:
            self.path = Path(config_path).resolve()
        else:
            config = Path("run/fsm_config_default.yml").resolve()
            self.path = config.parent / "fsm_config_temp.yml"
            shutil.copy2(config, self.path)

        # Shared parameter storage
        self.params = {}

        # Thread lock
        self.lock = threading.Lock()

        # Debounce timer
        self.last_reload_time = 0
        self.reload_delay = 0.5  # seconds

        # Initial load
        self._load_config()

        # Start watchdog observer
        self._start_watcher()

    def _load_config(self):
        try:
            with open(self.path, "r") as file:
                new_params = yaml.safe_load(file)

            if new_params is None:
                print("WARNING: Empty config file")
                return

            with self.lock:
                self.params = new_params
            print(f"[ConfigManager] Config reloaded from {self.path}")

        except Exception as e:
            print(f"[ConfigManager] Failed to load config: {e}")

    def get_section(self, section, default=None):
        with self.lock:
            return dict(self.params.get(section, default or {}))

    def get(self, *keys, default=None):
        with self.lock:
            ref = self.params
            for key in keys:
                if not isinstance(ref, dict):
                    return default

                if key not in ref:
                    return default

                ref = ref[key]
            return ref

    def _start_watcher(self):
        class ConfigHandler(FileSystemEventHandler):
            def __init__(self, outer):
                self.outer = outer

            def on_modified(self, event):
                event_path = Path(event.src_path).resolve()

                if event_path != self.outer.path:
                    return

                now = time.time()
                if now - self.outer.last_reload_time < self.outer.reload_delay:
                    return
                self.outer.last_reload_time = now
                self.outer._load_config()

        self.observer = Observer()
        handler = ConfigHandler(self)
        self.observer.schedule(handler, path=str(self.path.parent), recursive=False)
        self.observer.start()

    def stop(self):
        self.observer.stop()
        shutil.copy2(self.path, self.output_dir / "fsm_config.yml")
        self.observer.join()
