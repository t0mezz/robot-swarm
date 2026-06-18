# screen_manager.py
# Display Screen-Manager für den Pololu 3pi+ 2040
# Verwaltet mehrere Debug-Screens und ermöglicht Umschalten per Knopfdruck
# Unterstützt: Text-Log, Primitives, built-in Diagramme, Metriken

import time
import math

MAX_LOG_LINES = 5
DISPLAY_WIDTH = 16        # Zeichen pro Zeile (128px / 8px pro Zeichen)
PREFIX_LEN    = 2         # Länge von "> " bzw. "  "
MAX_CHARS     = DISPLAY_WIDTH - PREFIX_LEN  # 14 Zeichen für Inhalt
REFRESH_MS    = 100       # Display max. alle 100ms neu zeichnen
DIAG_HEIGHT   = 30        # Pixel-Höhe des Diagrammbereichs bei Split
DIAG_SEP      = 2         # Pixel Abstand (Padding) zwischen Diagramm und Log


# ─────────────────────────────────────────────────────────────
# Hilfs-Primitive (Bresenham)
# ─────────────────────────────────────────────────────────────

def _draw_line(display, x0, y0, x1, y1):
    """Linie auf das Display zeichnen – nutzt framebuf C-Implementierung statt Python-Bresenham."""
    display.line(x0, y0, x1, y1, 1)


def _draw_circle(display, cx, cy, r):
    """Bresenham-Kreis direkt auf das Display zeichnen."""
    x, y, err = r, 0, 1 - r
    while x >= y:
        for px, py in [(cx+x,cy+y),(cx-x,cy+y),(cx+x,cy-y),(cx-x,cy-y),
                       (cx+y,cy+x),(cx-y,cy+x),(cx+y,cy-x),(cx-y,cy-x)]:
            display.pixel(px, py, 1)
        y += 1
        if err < 0:
            err += 2 * y + 1
        else:
            x -= 1
            err += 2 * (y - x + 1)


# ─────────────────────────────────────────────────────────────
# Diagramme
# ─────────────────────────────────────────────────────────────

class LineGraph:
    """
    Scrollender Liniengraph für kontinuierliche Werte.

    Verwendung:
        graph = LineGraph(label="vel", max_val=255, history=64)
        screen.set_diagram(graph)
        # In der Loop:
        graph.push(current_speed)
    """
    def __init__(self, label: str, max_val: float = 100, min_val: float = 0,
                 history: int = None, auto_scale: bool = False, padding: float = 0.1):
        self.label      = label
        self.max_val    = max_val
        self.min_val    = min_val
        self.auto_scale = auto_scale
        self.padding    = padding
        self._data      = []
        self._history   = history   # None = wird beim ersten draw() auf inner_w gesetzt

    def push(self, value: float):
        """Fügt einen neuen Messwert hinzu."""
        self._data.append(value)
        if self._history is not None and len(self._data) > self._history:
            del self._data[0]

    def _scale(self):
        """Gibt (min, max) zurück – dynamisch bei auto_scale, sonst fest."""
        if not self.auto_scale or len(self._data) < 2:
            return self.min_val, self.max_val
        lo = min(self._data)
        hi = max(self._data)
        if lo == hi:
            return lo - 1, hi + 1
        pad = (hi - lo) * self.padding
        return lo - pad, hi + pad

    def draw(self, display, x: int, y: int, w: int, h: int):
        lo, hi = self._scale()

        # Feste Breite fuer Min/Max-Beschriftung (4 Zeichen = 32px)
        # Verhindert dass sich inner_w aendert und den Graph abschneidet
        LABEL_W = 32 if h >= 20 else 0

        gx      = x + LABEL_W
        gw      = w - LABEL_W
        inner_w = gw - 2
        inner_h = h - 2

        # History immer auf aktuelle Graphbreite anpassen
        if self._history != inner_w:
            self._history = inner_w
            if len(self._data) > self._history:
                self._data = self._data[-self._history:]

        display.rect(gx, y, gw, h, 1)

        # Min/Max links des Graphen
        if LABEL_W > 0:
            max_str = str(int(hi)) if self.auto_scale else str(int(self.max_val))
            min_str = str(int(lo)) if self.auto_scale else str(int(self.min_val))
            display.text(max_str, x, y + 1)
            display.text(min_str, x, y + h - 9)

        # Label unten rechts
        display.text(self.label, gx + gw - len(self.label) * 8 - 2, y + h - 9)

        if len(self._data) < 2:
            return

        val_range = hi - lo or 1
        # Rechtsbündig zeichnen: neueste Daten immer am rechten Rand
        # Kein Slice – direkt ueber self._data iterieren, um Heap-Allokation zu vermeiden
        n      = len(self._data)
        draw_n = min(n, inner_w)
        start  = n - draw_n
        offset = inner_w - draw_n
        for i in range(1, draw_n):
            x0 = gx + 1 + offset + (i - 1)
            x1 = gx + 1 + offset + i
            y0 = y + h - 1 - int((self._data[start + i - 1] - lo) / val_range * inner_h)
            y1 = y + h - 1 - int((self._data[start + i]     - lo) / val_range * inner_h)
            _draw_line(display, x0, y0, x1, y1)


class BarGraph:
    """
    Balkendiagramm für einen einzelnen Wert.

    Verwendung:
        bar = BarGraph(label="rpm", max_val=300)
        screen.set_diagram(bar)
        bar.set_value(current_rpm)
    """
    def __init__(self, label: str, max_val: float, min_val: float = 0):
        self.label   = label
        self.max_val = max_val
        self.min_val = min_val
        self._value  = 0

    def set_value(self, value: float):
        self._value = max(self.min_val, min(self.max_val, value))

    def draw(self, display, x: int, y: int, w: int, h: int):
        display.rect(x, y, w, h, 1)
        display.text(self.label, x + 1, y + 1)
        val_range = self.max_val - self.min_val or 1
        fill_w = int((self._value - self.min_val) / val_range * (w - 2))
        if fill_w > 0:
            display.fill_rect(x + 1, y + 1, fill_w, h - 2, 1)
        val_str = str(int(self._value))
        display.text(val_str, x + w - len(val_str) * 8 - 1, y + 1)


class Gauge:
    """
    Halbkreis-Gauge für einen einzelnen Wert.

    Verwendung:
        gauge = Gauge(label="temp", min_val=0, max_val=100)
        screen.set_diagram(gauge)
        gauge.set_value(current_temp)
    """
    def __init__(self, label: str, max_val: float, min_val: float = 0):
        self.label   = label
        self.max_val = max_val
        self.min_val = min_val
        self._value  = 0

    def set_value(self, value: float):
        self._value = max(self.min_val, min(self.max_val, value))

    def draw(self, display, x: int, y: int, w: int, h: int):
        cx = x + w // 2
        cy = y + h - 4
        r  = min(w // 2 - 2, h - 6)
        # Halbkreis-Bogen (links nach rechts)
        steps = 40
        for i in range(steps):
            a0 = math.pi - (math.pi / steps * i)
            a1 = math.pi - (math.pi / steps * (i + 1))
            _draw_line(display,
                       int(cx + r * math.cos(a0)), int(cy - r * math.sin(a0)),
                       int(cx + r * math.cos(a1)), int(cy - r * math.sin(a1)))
        # Zeiger
        val_range = self.max_val - self.min_val or 1
        angle = math.pi - ((self._value - self.min_val) / val_range * math.pi)
        needle_r = r - 3
        _draw_line(display, cx, cy,
                   int(cx + needle_r * math.cos(angle)),
                   int(cy - needle_r * math.sin(angle)))
        # Label + Wert
        display.text(self.label, x + 1, y + 1)
        val_str = str(int(self._value))
        display.text(val_str, cx - len(val_str) * 4, cy - r // 2)


# ─────────────────────────────────────────────────────────────
# Screen
# ─────────────────────────────────────────────────────────────

class Screen:
    """
    Repräsentiert einen einzelnen Debug-Screen.

    Verwendung:
        screen = Screen("MY APP")

        # Text-Log
        screen.log("Motor gestartet")

        # Primitives (persistent, einzeln oder alle entfernbar)
        h = screen.draw_line(0, 10, 64, 10)
        screen.draw_circle(64, 32, 10)
        screen.remove_primitive(h)   # einzelne entfernen
        screen.clear_primitives()    # alle entfernen

        # Metriken (feste Position, überlappen immer)
        screen.set_metric("vel",  lambda: speed, x=0,  y=56)
        screen.set_metric("temp", lambda: temp,  x=64, y=56)
        screen.remove_metric("vel")

        # Diagramm (teilt Display: oben=Diagramm, unten=Log)
        graph = LineGraph("vel", max_val=255)
        screen.set_diagram(graph)
        screen.clear_diagram()
    """

    def __init__(self, title: str):
        self.title       = title[:DISPLAY_WIDTH]
        self._lines      = []
        self._primitives = []   # [(id, callable), ...]
        self._prim_id    = 0
        self._metrics    = {}   # name -> (callable, x, y)
        self._diagrams   = []    # [(diagram, height), ...]

    # ── Text-Log ──────────────────────────────────────────────

    def log(self, msg: str):
        """Fügt eine neue Nachricht zum Log hinzu."""
        self._lines.append(str(msg))
        if len(self._lines) > MAX_LOG_LINES * 3:
            del self._lines[:-MAX_LOG_LINES]

    def clear(self):
        """Löscht alle Log-Einträge."""
        self._lines.clear()

    # ── Primitives ────────────────────────────────────────────

    def draw_line(self, x0: int, y0: int, x1: int, y1: int) -> int:
        """Persistente Linie. Gibt Handle zurück."""
        return self._add_primitive(lambda d: _draw_line(d, x0, y0, x1, y1))

    def draw_circle(self, cx: int, cy: int, r: int) -> int:
        """Persistenter Kreis. Gibt Handle zurück."""
        return self._add_primitive(lambda d: _draw_circle(d, cx, cy, r))

    def draw_rect(self, x: int, y: int, w: int, h: int) -> int:
        """Persistentes Rechteck. Gibt Handle zurück."""
        return self._add_primitive(lambda d: d.rect(x, y, w, h, 1))

    def draw_pixel(self, x: int, y: int) -> int:
        """Persistentes Pixel. Gibt Handle zurück."""
        return self._add_primitive(lambda d: d.pixel(x, y, 1))

    def remove_primitive(self, handle: int):
        """Entfernt eine einzelne Primitive per Handle."""
        self._primitives = [(i, fn) for i, fn in self._primitives if i != handle]

    def clear_primitives(self):
        """Entfernt alle Primitives."""
        self._primitives.clear()

    def _add_primitive(self, fn) -> int:
        handle = self._prim_id
        self._primitives.append((handle, fn))
        self._prim_id += 1
        return handle

    # ── Metriken ──────────────────────────────────────────────

    def set_metric(self, name: str, value_fn, x: int, y: int):
        """
        Registriert eine Metrik an fester Position (überlappen immer).

        :param name:     Label (z.B. "vel")
        :param value_fn: Callable ohne Argumente → aktueller Wert
        :param x:        X-Position in Pixeln
        :param y:        Y-Position in Pixeln
        """
        self._metrics[name] = (value_fn, x, y)

    def remove_metric(self, name: str):
        """Entfernt eine Metrik."""
        self._metrics.pop(name, None)

    # ── Diagramm ──────────────────────────────────────────────

    def set_diagram(self, diagram, height: int = None):
        """Setzt ein einzelnes Diagramm (ersetzt alle vorherigen).
        height: Pixel-Hoehe, None = DIAG_HEIGHT Standard."""
        self._diagrams = [(diagram, height or DIAG_HEIGHT)]

    def add_diagram(self, diagram, height: int = None):
        """Fuegt ein weiteres Diagramm unterhalb der vorherigen hinzu.
        height: Pixel-Hoehe, None = DIAG_HEIGHT Standard."""
        self._diagrams.append((diagram, height or DIAG_HEIGHT))

    def clear_diagram(self):
        """Entfernt alle Diagramme, kehrt zu Vollbild-Log zurueck."""
        self._diagrams = []

    # ── Internes Rendering ────────────────────────────────────

    def _render_lines(self, max_lines: int) -> list:
        lines = []
        for msg in self._lines[-MAX_LOG_LINES:]:
            lines.append('> ' + msg[:MAX_CHARS])
            rest = msg[MAX_CHARS:]
            while rest:
                lines.append('  ' + rest[:MAX_CHARS])
                rest = rest[MAX_CHARS:]
        return lines[-max_lines:]

    def _draw_onto(self, display, active_idx: int, total_screens: int):
        display.fill(0)

        if self._diagrams:
            # Mehrere Diagramme: gleichmaessig auf volle Hoehe verteilen
            # Log und Metriken werden nicht angezeigt
            if len(self._diagrams) > 1:
                total_sep = DIAG_SEP * (len(self._diagrams) - 1)
                h_each    = (64 - total_sep) // len(self._diagrams)
                cur_y     = 0
                for diagram, _ in self._diagrams:
                    diagram.draw(display, x=0, y=cur_y, w=128, h=h_each)
                    cur_y += h_each + DIAG_SEP
            else:
                # Ein Diagramm: Split-Modus mit Log
                diagram, h = self._diagrams[0]
                diagram.draw(display, x=0, y=0, w=128, h=h)
                log_y     = h + DIAG_SEP
                max_lines = (64 - log_y) // 10
                for i, line in enumerate(self._render_lines(max_lines)):
                    display.text(line, 0, log_y + i * 10)
        else:
            # Vollbild-Modus: Titel + Log
            lines      = self._render_lines(6)
            show_title = len(lines) <= 5
            if show_title:
                idx_str = f"{active_idx + 1}/{total_screens}"
                display.text(self.title, 0, 0)
                display.text(idx_str, 128 - len(idx_str) * 8, 0)
                log_y     = 10
                max_lines = 5
            else:
                log_y     = 0
                max_lines = 6
            for i, line in enumerate(lines[-max_lines:]):
                display.text(line, 0, log_y + i * 10)

        # Primitives (freie Platzierung, kein Split)
        for _, fn in self._primitives:
            fn(display)

        # Metriken (nur wenn weniger als 2 Diagramme aktiv)
        if len(self._diagrams) < 2:
            for name, (value_fn, x, y) in self._metrics.items():
                try:
                    val = value_fn()
                except Exception:
                    val = "err"
                display.text(f"{name}:{val}", x, y)

        display.show()


# ─────────────────────────────────────────────────────────────
# ScreenManager
# ─────────────────────────────────────────────────────────────

class ScreenManager:
    """
    Verwaltet mehrere Screens und zeichnet den aktiven auf das Display.
    Knopfdruck schaltet zwischen registrierten Screens um.

    Verwendung:
        from pololu_3pi_2040_robot import robot
        mgr = ScreenManager(display=robot.Display(), button=robot.ButtonB())

        uart_screen = Screen("UART LOG")
        app_screen  = Screen("MY APP")
        mgr.register(uart_screen)
        mgr.register(app_screen)

        while True:
            proto.loop()
            mgr.loop()
    """

    def __init__(self, display, button=None):
        """
        :param display: robot.Display() Instanz
        :param button:  Button-Instanz mit .check() Methode (optional)
        """
        self._display      = display
        self._button       = button
        self._screens      = []
        self._active_idx   = 0
        self._dirty        = True
        self._last_refresh = 0

    def register(self, screen: Screen):
        """Registriert einen Screen. Erster registrierter Screen ist aktiv."""
        self._screens.append(screen)
        self._dirty = True

    def set_active(self, index: int):
        """Setzt den aktiven Screen direkt per Index."""
        if 0 <= index < len(self._screens):
            self._active_idx = index
            self._dirty = True

    def next_screen(self):
        """Schaltet zum nächsten Screen weiter (wraps around)."""
        if self._screens:
            self._active_idx = (self._active_idx + 1) % len(self._screens)
            self._dirty = True

    @property
    def active_screen(self) -> Screen:
        """Gibt den aktuell aktiven Screen zurück."""
        return self._screens[self._active_idx] if self._screens else None

    def mark_dirty(self):
        """Markiert das Display als neu zu zeichnen (z.B. nach screen.log())."""
        self._dirty = True

    def loop(self):
        """Muss regelmäßig in der Hauptschleife aufgerufen werden."""
        self._handle_button()
        self._refresh_display()

    def _handle_button(self):
        """Entprellter Knopfdruck via check()."""
        if self._button is None:
            return
        if self._button.check():
            self.next_screen()

    def _refresh_display(self):
        """Zeichnet den aktiven Screen (max. alle REFRESH_MS ms)."""
        if not self._dirty or not self._screens:
            return
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_refresh) < REFRESH_MS:
            return
        self._screens[self._active_idx]._draw_onto(
            self._display, self._active_idx, len(self._screens)
        )
        self._last_refresh = now
        self._dirty = False