#!/usr/bin/env python3
"""Desktop GUI for the trajectory generator.

The generation backend stays in ``gen_path.py``. This file only handles user
interaction: selecting YAML files, editing generation parameters and previewing
the input/output paths.
"""

from __future__ import annotations

import math
import os
import tkinter as tk
from dataclasses import dataclass
from tkinter import filedialog, messagebox, ttk
from typing import List, Optional, Sequence, Tuple

from trajectory_generator import (
    Point2D,
    TrajectoryConfig,
    generate_trajectory_yaml,
    read_input_points,
    visualize_trajectory,
)


APP_TITLE = "Generateur de trajectoire"
DEFAULT_OUTPUT = "trajectory_generated.yaml"


@dataclass
class PathSummary:
    count: int
    length: float


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _path_summary(points: Sequence[Point2D]) -> PathSummary:
    if not points:
        return PathSummary(0, 0.0)
    return PathSummary(
        count=len(points),
        length=sum(_distance(points[idx - 1], points[idx]) for idx in range(1, len(points))),
    )


def _read_points_from_generated_data(data: dict, list_key: str, x_key: str, y_key: str) -> List[Point2D]:
    raw_points = data.get(list_key)
    if not isinstance(raw_points, list):
        raise ValueError(f"La sortie ne contient pas la liste '{list_key}'")

    points: List[Point2D] = []
    for idx, item in enumerate(raw_points, start=1):
        if not isinstance(item, dict) or x_key not in item or y_key not in item:
            raise ValueError(f"Point de sortie #{idx} invalide")
        points.append((float(item[x_key]), float(item[y_key])))
    return points


class PathCanvas(tk.Canvas):
    def __init__(self, master: tk.Widget) -> None:
        super().__init__(master, background="#f7f8fa", highlightthickness=0)
        self.input_points: List[Point2D] = []
        self.output_points: List[Point2D] = []
        self.map_like = True
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_paths(
        self,
        input_points: Sequence[Point2D],
        output_points: Sequence[Point2D],
        *,
        map_like: bool,
    ) -> None:
        self.input_points = list(input_points)
        self.output_points = list(output_points)
        self.map_like = map_like
        self.redraw()

    def _screen_points(self, points: Sequence[Point2D], bounds: Tuple[float, float, float, float]) -> List[Point2D]:
        width = max(1, self.winfo_width())
        height = max(1, self.winfo_height())
        pad = 38
        min_x, max_x, min_y, max_y = bounds
        span_x = max(max_x - min_x, 1e-12)
        span_y = max(max_y - min_y, 1e-12)
        usable_w = max(1, width - 2 * pad)
        usable_h = max(1, height - 2 * pad)
        scale = min(usable_w / span_x, usable_h / span_y)
        draw_w = span_x * scale
        draw_h = span_y * scale
        offset_x = (width - draw_w) / 2.0
        offset_y = (height - draw_h) / 2.0

        out: List[Point2D] = []
        for px, py in points:
            x = offset_x + (px - min_x) * scale
            y = offset_y + draw_h - (py - min_y) * scale
            out.append((x, y))
        return out

    def _display_points(self, points: Sequence[Point2D]) -> List[Point2D]:
        if self.map_like:
            return [(lon, lat) for lat, lon in points]
        return list(points)

    def _draw_polyline(self, points: Sequence[Point2D], color: str, width: int, dash: Optional[Tuple[int, ...]] = None) -> None:
        if len(points) < 2:
            return
        flat = [coord for point in points for coord in point]
        self.create_line(*flat, fill=color, width=width, dash=dash, capstyle=tk.ROUND, joinstyle=tk.ROUND)

    def _draw_points(self, points: Sequence[Point2D], color: str, radius: int, with_labels: bool) -> None:
        for idx, (x, y) in enumerate(points, start=1):
            self.create_oval(x - radius, y - radius, x + radius, y + radius, fill=color, outline="white", width=2)
            if with_labels:
                self.create_text(x + 9, y - 9, text=str(idx), fill="#20242a", anchor="w", font=("TkDefaultFont", 9, "bold"))

    def redraw(self) -> None:
        self.delete("all")
        if not self.input_points and not self.output_points:
            self.create_text(
                self.winfo_width() / 2,
                self.winfo_height() / 2,
                text="Chargez un YAML pour previsualiser le chemin",
                fill="#6b7280",
                font=("TkDefaultFont", 12),
            )
            return

        displayed_input = self._display_points(self.input_points)
        displayed_output = self._display_points(self.output_points)
        all_points = displayed_input + displayed_output
        min_x = min(point[0] for point in all_points)
        max_x = max(point[0] for point in all_points)
        min_y = min(point[1] for point in all_points)
        max_y = max(point[1] for point in all_points)
        bounds = (min_x, max_x, min_y, max_y)

        input_screen = self._screen_points(displayed_input, bounds)
        output_screen = self._screen_points(displayed_output, bounds)
        self._draw_polyline(output_screen, "#d53131", 3)
        self._draw_polyline(input_screen, "#23272f", 2, dash=(5, 4))
        self._draw_points(input_screen, "#23272f", 5, True)

        if output_screen:
            self._draw_points([output_screen[0]], "#16803c", 6, False)
            self._draw_points([output_screen[-1]], "#7c3aed", 6, False)

        self.create_rectangle(14, 14, 255, 72, fill="#ffffff", outline="#d7dce2")
        self.create_line(28, 34, 68, 34, fill="#23272f", width=2, dash=(5, 4))
        self.create_text(78, 34, text="points d'entree", anchor="w", fill="#20242a")
        self.create_line(28, 55, 68, 55, fill="#d53131", width=3)
        self.create_text(78, 55, text="trajectoire generee", anchor="w", fill="#20242a")


class PathGeneratorApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1180x760")
        self.minsize(980, 640)

        self.input_points: List[Point2D] = []
        self.output_points: List[Point2D] = []
        self.last_output_data: Optional[dict] = None

        self.input_path = tk.StringVar()
        self.output_path = tk.StringVar(value=os.path.abspath(DEFAULT_OUTPUT))
        self.step = tk.StringVar(value="0.10")
        self.turn_radius = tk.StringVar(value="1.50")
        self.input_list_key = tk.StringVar(value="waypoints")
        self.output_list_key = tk.StringVar(value="trajectory")
        self.x_key = tk.StringVar(value="latitude")
        self.y_key = tk.StringVar(value="longitude")
        self.yaw_key = tk.StringVar(value="yaw")
        self.origin_lat = tk.StringVar()
        self.origin_lon = tk.StringVar()
        self.earth_radius = tk.StringVar(value="6378137.0")
        self.gnss_to_local = tk.BooleanVar(value=True)
        self.include_yaw = tk.BooleanVar(value=True)
        self.status = tk.StringVar(value="Pret.")
        self.input_summary = tk.StringVar(value="Entree: aucun point")
        self.output_summary = tk.StringVar(value="Sortie: non generee")

        self._configure_style()
        self._build_ui()

    def _configure_style(self) -> None:
        style = ttk.Style()
        style.configure("TFrame", background="#f0f2f5")
        style.configure("Panel.TFrame", background="#ffffff", relief="flat")
        style.configure("Title.TLabel", background="#ffffff", foreground="#1f2937", font=("TkDefaultFont", 12, "bold"))
        style.configure("Muted.TLabel", background="#ffffff", foreground="#5f6876")
        style.configure("Status.TLabel", background="#e9edf2", foreground="#334155")
        style.configure("Primary.TButton", padding=(12, 8))
        style.configure("Tool.TButton", padding=(9, 7))

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        left = ttk.Frame(self, style="Panel.TFrame", padding=16)
        left.grid(row=0, column=0, sticky="nsw")
        left.columnconfigure(1, weight=1)

        ttk.Label(left, text="Configuration", style="Title.TLabel").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 12))

        row = 1
        row = self._file_row(left, row, "YAML entree", self.input_path, self._choose_input)
        row = self._file_row(left, row, "YAML sortie", self.output_path, self._choose_output)

        ttk.Separator(left).grid(row=row, column=0, columnspan=3, sticky="ew", pady=14)
        row += 1

        row = self._entry_row(left, row, "Pas (m)", self.step)
        row = self._entry_row(left, row, "Rayon virage (m)", self.turn_radius)
        row = self._entry_row(left, row, "Liste entree", self.input_list_key)
        row = self._entry_row(left, row, "Liste sortie", self.output_list_key)
        row = self._entry_row(left, row, "Cle X / latitude", self.x_key)
        row = self._entry_row(left, row, "Cle Y / longitude", self.y_key)
        row = self._entry_row(left, row, "Cle yaw", self.yaw_key)

        ttk.Checkbutton(left, text="Conversion GNSS vers local", variable=self.gnss_to_local).grid(
            row=row, column=0, columnspan=3, sticky="w", pady=(8, 2)
        )
        row += 1
        ttk.Checkbutton(left, text="Inclure yaw dans la sortie", variable=self.include_yaw).grid(
            row=row, column=0, columnspan=3, sticky="w", pady=2
        )
        row += 1

        ttk.Separator(left).grid(row=row, column=0, columnspan=3, sticky="ew", pady=14)
        row += 1

        ttk.Label(left, text="Origine optionnelle", style="Muted.TLabel").grid(row=row, column=0, columnspan=3, sticky="w")
        row += 1
        row = self._entry_row(left, row, "Latitude origine", self.origin_lat)
        row = self._entry_row(left, row, "Longitude origine", self.origin_lon)
        row = self._entry_row(left, row, "Rayon Terre (m)", self.earth_radius)

        actions = ttk.Frame(left, style="Panel.TFrame")
        actions.grid(row=row, column=0, columnspan=3, sticky="ew", pady=(16, 8))
        actions.columnconfigure(0, weight=1)
        actions.columnconfigure(1, weight=1)
        ttk.Button(actions, text="Generer", style="Primary.TButton", command=self._generate).grid(row=0, column=0, sticky="ew", padx=(0, 6))
        ttk.Button(actions, text="Exporter PNG", style="Tool.TButton", command=self._export_png).grid(row=0, column=1, sticky="ew", padx=(6, 0))
        row += 1

        ttk.Button(left, text="Recharger l'entree", style="Tool.TButton", command=self._load_input).grid(
            row=row, column=0, columnspan=3, sticky="ew", pady=(0, 12)
        )
        row += 1

        ttk.Label(left, textvariable=self.input_summary, style="Muted.TLabel", wraplength=330).grid(row=row, column=0, columnspan=3, sticky="w", pady=2)
        row += 1
        ttk.Label(left, textvariable=self.output_summary, style="Muted.TLabel", wraplength=330).grid(row=row, column=0, columnspan=3, sticky="w", pady=2)
        row += 1

        ttk.Label(left, textvariable=self.status, style="Status.TLabel", wraplength=330, padding=8).grid(
            row=row, column=0, columnspan=3, sticky="ew", pady=(12, 0)
        )

        right = ttk.Frame(self, padding=(14, 14, 14, 14))
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)

        header = ttk.Frame(right)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="Visualisation", font=("TkDefaultFont", 15, "bold"), background="#f0f2f5").grid(row=0, column=0, sticky="w")
        ttk.Label(header, text="Noir: entree, rouge: sortie", background="#f0f2f5", foreground="#586174").grid(row=0, column=1, sticky="e")

        self.canvas = PathCanvas(right)
        self.canvas.grid(row=1, column=0, sticky="nsew")

    def _entry_row(self, parent: ttk.Frame, row: int, label: str, variable: tk.StringVar) -> int:
        ttk.Label(parent, text=label, style="Muted.TLabel").grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(parent, textvariable=variable, width=20).grid(row=row, column=1, columnspan=2, sticky="ew", pady=4)
        return row + 1

    def _file_row(self, parent: ttk.Frame, row: int, label: str, variable: tk.StringVar, command) -> int:
        ttk.Label(parent, text=label, style="Muted.TLabel").grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(parent, textvariable=variable, width=24).grid(row=row, column=1, sticky="ew", pady=4, padx=(0, 6))
        ttk.Button(parent, text="...", width=4, command=command).grid(row=row, column=2, sticky="e", pady=4)
        return row + 1

    def _choose_input(self) -> None:
        path = filedialog.askopenfilename(
            title="Choisir un YAML d'entree",
            filetypes=[("YAML", "*.yaml *.yml"), ("Tous les fichiers", "*.*")],
        )
        if path:
            self.input_path.set(path)
            if self.output_path.get() == os.path.abspath(DEFAULT_OUTPUT):
                root, _ext = os.path.splitext(path)
                self.output_path.set(f"{root}_generated.yaml")
            self._load_input()

    def _choose_output(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Choisir le YAML de sortie",
            defaultextension=".yaml",
            filetypes=[("YAML", "*.yaml *.yml"), ("Tous les fichiers", "*.*")],
        )
        if path:
            self.output_path.set(path)

    def _float_from_var(self, variable: tk.StringVar, label: str, *, required: bool = True) -> Optional[float]:
        raw = variable.get().strip()
        if not raw:
            if required:
                raise ValueError(f"'{label}' est obligatoire")
            return None
        return float(raw)

    def _config(self) -> TrajectoryConfig:
        return TrajectoryConfig(
            step=self._float_from_var(self.step, "Pas") or 0.0,
            turn_radius=self._float_from_var(self.turn_radius, "Rayon virage") or 0.0,
            list_key_in=self.input_list_key.get().strip() or "waypoints",
            list_key_out=self.output_list_key.get().strip() or "trajectory",
            x_key=self.x_key.get().strip() or "latitude",
            y_key=self.y_key.get().strip() or "longitude",
            yaw_key=self.yaw_key.get().strip() or "yaw",
            include_yaw=self.include_yaw.get(),
            gnss_to_local=self.gnss_to_local.get(),
            origin_lat=self._float_from_var(self.origin_lat, "Latitude origine", required=False),
            origin_lon=self._float_from_var(self.origin_lon, "Longitude origine", required=False),
            earth_radius_m=self._float_from_var(self.earth_radius, "Rayon Terre") or 6378137.0,
        )

    def _load_input(self) -> None:
        try:
            path = self.input_path.get().strip()
            if not path:
                raise ValueError("Choisissez un fichier YAML d'entree")
            config = self._config()
            self.input_points = read_input_points(path, config.list_key_in, config.x_key, config.y_key)
            self.output_points = []
            self.last_output_data = None
            self._update_summaries()
            self._redraw()
            self.status.set(f"Entree chargee: {path}")
        except Exception as exc:
            self.status.set(f"Erreur: {exc}")
            messagebox.showerror(APP_TITLE, str(exc))

    def _generate(self) -> None:
        try:
            input_path = self.input_path.get().strip()
            output_path = self.output_path.get().strip()
            if not input_path:
                raise ValueError("Choisissez un fichier YAML d'entree")
            if not output_path:
                raise ValueError("Choisissez un fichier YAML de sortie")

            config = self._config()
            self.input_points = read_input_points(input_path, config.list_key_in, config.x_key, config.y_key)
            self.last_output_data = generate_trajectory_yaml(input_path, output_path, config)
            self.output_points = _read_points_from_generated_data(
                self.last_output_data,
                config.list_key_out,
                config.x_key,
                config.y_key,
            )
            self._update_summaries()
            self._redraw()
            self.status.set(f"Trajectoire generee: {output_path}")
        except Exception as exc:
            self.status.set(f"Erreur: {exc}")
            messagebox.showerror(APP_TITLE, str(exc))

    def _export_png(self) -> None:
        try:
            if not self.input_points or not self.output_points:
                raise ValueError("Generez une trajectoire avant d'exporter le PNG")
            output_yaml = self.output_path.get().strip() or DEFAULT_OUTPUT
            default_png = os.path.splitext(output_yaml)[0] + ".png"
            png_path = filedialog.asksaveasfilename(
                title="Exporter la visualisation PNG",
                initialfile=os.path.basename(default_png),
                initialdir=os.path.dirname(default_png) or os.getcwd(),
                defaultextension=".png",
                filetypes=[("PNG", "*.png"), ("Tous les fichiers", "*.*")],
            )
            if not png_path:
                return
            config = self._config()
            visualize_trajectory(
                input_points=self.input_points,
                trajectory_points=self.output_points,
                output_png_path=png_path,
                x_key=config.x_key,
                y_key=config.y_key,
                title="Dubins turns-only trajectory",
            )
            self.status.set(f"PNG exporte: {png_path}")
        except Exception as exc:
            self.status.set(f"Erreur: {exc}")
            messagebox.showerror(APP_TITLE, str(exc))

    def _redraw(self) -> None:
        map_like = self.x_key.get().strip().lower() == "latitude" and self.y_key.get().strip().lower() == "longitude"
        self.canvas.set_paths(self.input_points, self.output_points, map_like=map_like)

    def _update_summaries(self) -> None:
        input_summary = _path_summary(self.input_points)
        output_summary = _path_summary(self.output_points)
        self.input_summary.set(f"Entree: {input_summary.count} points, longueur approx. {input_summary.length:.3f}")
        if output_summary.count:
            self.output_summary.set(f"Sortie: {output_summary.count} points, longueur approx. {output_summary.length:.3f}")
        else:
            self.output_summary.set("Sortie: non generee")


def main() -> None:
    app = PathGeneratorApp()
    app.mainloop()


if __name__ == "__main__":
    main()
