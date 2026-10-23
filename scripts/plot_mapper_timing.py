#!/usr/bin/env python3
"""Plot Mapping timing CSVs. Costs are milliseconds; blank get costs are unsampled.

Usage: python3 scripts/plot_mapper_timing.py octomap.csv [bonxai.csv]
       Add --labels OctoMap Bonxai to name the compared schemes.
Requires matplotlib. Uses a headless backend unless --show is requested.
"""

import argparse
import csv
import math
from pathlib import Path


COLUMNS = ("timestamp", "data_size", "update_cost", "get_map_cost")


def read_timing(path):
    """Return numeric columns, retaining unsampled get costs as NaN."""
    data = {name: [] for name in COLUMNS}
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = set(COLUMNS) - set(reader.fieldnames or [])
        if missing:
            raise ValueError("Missing CSV columns: " + ", ".join(sorted(missing)))
        diagnostics = [name for name in ("valid_points", "candidate_rays", "ray_count", "touched_voxels")
                       if name in reader.fieldnames]
        data.update({name: [] for name in diagnostics})
        for row in reader:
            try:
                timestamp = float(row["timestamp"])
                size = int(row["data_size"])
                update = float(row["update_cost"])
                raw_get = row["get_map_cost"]
                get = float(raw_get) if raw_get and raw_get.strip() else math.nan
                counts = {name: int(row[name]) for name in diagnostics}
                if any(value < 0 for value in counts.values()):
                    raise ValueError("ray/voxel counts must be nonnegative")
                if ("ray_count" in counts and "candidate_rays" in counts and
                        counts["ray_count"] > counts["candidate_rays"]):
                    raise ValueError("ray_count cannot exceed candidate_rays")
                if not math.isfinite(timestamp):
                    raise ValueError("timestamp must be finite")
                if size < 0 or not math.isfinite(update) or update < 0:
                    raise ValueError("data_size and update_cost must be nonnegative")
                if raw_get and raw_get.strip() and (not math.isfinite(get) or get < 0):
                    raise ValueError("get_map_cost must be blank or finite and nonnegative")
                if data["timestamp"] and timestamp < data["timestamp"][-1]:
                    raise ValueError("timestamps must be nondecreasing; use one run per CSV")
            except (ValueError, TypeError) as error:
                raise ValueError(f"CSV line {reader.line_num}: {error}") from error
            for name, value in zip(COLUMNS, (timestamp, size, update, get)):
                data[name].append(value)
            for name, value in counts.items():
                data[name].append(value)
    if not data["timestamp"]:
        raise ValueError("CSV contains no timing samples")
    return data


def draw(datasets, names, output_dir, stem, image_format, dpi, show, time_axis):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = ("#4278a8", "#db8534")
    styles = ("-", "--")
    columns = ["data_size", "update_cost", "get_map_cost"]
    labels = ["Input LiDAR points", "Update cost (ms)", "Get map cost (ms)"]
    if any("ray_count" in data for data in datasets):
        columns.append("ray_count")
        labels.append("Actual ray count")
    fig, axes = plt.subplots(len(columns), 1, figsize=(11, 2.6 * len(columns)), sharex=True,
                             constrained_layout=True)
    for data, name, color, style in zip(datasets, names, colors, styles):
        origin = data["timestamp"][0] if time_axis == "elapsed" else 0
        times = [stamp - origin for stamp in data["timestamp"]]
        get_count = sum(math.isfinite(value) for value in data["get_map_cost"])
        print(f"{name}: {len(times)} updates, {get_count} map extractions, "
              f"sensor duration={data['timestamp'][-1] - data['timestamp'][0]:.3f} s")
        if "candidate_rays" in data and "ray_count" in data:
            candidates, rays = sum(data["candidate_rays"]), sum(data["ray_count"])
            reduction = 100 * (1 - rays / candidates) if candidates else 0
            print(f"{name}: candidate rays={candidates}, actual rays={rays}, "
                  f"ray reduction={reduction:.2f}%")
        for ax, column in zip(axes, columns):
            if column not in data:
                ax.plot([], [], color=color, linestyle=style, label=f"{name} (not recorded)")
                continue
            label = name
            if column == "get_map_cost" and not get_count:
                label += " (no samples)"
            ax.plot(times, data[column], color=color, linestyle=style, linewidth=1,
                    marker=".", markersize=3, label=label)
    for ax, label in zip(axes, labels):
        ax.set_ylabel(label)
        ax.grid(alpha=0.25)
        ax.set_ylim(bottom=0)
        ax.legend(loc="best")
    axes[-1].set_xlabel("Elapsed sensor time (s)" if time_axis == "elapsed"
                        else "Sensor timestamp (s)")
    title = " vs ".join(names)
    fig.suptitle(title)
    time_path = output_dir / f"{stem}_timeseries.{image_format}"
    fig.savefig(time_path, dpi=dpi)

    # Separate metric axes keep the two schemes comparable even when extraction
    # and update times have very different magnitudes.
    box_fig, box_axes = plt.subplots(1, 2, figsize=(10, 5), constrained_layout=True)
    for ax, column, metric in zip(box_axes, ("update_cost", "get_map_cost"),
                                  ("Update", "Get map")):
        for position, (data, name, color) in enumerate(zip(datasets, names, colors), 1):
            values = [value for value in data[column] if math.isfinite(value)]
            if not values:
                ax.text(position, 0.5, "No samples", ha="center",
                        transform=ax.get_xaxis_transform())
                print(f"{name} / {metric}: no samples")
                continue
            boxes = ax.boxplot([values], positions=[position], widths=0.5,
                               patch_artist=True, showmeans=True, whis=1.5,
                               medianprops={"color": "black"})
            boxes["boxes"][0].set_facecolor(color)
            boxes["boxes"][0].set_alpha(0.6)
            ordered = sorted(values)
            count = len(ordered)
            median = (ordered[(count - 1) // 2] + ordered[count // 2]) / 2
            p95 = ordered[math.ceil(0.95 * count) - 1]
            print(f"{name} / {metric}: n={count}, mean={sum(values) / count:.3f} ms, "
                  f"median={median:.3f} ms, p95={p95:.3f} ms")
        counts = [sum(math.isfinite(v) for v in data[column]) for data in datasets]
        ax.set_xticks(range(1, len(names) + 1))
        ax.set_xticklabels([f"{name}\n(n={count})" for name, count in zip(names, counts)])
        ax.set_xlim(0.5, len(names) + 0.5)
        ax.set_ylabel("Wall time (ms)")
        ax.set_ylim(bottom=0)
        ax.set_title(metric)
        ax.grid(axis="y", alpha=0.25)
    box_fig.suptitle(f"{title} | Timing distributions")
    box_path = output_dir / f"{stem}_boxplot.{image_format}"
    box_fig.savefig(box_path, dpi=dpi)

    if len(datasets) == 2:
        first, second = datasets
        if (first["timestamp"] == second["timestamp"] and
                first["data_size"] == second["data_size"]):
            base, alternative = sum(first["update_cost"]), sum(second["update_cost"])
            if base > 0 and alternative > 0:
                print(f"Matching timestamps and point counts: {names[1]} vs {names[0]} "
                      f"update speedup={base / alternative:.3f}x, "
                      f"time reduction={100 * (1 - alternative / base):.2f}%")
        else:
            print("Timestamp/point-count sequences differ; no paired update speedup reported.")
    print(f"Saved {time_path}\nSaved {box_path}")
    if show:
        plt.show()
    plt.close(fig)
    plt.close(box_fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, nargs="+", help="One or two timing CSVs from Mapping")
    parser.add_argument("--labels", nargs="+", help="One scheme name per CSV, in input order")
    parser.add_argument("--time-axis", choices=("elapsed", "timestamp"), default="elapsed",
                        help="Align each run at zero (default), or use original sensor timestamps")
    parser.add_argument("--output-dir", type=Path, help="Default: beside the first CSV")
    parser.add_argument("--format", choices=("png", "pdf", "svg"), default="png")
    parser.add_argument("--dpi", type=int, default=200)
    parser.add_argument("--show", action="store_true", help="Also open interactive plots")
    args = parser.parse_args()
    if len(args.csv) > 2:
        parser.error("provide one or two CSV files")
    if args.labels and (len(args.labels) != len(args.csv) or
                        any(not label.strip() for label in args.labels)):
        parser.error("--labels requires one nonempty name per CSV")
    names = args.labels or [path.stem for path in args.csv]
    if len(names) == 2 and names[0] == names[1]:
        names = [f"{name} ({i})" for i, name in enumerate(names, 1)]
    if args.dpi <= 0:
        parser.error("--dpi must be positive")
    try:
        datasets = []
        for path in args.csv:
            try:
                datasets.append(read_timing(path))
            except (OSError, ValueError) as error:
                raise ValueError(f"{path}: {error}") from error
        output_dir = args.output_dir or args.csv[0].parent
        output_dir.mkdir(parents=True, exist_ok=True)
        stem = "_vs_".join(path.stem for path in args.csv)
        draw(datasets, names, output_dir, stem, args.format, args.dpi, args.show,
             args.time_axis)
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
