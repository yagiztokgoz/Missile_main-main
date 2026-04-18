import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from matplotlib.ticker import AutoMinorLocator
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def load_log(filepath: str) -> pd.DataFrame:
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"Dosya bulunamadı: {filepath}")

    df = pd.read_csv(path)
    df.columns = [c.strip() for c in df.columns]

    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    if "t" not in df.columns:
        raise ValueError("Log dosyasında 't' kolonu bulunamadı.")

    return df


def style_axis(ax, title: str, xlabel: str = "Time [s]", ylabel: str = ""):
    ax.set_title(title, fontsize=13, pad=10, fontweight="bold")
    ax.set_xlabel(xlabel, fontsize=10)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=10)

    ax.minorticks_on()
    ax.grid(True, which="major", linewidth=0.8, alpha=0.35)
    ax.grid(True, which="minor", linewidth=0.5, alpha=0.18)
    ax.xaxis.set_minor_locator(AutoMinorLocator())
    ax.yaxis.set_minor_locator(AutoMinorLocator())
    ax.tick_params(labelsize=9)


def style_3d_axis(ax, title: str, xlabel: str, ylabel: str, zlabel: str):
    ax.set_title(title, fontsize=13, pad=14, fontweight="bold")
    ax.set_xlabel(xlabel, fontsize=10, labelpad=8)
    ax.set_ylabel(ylabel, fontsize=10, labelpad=8)
    ax.set_zlabel(zlabel, fontsize=10, labelpad=8)
    ax.tick_params(labelsize=9)
    ax.grid(True)


def plot_series(ax, df: pd.DataFrame, cols, labels=None, linewidth=1.8):
    existing = [c for c in cols if c in df.columns]
    if not existing:
        ax.set_visible(False)
        return False

    if labels is None:
        labels = existing

    for c, lbl in zip(existing, labels):
        ax.plot(df["t"], df[c], label=lbl, linewidth=linewidth)

    if len(existing) > 1:
        ax.legend(fontsize=9, loc="best")
    return True


def finalize_figure(fig, title: str):
    fig.suptitle(title, fontsize=18, fontweight="bold", y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.96])


def make_states_figure(df: pd.DataFrame):
    fig, axes = plt.subplots(3, 2, figsize=(15, 10.5))
    axes = axes.ravel()

    plot_series(axes[0], df, ["dvbe"], ["Speed"])
    style_axis(axes[0], "Speed", ylabel="m/s")

    plot_series(axes[1], df, ["mach"], ["Mach"])
    style_axis(axes[1], "Mach Number", ylabel="-")

    plot_series(axes[2], df, ["alphax", "betax"], ["Alpha", "Beta"])
    style_axis(axes[2], "Aerodynamic Angles", ylabel="deg")

    plot_series(axes[3], df, ["ppx", "qqx", "rrx"], ["p", "q", "r"])
    style_axis(axes[3], "Body Rates", ylabel="rad/s")

    plot_series(axes[4], df, ["mass", "xcg"], ["Mass", "CG"])
    style_axis(axes[4], "Mass and Center of Gravity")

    plot_series(axes[5], df, ["pdynmc", "thrust"], ["Dynamic Pressure", "Thrust"])
    style_axis(axes[5], "Dynamic Pressure and Thrust")

    finalize_figure(fig, "Missile States")


def make_control_figure(df: pd.DataFrame):
    fig, axes = plt.subplots(2, 2, figsize=(15, 8.5))
    axes = axes.ravel()

    plot_series(axes[0], df, ["dpcx", "dqcx", "drcx"], ["Roll Cmd", "Pitch Cmd", "Yaw Cmd"])
    style_axis(axes[0], "Commanded Control Deflections", ylabel="deg")

    plot_series(axes[1], df, ["dpx", "dqx", "drx"], ["Roll Act", "Pitch Act", "Yaw Act"])
    style_axis(axes[1], "Actual Control Deflections", ylabel="deg")

    plot_series(axes[2], df, ["ancomx", "alcomx"], ["Normal Accel Cmd", "Lateral Accel Cmd"])
    style_axis(axes[2], "Guidance Acceleration Commands", ylabel="g")

    plot_series(axes[3], df, ["anx", "ayx"], ["Normal Accel", "Lateral Accel"])
    style_axis(axes[3], "Acceleration Response", ylabel="g")

    finalize_figure(fig, "Control Commands and Responses")


def make_guidance_figure(df: pd.DataFrame):
    fig, axes = plt.subplots(2, 2, figsize=(15, 8.5))
    axes = axes.ravel()

    plot_series(axes[0], df, ["dtbc"], ["Distance to Target"])
    style_axis(axes[0], "Distance to Target", ylabel="m")

    plot_series(axes[1], df, ["tgoc"], ["Time-to-Go"])
    style_axis(axes[1], "Time-to-Go", ylabel="s")

    plot_series(axes[2], df, ["psivlx", "thtvlx"], ["Velocity Azimuth", "Velocity Elevation"])
    style_axis(axes[2], "Velocity Direction Angles", ylabel="deg")

    plot_series(axes[3], df, ["psiblx", "thtblx", "phiblx"], ["Yaw", "Pitch", "Roll"])
    style_axis(axes[3], "Body Euler Angles", ylabel="deg")

    finalize_figure(fig, "Guidance and Geometry")


def make_tracking_figure(df: pd.DataFrame):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.5))
    axes = axes.ravel()

    ok1 = plot_series(
        axes[0], df,
        ["ancomx", "anx"],
        ["Normal Acceleration Command", "Normal Acceleration Response"]
    )
    if ok1:
        style_axis(axes[0], "Normal Acceleration Tracking", ylabel="g")

    ok2 = plot_series(
        axes[1], df,
        ["alcomx", "ayx"],
        ["Lateral Acceleration Command", "Lateral Acceleration Response"]
    )
    if ok2:
        style_axis(axes[1], "Lateral Acceleration Tracking", ylabel="g")

    finalize_figure(fig, "Command Tracking")


def make_trajectory_figure(df: pd.DataFrame):
    required = ["sbel1", "sbel2", "hbe"]
    if not all(c in df.columns for c in required):
        print("Trajectory için gerekli kolonlar yok: sbel1, sbel2, hbe")
        return

    fig = plt.figure(figsize=(18, 6))

    ax1 = fig.add_subplot(1, 3, 1)
    ax1.plot(df["sbel1"], df["hbe"], linewidth=2.2, label="Missile Path")
    ax1.scatter(df["sbel1"].iloc[0], df["hbe"].iloc[0], s=70, marker="o", label="Start", zorder=3)
    ax1.scatter(df["sbel1"].iloc[-1], df["hbe"].iloc[-1], s=70, marker="s", label="End", zorder=3)
    style_axis(ax1, "Trajectory - Side View", xlabel="Downrange [m]", ylabel="Altitude [m]")
    ax1.legend(fontsize=9, loc="best")

    ax2 = fig.add_subplot(1, 3, 2)
    ax2.plot(df["sbel1"], df["sbel2"], linewidth=2.2, label="Missile Path")
    ax2.scatter(df["sbel1"].iloc[0], df["sbel2"].iloc[0], s=70, marker="o", label="Start", zorder=3)
    ax2.scatter(df["sbel1"].iloc[-1], df["sbel2"].iloc[-1], s=70, marker="s", label="End", zorder=3)
    style_axis(ax2, "Trajectory - Top View", xlabel="Downrange [m]", ylabel="Crossrange [m]")
    ax2.legend(fontsize=9, loc="best")
    ax2.axis("equal")

    ax3 = fig.add_subplot(1, 3, 3, projection="3d")
    ax3.plot(df["sbel1"], df["sbel2"], df["hbe"], linewidth=2.2, label="Missile Path")
    ax3.scatter(df["sbel1"].iloc[0], df["sbel2"].iloc[0], df["hbe"].iloc[0], s=55, marker="o", label="Start")
    ax3.scatter(df["sbel1"].iloc[-1], df["sbel2"].iloc[-1], df["hbe"].iloc[-1], s=55, marker="s", label="End")
    style_3d_axis(ax3, "Trajectory - 3D View", "Downrange [m]", "Crossrange [m]", "Altitude [m]")
    ax3.legend(fontsize=9, loc="best")
    ax3.view_init(elev=24, azim=-58)

    finalize_figure(fig, "Missile Trajectory")


def make_overview_figure(df: pd.DataFrame):
    fig, axes = plt.subplots(3, 3, figsize=(16, 11))
    axes = axes.ravel()

    configs = [
        (["dvbe", "mach"], ["Speed", "Mach"], "Speed and Mach", "Time [s]", ""),
        (["alphax", "betax"], ["Alpha", "Beta"], "Aerodynamic Angles", "Time [s]", "deg"),
        (["ppx", "qqx", "rrx"], ["p", "q", "r"], "Body Rates", "Time [s]", "rad/s"),
        (["dpcx", "dqcx", "drcx"], ["Roll Cmd", "Pitch Cmd", "Yaw Cmd"], "Control Commands", "Time [s]", "deg"),
        (["dpx", "dqx", "drx"], ["Roll Act", "Pitch Act", "Yaw Act"], "Actuator Outputs", "Time [s]", "deg"),
        (["ancomx", "alcomx"], ["Normal Cmd", "Lateral Cmd"], "Guidance Commands", "Time [s]", "g"),
        (["anx", "ayx"], ["Normal Accel", "Lateral Accel"], "Acceleration Response", "Time [s]", "g"),
        (["dtbc"], ["Distance to Target"], "Distance to Target", "Time [s]", "m"),
        (["tgoc"], ["Time-to-Go"], "Time [s]", "Time [s]", "s"),
    ]

    for ax, (cols, labels, title, xlabel, ylabel) in zip(axes, configs):
        ok = plot_series(ax, df, cols, labels)
        if ok:
            style_axis(ax, title, xlabel=xlabel, ylabel=ylabel)

    finalize_figure(fig, "Simulation Overview")


def main():
    filepath = "logs/sim_log_s2.csv"  # gerekirse log.txt yap
    df = load_log(filepath)

    print("Loaded columns:")
    print(df.columns.tolist())

    make_overview_figure(df)
    make_states_figure(df)
    make_control_figure(df)
    make_guidance_figure(df)
    make_tracking_figure(df)
    make_trajectory_figure(df)

    plt.show()


if __name__ == "__main__":
    plt.rcParams.update({
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
        "axes.edgecolor": "#333333",
        "axes.linewidth": 0.8,
        "axes.titlesize": 12,
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.frameon": True,
        "legend.framealpha": 0.92,
        "legend.fancybox": True,
        "lines.linewidth": 1.8,
    })

    main()
