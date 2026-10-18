"""Interactive SO(3) axis-angle visualization and README figure generator."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from matplotlib.widgets import Button, Slider


FONT_FAMILY = "Noto Sans CJK SC"


def configure_matplotlib():
    matplotlib.rcParams["font.family"] = "sans-serif"
    matplotlib.rcParams["font.sans-serif"] = [
        FONT_FAMILY,
        "WenQuanYi Micro Hei",
        "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False


def normalized(vector):
    value = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(value))
    if norm <= 1e-12:
        raise ValueError("zero vector")
    return value / norm


def axis_from_alpha_beta(alpha_deg, beta_deg):
    alpha = math.radians(float(alpha_deg))
    beta = math.radians(float(beta_deg))
    return np.array(
        [
            math.cos(beta) * math.cos(alpha),
            math.cos(beta) * math.sin(alpha),
            math.sin(beta),
        ],
        dtype=np.float64,
    )


def skew(axis):
    x, y, z = normalized(axis)
    return np.array(
        [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]],
        dtype=np.float64,
    )


def axis_angle_rotation(axis, theta_deg):
    theta = math.radians(float(theta_deg))
    cross = skew(axis)
    return (
        np.eye(3)
        + math.sin(theta) * cross
        + (1.0 - math.cos(theta)) * (cross @ cross)
    )


def draw_vector(ax, vector, color, label, linewidth=3.0, linestyle="-"):
    value = np.asarray(vector, dtype=np.float64)
    ax.quiver(
        0.0,
        0.0,
        0.0,
        value[0],
        value[1],
        value[2],
        color=color,
        linewidth=linewidth,
        linestyle=linestyle,
        arrow_length_ratio=0.09,
    )
    ax.text(
        value[0] * 1.06,
        value[1] * 1.06,
        value[2] * 1.06,
        label,
        color=color,
        fontsize=11,
        weight="bold",
    )


def slerp_vectors(first, second, samples=40):
    start = normalized(first)
    end = normalized(second)
    dot = float(np.clip(np.dot(start, end), -1.0, 1.0))
    angle = math.acos(dot)
    if angle < 1e-8:
        return np.repeat(start[None, :], samples, axis=0)
    values = []
    for fraction in np.linspace(0.0, 1.0, samples):
        value = (
            math.sin((1.0 - fraction) * angle) * start
            + math.sin(fraction * angle) * end
        ) / math.sin(angle)
        values.append(value)
    return np.asarray(values)


class SO3Visualization:
    def __init__(self, alpha=35.0, beta=24.0, theta=42.0):
        configure_matplotlib()
        self.figure = plt.figure(figsize=(15.5, 8.8), facecolor="#f8fafc")
        self.axes = self.figure.add_axes(
            [0.035, 0.18, 0.61, 0.76], projection="3d"
        )
        self.info_axes = self.figure.add_axes([0.675, 0.21, 0.295, 0.69])
        self.info_axes.axis("off")

        self.alpha_slider = Slider(
            self.figure.add_axes([0.12, 0.115, 0.48, 0.025]),
            "alpha / deg",
            -180.0,
            180.0,
            valinit=alpha,
            valstep=0.5,
            color="#7c3aed",
        )
        self.beta_slider = Slider(
            self.figure.add_axes([0.12, 0.075, 0.48, 0.025]),
            "beta / deg",
            -89.0,
            89.0,
            valinit=beta,
            valstep=0.5,
            color="#0891b2",
        )
        self.theta_slider = Slider(
            self.figure.add_axes([0.12, 0.035, 0.48, 0.025]),
            "theta / deg",
            -180.0,
            180.0,
            valinit=theta,
            valstep=0.5,
            color="#ea580c",
        )
        self.reset_button = Button(
            self.figure.add_axes([0.69, 0.08, 0.10, 0.05]),
            "复位",
            color="#e2e8f0",
            hovercolor="#cbd5e1",
        )
        self.play_button = Button(
            self.figure.add_axes([0.81, 0.08, 0.13, 0.05]),
            "播放 theta",
            color="#ffedd5",
            hovercolor="#fed7aa",
        )
        self._playing = False
        self._timer = self.figure.canvas.new_timer(interval=60)
        self._timer.add_callback(self._advance_theta)

        for slider in (
            self.alpha_slider,
            self.beta_slider,
            self.theta_slider,
        ):
            slider.on_changed(self.update)
        self.reset_button.on_clicked(self.reset)
        self.play_button.on_clicked(self.toggle_play)
        self.update()

    def _advance_theta(self):
        if not self._playing:
            return
        value = self.theta_slider.val + 1.5
        if value > 180.0:
            value = -180.0
        self.theta_slider.set_val(value)

    def toggle_play(self, _event):
        self._playing = not self._playing
        self.play_button.label.set_text(
            "暂停" if self._playing else "播放 theta"
        )
        if self._playing:
            self._timer.start()
        else:
            self._timer.stop()

    def reset(self, _event):
        self.alpha_slider.reset()
        self.beta_slider.reset()
        self.theta_slider.reset()

    def update(self, _value=None):
        alpha = self.alpha_slider.val
        beta = self.beta_slider.val
        theta = self.theta_slider.val
        axis = axis_from_alpha_beta(alpha, beta)
        rotation = axis_angle_rotation(axis, theta)

        self._draw_geometry(axis, rotation)
        self._draw_information(alpha, beta, theta, axis, rotation)
        self.figure.canvas.draw_idle()

    def _draw_geometry(self, axis, rotation):
        ax = self.axes
        ax.cla()
        ax.set_facecolor("#ffffff")

        longitude = np.linspace(0.0, 2.0 * np.pi, 42)
        latitude = np.linspace(0.0, np.pi, 22)
        sphere_x = np.outer(np.cos(longitude), np.sin(latitude))
        sphere_y = np.outer(np.sin(longitude), np.sin(latitude))
        sphere_z = np.outer(np.ones_like(longitude), np.cos(latitude))
        ax.plot_wireframe(
            sphere_x,
            sphere_y,
            sphere_z,
            rstride=3,
            cstride=3,
            color="#67e8f9",
            linewidth=0.55,
            alpha=0.48,
        )

        colors = ("#dc2626", "#16a34a", "#2563eb")
        labels = ("X", "Y", "Z")
        basis = np.eye(3)
        for index, (color, label) in enumerate(zip(colors, labels)):
            draw_vector(
                ax,
                basis[:, index] * 0.72,
                color,
                f"{label}0",
                linewidth=1.5,
                linestyle="--",
            )
            draw_vector(
                ax,
                rotation[:, index] * 1.05,
                color,
                f"{label}'",
                linewidth=4.0,
            )

        draw_vector(ax, axis * 1.34, "#7c3aed", "u / A ray", linewidth=5.0)
        ax.plot(
            [-axis[0] * 1.15, axis[0] * 1.15],
            [-axis[1] * 1.15, axis[1] * 1.15],
            [-axis[2] * 1.15, axis[2] * 1.15],
            color="#7c3aed",
            linewidth=1.5,
            linestyle=":",
            alpha=0.8,
        )

        b_ray = normalized([0.20, -0.88, 0.43])
        if abs(float(np.dot(b_ray, axis))) > 0.92:
            b_ray = normalized([-0.68, 0.22, 0.70])
        rotated_b = rotation @ b_ray
        draw_vector(ax, b_ray * 0.92, "#94a3b8", "B0", linewidth=2.0)
        draw_vector(ax, rotated_b * 1.22, "#ea580c", "B(theta)", linewidth=4.0)
        arc = slerp_vectors(b_ray, rotated_b, samples=42) * 0.92
        ax.plot(
            arc[:, 0],
            arc[:, 1],
            arc[:, 2],
            color="#f97316",
            linewidth=3.0,
        )

        ax.scatter([0.0], [0.0], [0.0], color="#111827", s=35)
        ax.set_xlim(-1.35, 1.35)
        ax.set_ylim(-1.35, 1.35)
        ax.set_zlim(-1.35, 1.35)
        ax.set_box_aspect((1.0, 1.0, 1.0))
        ax.view_init(elev=23.0, azim=37.0)
        ax.set_xlabel("world X")
        ax.set_ylabel("world Y")
        ax.set_zlabel("world Z")
        ax.set_title(
            "SO(3) 轴角动态可视化\n"
            "虚线三轴 = R0，实线三轴 = Exp(theta[u]x) R0",
            fontsize=17,
            weight="bold",
            pad=15,
        )

    def _draw_information(self, alpha, beta, theta, axis, rotation):
        ax = self.info_axes
        ax.cla()
        ax.axis("off")
        ax.text(
            0.0,
            0.98,
            "当前 SO(3) 参数",
            fontsize=19,
            weight="bold",
            color="#0f172a",
            va="top",
        )
        ax.text(
            0.0,
            0.90,
            f"alpha = {alpha:8.2f} deg\n"
            f"beta  = {beta:8.2f} deg\n"
            f"theta = {theta:8.2f} deg",
            fontsize=15,
            family="monospace",
            color="#334155",
            va="top",
            linespacing=1.55,
        )
        ax.text(
            0.0,
            0.71,
            "转轴 u",
            fontsize=16,
            weight="bold",
            color="#6d28d9",
        )
        ax.text(
            0.0,
            0.65,
            np.array2string(axis, precision=4, suppress_small=True),
            fontsize=14,
            family="monospace",
            color="#334155",
        )
        ax.text(
            0.0,
            0.56,
            "旋转矩阵 C = Exp(theta [u]x)",
            fontsize=16,
            weight="bold",
            color="#c2410c",
        )
        matrix_text = "\n".join(
            "  ".join(f"{value: .4f}" for value in row) for row in rotation
        )
        ax.text(
            0.0,
            0.49,
            matrix_text,
            fontsize=13,
            family="monospace",
            color="#1e293b",
            va="top",
            linespacing=1.5,
        )
        orthogonal_error = float(
            np.linalg.norm(rotation.T @ rotation - np.eye(3))
        )
        determinant = float(np.linalg.det(rotation))
        ax.text(
            0.0,
            0.29,
            "SO(3) 合法性",
            fontsize=16,
            weight="bold",
            color="#0f172a",
        )
        ax.text(
            0.0,
            0.22,
            f"||C^T C - I|| = {orthogonal_error:.3e}\n"
            f"det(C)          = {determinant:.6f}",
            fontsize=13,
            family="monospace",
            color="#334155",
            linespacing=1.5,
        )
        ax.text(
            0.0,
            0.08,
            "A 成功锁定轴方向 alpha/beta；\n"
            "B 不共线后，只调 theta。",
            fontsize=15,
            color="#0f172a",
            linespacing=1.5,
        )

    def save(self, path):
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        self.figure.savefig(
            output,
            dpi=180,
            bbox_inches="tight",
            facecolor=self.figure.get_facecolor(),
        )

    def show(self):
        plt.show()


def add_box(ax, x, y, width, height, text, face, edge, fontsize=13):
    patch = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.03,rounding_size=0.12",
        linewidth=2.0,
        facecolor=face,
        edgecolor=edge,
    )
    ax.add_patch(patch)
    ax.text(
        x + width / 2.0,
        y + height / 2.0,
        text,
        ha="center",
        va="center",
        fontsize=fontsize,
        color="#0f172a",
        linespacing=1.35,
    )
    return patch


def add_arrow(ax, start, end, color="#475569", connection="arc3"):
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=18,
        linewidth=2.3,
        color=color,
        connectionstyle=connection,
    )
    ax.add_patch(arrow)


def save_chinese_flowchart(path):
    configure_matplotlib()
    figure, ax = plt.subplots(figsize=(16, 9), facecolor="#f8fafc")
    ax.set_xlim(0.0, 16.0)
    ax.set_ylim(0.0, 9.0)
    ax.axis("off")
    ax.text(
        8.0,
        8.62,
        "相机安装角两阶段标定流程",
        ha="center",
        fontsize=25,
        weight="bold",
        color="#0f172a",
    )

    ax.add_patch(
        FancyBboxPatch(
            (0.35, 1.25),
            7.35,
            6.85,
            boxstyle="round,pad=0.05,rounding_size=0.2",
            facecolor="#f5f3ff",
            edgecolor="#8b5cf6",
            linewidth=2.5,
        )
    )
    ax.text(
        0.72,
        7.65,
        "阶段 A：确定旋转轴方向 alpha / beta",
        fontsize=18,
        weight="bold",
        color="#6d28d9",
    )
    add_box(
        ax,
        0.8,
        6.35,
        2.45,
        0.82,
        "实时 raw IMU\n+ 旧总偏置",
        "#ffffff",
        "#c4b5fd",
    )
    add_box(
        ax,
        4.1,
        6.35,
        2.75,
        0.82,
        "生成实时相机旋转\nR_A(t)",
        "#ffffff",
        "#c4b5fd",
    )
    add_arrow(ax, (3.25, 6.76), (4.08, 6.76))
    add_box(
        ax,
        2.2,
        4.85,
        3.75,
        0.95,
        "电控寻找可直接插入位置 A\n发送原 0x02，comm 发送原 0x01",
        "#ffffff",
        "#8b5cf6",
    )
    add_arrow(ax, (5.48, 6.35), (4.25, 5.82))
    add_box(
        ax,
        0.8,
        3.25,
        2.9,
        0.95,
        "锁存 Trial A\nPnP / IMU / R_A / UART",
        "#ffffff",
        "#8b5cf6",
    )
    add_box(
        ax,
        4.65,
        3.25,
        2.05,
        0.95,
        "人工确认\nY 成功 / N 失败",
        "#ffffff",
        "#8b5cf6",
    )
    add_arrow(ax, (4.05, 4.85), (2.6, 4.22))
    add_arrow(ax, (3.7, 3.72), (4.62, 3.72))
    add_box(
        ax,
        2.15,
        1.78,
        3.85,
        0.9,
        "A 成功：冻结 i_A、R_A\n计算轴 u 及 alpha / beta",
        "#ede9fe",
        "#7c3aed",
        fontsize=14,
    )
    add_arrow(ax, (5.68, 3.25), (4.65, 2.70), color="#16a34a")

    ax.add_patch(
        FancyBboxPatch(
            (8.3, 1.25),
            7.35,
            6.85,
            boxstyle="round,pad=0.05,rounding_size=0.2",
            facecolor="#fff7ed",
            edgecolor="#f97316",
            linewidth=2.5,
        )
    )
    ax.text(
        8.67,
        7.65,
        "阶段 B：确定绕固定轴的 theta",
        fontsize=18,
        weight="bold",
        color="#c2410c",
    )
    add_box(
        ax,
        8.78,
        6.35,
        2.2,
        0.82,
        "Q / E 调 theta\n[ / ] 调步长",
        "#ffffff",
        "#fdba74",
    )
    add_box(
        ax,
        11.72,
        6.35,
        3.05,
        0.82,
        "SO(3) 上计算\nR_theta = Exp(theta[u]x) R_A",
        "#ffffff",
        "#fdba74",
    )
    add_arrow(ax, (10.98, 6.76), (11.70, 6.76))
    add_box(
        ax,
        10.0,
        4.85,
        3.9,
        0.95,
        "转为有效 YPR，再减 i_A\n得到候选新总偏置 o_theta",
        "#ffffff",
        "#f97316",
    )
    add_arrow(ax, (13.22, 6.35), (12.05, 5.82))
    add_box(
        ax,
        8.78,
        3.25,
        3.0,
        0.95,
        "选择与 A 不共线的 B\n电控发送 0x02，锁存 Trial B",
        "#ffffff",
        "#f97316",
    )
    add_box(
        ax,
        12.65,
        3.25,
        2.2,
        0.95,
        "人工确认\nY 成功 / N 继续调",
        "#ffffff",
        "#f97316",
    )
    add_arrow(ax, (11.95, 4.85), (10.45, 4.22))
    add_arrow(ax, (11.78, 3.72), (12.62, 3.72))
    add_box(
        ax,
        10.05,
        1.78,
        3.65,
        0.9,
        "B 成功：输出新总偏置\n完整替换旧 offset，不相加",
        "#dcfce7",
        "#16a34a",
        fontsize=14,
    )
    add_arrow(ax, (13.75, 3.25), (12.25, 2.70), color="#16a34a")

    add_arrow(ax, (7.0, 2.18), (9.75, 2.18), color="#334155")
    ax.text(
        8.35,
        2.43,
        "轴已固定，只剩 1 DOF",
        ha="center",
        fontsize=12,
        color="#334155",
    )
    ax.text(
        8.0,
        0.62,
        "机械臂遮挡后仍确认已锁存的 Trial ID；没有实际 UART TX 时不允许确认。",
        ha="center",
        fontsize=14,
        color="#475569",
    )

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(
        output,
        dpi=180,
        bbox_inches="tight",
        facecolor=figure.get_facecolor(),
    )
    plt.close(figure)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Interactive SO(3) axis-angle visualization"
    )
    parser.add_argument(
        "--save",
        help="Save the current interactive view to a PNG before showing",
    )
    parser.add_argument(
        "--flow",
        help="Generate the Chinese calibration flowchart PNG",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open the interactive window",
    )
    parser.add_argument("--alpha", type=float, default=35.0)
    parser.add_argument("--beta", type=float, default=24.0)
    parser.add_argument("--theta", type=float, default=42.0)
    return parser.parse_args()


def main():
    arguments = parse_args()
    if arguments.flow:
        save_chinese_flowchart(arguments.flow)
    if arguments.save or not arguments.flow:
        visualization = SO3Visualization(
            alpha=arguments.alpha,
            beta=arguments.beta,
            theta=arguments.theta,
        )
        if arguments.save:
            visualization.save(arguments.save)
        if not arguments.no_show:
            visualization.show()


if __name__ == "__main__":
    main()
