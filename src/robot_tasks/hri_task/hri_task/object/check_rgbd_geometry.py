#!/usr/bin/env python3
"""実機を動かさず、中央・左・右画素の3次元投影を確認する。

既定値は従来の640x480固定内部パラメータ。実機のCameraInfo値が分かる場合は
``--fx``などへ渡すことで、その値を使った座標を同じ形式で確認できる。
"""

from __future__ import annotations

import argparse

from rgbd_geometry import deproject, validate_camera


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fx", type=float, default=615.0)
    parser.add_argument("--fy", type=float, default=615.0)
    parser.add_argument("--cx", type=float, default=320.0)
    parser.add_argument("--cy", type=float, default=240.0)
    parser.add_argument("--depth", type=float, default=0.5, help="深度[m]")
    parser.add_argument(
        "--horizontal-span",
        type=float,
        default=160.0,
        help="中央から左右へ離す画素数",
    )
    return parser.parse_args()


def projection_rows(args: argparse.Namespace):
    """左・中央・右の画素とカメラ座標を返す。"""

    frame_id = "offline_color_optical_frame"
    model = validate_camera(
        {
            "width": args.width,
            "height": args.height,
            "fx": args.fx,
            "fy": args.fy,
            "cx": args.cx,
            "cy": args.cy,
            "d": [],
            "distortion_model": "",
            "frame_id": frame_id,
            "source": "offline_arguments",
        },
        args.width,
        args.height,
        frame_id,
    )
    pixels = (
        ("left", args.cx - args.horizontal_span, args.cy),
        ("center", args.cx, args.cy),
        ("right", args.cx + args.horizontal_span, args.cy),
    )
    return [
        (name, u, v, *deproject(u, v, args.depth, model))
        for name, u, v in pixels
    ]


def validate_rows(rows) -> None:
    """投影方向と、中心から等距離な左右の対称性を確認する。"""

    left_x = rows[0][3]
    center_x = rows[1][3]
    right_x = rows[2][3]
    if not left_x < center_x < right_x:
        raise RuntimeError("Horizontal projection is not monotonic")
    if abs(center_x) > 1e-12:
        raise RuntimeError("Principal point did not project to camera X=0")
    if abs(left_x + right_x) > 1e-12:
        raise RuntimeError("Left/right projections are not symmetric")


def main() -> None:
    args = parse_args()
    rows = projection_rows(args)
    validate_rows(rows)

    print("position  pixel_u  pixel_v  camera_x[m]  camera_y[m]  camera_z[m]")
    for name, u, v, x, y, z in rows:
        print(f"{name:8s} {u:8.2f} {v:8.2f} {x:12.6f} {y:12.6f} {z:12.6f}")
    print("offline RGBD projection check: OK")


if __name__ == "__main__":
    main()
