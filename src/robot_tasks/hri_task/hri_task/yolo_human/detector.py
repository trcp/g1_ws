"""
detector.py
===========
YOLO11-Seg による人・椅子の検出。
person クラスはセグメンテーションマスクも返す。
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional
import numpy as np
import cv2
from ultralytics import YOLOE


@dataclass
class Detection:
    class_name: str                        # "person" / "chair"
    confidence: float
    bbox: tuple[int, int, int, int]        # (x1, y1, x2, y2) pixel
    center_px: tuple[int, int]             # (cx, cy) pixel
    # セグメンテーションマスク（person のみ、フレーム全体と同サイズ bool）
    mask: Optional[np.ndarray] = field(default=None, repr=False)
    # 2D カメラ座標
    cam_x: float = 0.0
    cam_y: float = 0.0
    # 3D カメラ座標（RGBD のみ）
    cam_z: Optional[float] = None
    world_x: Optional[float] = None
    world_y: Optional[float] = None
    # 服解析結果（person のみ）
    clothing: Optional[dict] = field(default=None)

    def __repr__(self):
        z_str = f", Z={self.cam_z:.2f}m" if self.cam_z is not None else ""
        return (f"[{self.class_name}] conf={self.confidence:.2f} "
                f"px=({self.cam_x:.0f},{self.cam_y:.0f}){z_str}")


class Detector:
    """
    YOLO11-Seg で人・椅子を検出する。
    person には segmentation mask を付与する。
    depth_frame を渡すと 3D 座標も付与する。
    """

    def __init__(
        self,
        model_path: str = "yolo11n-seg.pt",
        classes: list[str] | None = None,
        conf_threshold: float = 0.30,
        iou_threshold: float = 0.45,
        device: str = "cuda",
    ):
        self.conf = conf_threshold
        self.iou = iou_threshold
        self.classes = classes or ["person", "chair"]

        print(f"[Detector] Loading: {model_path}")
        self.model = YOLOE(model_path)
        self.device = device

        # YOLO クラス ID → クラス名 マッピングを構築
        self._cls_name_to_id: dict[str, int] = {}
        self._target_ids: list[int] = []
        self.set_classes(self.classes)

    def set_classes(self, classes: list[str]):
        """YOLO-World 等でターゲットクラスを動的に変更する"""
        if hasattr(self.model, "set_classes") and self.classes != classes:
            self.model.set_classes(classes)
        
        self.classes = classes
        self._cls_name_to_id = {v: k for k, v in self.model.names.items()}
        self._target_ids = [
            self._cls_name_to_id[c]
            for c in self.classes
            if c in self._cls_name_to_id
        ]
        # print(f"[Detector] Target classes updated: {self.classes} → IDs: {self._target_ids}")

    # ── 推論 ──────────────────────────────────────────────────────────────────
    def detect(
        self,
        rgb_frame: np.ndarray,
        depth_frame: np.ndarray | None = None,
        camera_intrinsics: dict | None = None,
        classes: list[str] | None = None,
    ) -> list[Detection]:
        """
        Parameters
        ----------
        rgb_frame         : BGR 画像 (H, W, 3)
        depth_frame       : 深度画像 [m] (H, W) float32  ※ RGBD のみ
        camera_intrinsics : {"fx", "fy", "cx", "cy"}      ※ RGBD のみ
        classes           : 検出対象のクラス名リスト (YOLO-World用、動的変更)

        Returns
        -------
        list[Detection]
        """
        if classes is not None:
            self.set_classes(classes)

        H, W = rgb_frame.shape[:2]

        results = self.model.predict(
            source=rgb_frame,
            conf=self.conf,
            iou=self.iou,
            classes=self._target_ids,
            device=self.device,
            verbose=False,
            retina_masks=True,   # 元解像度マスクを取得
        )

        detections: list[Detection] = []
        if not results or results[0].boxes is None:
            return detections

        boxes  = results[0].boxes
        masks  = results[0].masks   # None の場合あり（マスクなしモデル）

        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(W - 1, x2), min(H - 1, y2)

            cls_id   = int(box.cls[0])
            cls_name = self.model.names.get(cls_id, "unknown")
            conf     = float(box.conf[0])
            cx, cy   = (x1 + x2) // 2, (y1 + y2) // 2

            # セグメンテーションマスク取得（person のみ）
            mask: np.ndarray | None = None
            if cls_name == "person" and masks is not None:
                raw = masks.data[i].cpu().numpy()          # (H', W') float
                # 元解像度にリサイズ
                mask_resized = cv2.resize(raw, (W, H), interpolation=cv2.INTER_NEAREST)
                mask = mask_resized > 0.5                  # bool (H, W)

            det = Detection(
                class_name=cls_name,
                confidence=conf,
                bbox=(x1, y1, x2, y2),
                center_px=(cx, cy),
                mask=mask,
                cam_x=float(cx),
                cam_y=float(cy),
            )

            # 3D 座標（RGBD）
            if depth_frame is not None and camera_intrinsics is not None:
                det = self._add_3d(det, depth_frame, camera_intrinsics)

            detections.append(det)

        return detections

    # ── 3D 座標付与 ───────────────────────────────────────────────────────────
    @staticmethod
    def _add_3d(det: Detection, depth: np.ndarray, K: dict) -> Detection:
        cx, cy = det.center_px
        H, W = depth.shape
        if 0 <= cx < W and 0 <= cy < H:
            z = float(depth[cy, cx])
            if z > 0.01:
                det.cam_z   = z
                det.world_x = (cx - K["cx"]) * z / K["fx"]
                det.world_y = (cy - K["cy"]) * z / K["fy"]
        return det
