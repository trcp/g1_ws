"""
clothing_analyzer.py
====================
YOLO11-Seg の person マスクを使って服の特徴を推定する。

処理フロー:
  1. マスクから上半身領域（bbox上20%〜65%）を抽出
  2. 背景・椅子ピクセルをマスクで除去
  3. KMeans(n=3) で代表色クラスタを取得
  4. HSV ヒストグラムから優勢色を色名に変換
  5. CLIP で柄カテゴリを分類（重い処理 → person 検出時のみ呼ばれる）
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np
import cv2

# CLIP / sklearn は遅延インポート
_clip_model = None
_clip_preprocess = None
_clip_text_features = None
_clip_device = "cpu"
_kmeans_available = False


def _load_clip(device: str = "cuda") -> bool:
    global _clip_model, _clip_preprocess, _clip_text_features, _clip_device
    if _clip_model is not None:
        return True
    try:
        import clip
        import torch
        _clip_device = device if torch.cuda.is_available() else "cpu"
        _clip_model, _clip_preprocess = clip.load("ViT-B/32", device=_clip_device)
        _clip_text_features = _encode_pattern_texts(clip, torch)
        _encode_attribute_texts(clip, torch)
        print(f"[ClothingAnalyzer] CLIP loaded on {_clip_device}")
        return True
    except ImportError:
        print("[ClothingAnalyzer] CLIP not available. Pattern detection disabled.")
        return False


def _encode_pattern_texts(clip, torch):
    prompts = [f"a photo of {p}" for p in PATTERN_LABELS_EN]
    tokens = clip.tokenize(prompts).to(_clip_device)
    with torch.no_grad():
        feats = _clip_model.encode_text(tokens)
        feats = feats / feats.norm(dim=-1, keepdim=True)
    return feats


_attribute_prompts = {
    "glasses": ["a photo of a person wearing glasses", "a photo of a person without glasses"],
    "hat": ["a photo of a person wearing a hat", "a photo of a person without a hat"],
    "hair": ["a photo of a person with short hair", "a photo of a person with long hair"],
    "sleeve": ["a photo of a person wearing short sleeves", "a photo of a person wearing long sleeves"]
}
_attribute_text_features = {}

def _encode_attribute_texts(clip, torch):
    global _attribute_text_features
    for key, prompts in _attribute_prompts.items():
        tokens = clip.tokenize(prompts).to(_clip_device)
        with torch.no_grad():
            feats = _clip_model.encode_text(tokens)
            feats = feats / feats.norm(dim=-1, keepdim=True)
        _attribute_text_features[key] = feats


def _load_kmeans() -> bool:
    global _kmeans_available
    try:
        from sklearn.cluster import KMeans  # noqa: F401
        _kmeans_available = True
        return True
    except ImportError:
        print("[ClothingAnalyzer] scikit-learn not available. Falling back to mean color.")
        return False


# ── 柄カテゴリ ─────────────────────────────────────────────────────────────────
PATTERN_LABELS_EN = [
    "solid color clothing",
    "striped clothing",
    "checked or plaid clothing",
    "polka dot clothing",
    "floral print clothing",
    "camouflage clothing",
    "graphic print clothing",
]
PATTERN_LABELS_JA = ["無地", "ストライプ", "チェック", "水玉", "花柄", "迷彩", "グラフィック"]


# ── HSV → 色名テーブル ─────────────────────────────────────────────────────────
# (名前, H_lo, H_hi)  ※ H: 0-179
_COLOR_RULES = [
    ("赤",       0,   10),
    ("赤",     160,  179),
    ("オレンジ", 10,   22),
    ("黄",      22,   35),
    ("緑",      35,   85),
    ("水色",    85,  100),
    ("青",     100,  130),
    ("紫",     130,  160),
]
_COLOR_EN = {
    "赤": "red", "オレンジ": "orange", "黄": "yellow",
    "緑": "green", "水色": "cyan", "青": "blue",
    "紫": "purple", "白": "white", "グレー": "gray", "黒": "black",
}


@dataclass
class ClothingFeatures:
    color_name: str        # 例: "青"
    color_name_en: str     # 例: "blue"
    color_hsv: tuple       # 代表色 (H, S, V)
    pattern: str           # 例: "ストライプ"
    pattern_en: str
    pattern_conf: float

    def __repr__(self):
        return (f"[Clothing] {self.color_name}({self.color_name_en}) "
                f"/ {self.pattern}({self.pattern_en} {self.pattern_conf:.2f})")


class ClothingAnalyzer:
    """
    使い方:
        analyzer = ClothingAnalyzer(device="cuda")
        features = analyzer.analyze(frame, bbox, mask)
        # mask は Detection.mask (bool, フレーム全体サイズ)
    """

    def __init__(self, device: str = "cuda"):
        self.clip_ok   = _load_clip(device)
        self.kmeans_ok = _load_kmeans()

    # ── メイン ────────────────────────────────────────────────────────────────
    def analyze(
        self,
        frame: np.ndarray,
        bbox: tuple[int, int, int, int],
        mask: np.ndarray | None = None,
    ) -> ClothingFeatures:
        """
        レガシーメソッド（互換性のため残す）。
        新しいフローでは human_parsing -> extract_color -> classify_pattern を順に呼ぶ。
        """
        torso_pixels, torso_crop = self.human_parsing(frame, bbox, mask)

        if torso_pixels is None or len(torso_pixels) < 10:
            return self.empty()

        color_name, color_en, color_hsv = self.extract_color(torso_pixels)

        if self.clip_ok and torso_crop is not None:
            pat_idx, pat_conf = self.classify_pattern(torso_crop)
        else:
            pat_idx, pat_conf = 0, 0.0

        return ClothingFeatures(
            color_name=color_name,
            color_name_en=color_en,
            color_hsv=color_hsv,
            pattern=PATTERN_LABELS_JA[pat_idx],
            pattern_en=PATTERN_LABELS_EN[pat_idx].split()[0],
            pattern_conf=pat_conf,
        )

    # ── 複数属性の統合抽出 (Safe Attribute Extraction) ────────────────────────
    def analyze_attributes(
        self,
        frame: np.ndarray,
        bbox: tuple[int, int, int, int],
        mask: np.ndarray | None = None,
    ) -> dict:
        """
        抽出結果の辞書を返す。自信がないもの(確率 < 0.85) は False になる。
        """
        features = {
            "color": False,
            "pattern": False,
            "pattern_conf": 0.0,
            "glasses": False,
            "glasses_conf": 0.0,
            "hat": False,
            "hat_conf": 0.0,
            "hair": False,
            "hair_conf": 0.0,
            "sleeve": False,
            "sleeve_conf": 0.0
        }

        # 1. 画像切り出し (KMeans用に少し範囲を絞る)
        torso_pixels, torso_crop = self.human_parsing(frame, bbox, mask)

        # 2. 色の抽出
        if torso_pixels is not None and len(torso_pixels) >= 10:
            _, color_en, _ = self.extract_color(torso_pixels)
            if color_en != "unknown":
                features["color"] = color_en

        # 顔周りの判定等も含めたいので、CLIP用には BBox 全体をクロップする
        x1, y1, x2, y2 = bbox
        h, w = frame.shape[:2]
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(w, x2)
        y2 = min(h, y2)
        full_crop = frame[y1:y2, x1:x2].copy()

        # 3. CLIP による属性分類
        if self.clip_ok and full_crop.size > 0:
            import torch
            from PIL import Image

            rgb = cv2.cvtColor(full_crop, cv2.COLOR_BGR2RGB)
            pil = Image.fromarray(rgb)
            inp = _clip_preprocess(pil).unsqueeze(0).to(_clip_device)

            with torch.no_grad():
                img_feats = _clip_model.encode_image(inp)
                img_feats = img_feats / img_feats.norm(dim=-1, keepdim=True)

                # パターン (既存の分類器を使用)
                logits_pat = (img_feats @ _clip_text_features.T).squeeze(0)
                probs_pat = logits_pat.softmax(dim=-1).cpu().numpy()
                idx_pat = int(np.argmax(probs_pat))
                if probs_pat[idx_pat] > 0.5:
                    pattern_en = PATTERN_LABELS_EN[idx_pat]
                    # solid color clothing -> solid color
                    pattern_str = " ".join(pattern_en.split()[:2]) if "solid" in pattern_en else pattern_en.split()[0]
                    features["pattern"] = pattern_str
                features["pattern_conf"] = float(probs_pat[idx_pat])

                # 新しいペア属性
                for key, txt_feats in _attribute_text_features.items():
                    logits = (img_feats @ txt_feats.T).squeeze(0)
                    probs = logits.softmax(dim=-1).cpu().numpy()
                    idx = int(np.argmax(probs))
                    conf = float(probs[idx])

                    if conf > 0.5:
                        if key == "glasses":
                            features["glasses"] = "wearing glasses" if idx == 0 else "not wearing glasses"
                        elif key == "hat":
                            features["hat"] = "wearing a hat" if idx == 0 else "not wearing a hat"
                        elif key == "hair":
                            features["hair"] = "short hair" if idx == 0 else "long hair"
                        elif key == "sleeve":
                            features["sleeve"] = "short sleeves" if idx == 0 else "long sleeves"
                    features[f"{key}_conf"] = conf

        return features

    # ── 上半身ピクセル抽出（Human Parsing） ───────────────────────────────────
    @staticmethod
    def human_parsing(
        frame: np.ndarray,
        bbox: tuple[int, int, int, int],
        mask: np.ndarray | None,
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        """
        bbox の上 20% 〜 65% を上半身領域とし、
        mask が有れば背景ピクセルを除去した BGR ピクセル配列を返す。
        """
        x1, y1, x2, y2 = bbox
        h = y2 - y1
        w = x2 - x1
        ty1 = y1 + int(h * 0.20)
        ty2 = y1 + int(h * 0.65)
        ty1 = max(0, ty1)
        ty2 = min(frame.shape[0] - 1, ty2)

        if ty2 <= ty1 or x2 <= x1:
            return None, None

        # CLIP用は少し広め(元の幅の70%)で背景を減らす
        cx1 = x1 + int(w * 0.15)
        cx2 = x1 + int(w * 0.85)
        cx1 = max(0, cx1)
        cx2 = min(frame.shape[1] - 1, cx2)

        torso_crop = frame[ty1:ty2, cx1:cx2].copy()

        if mask is not None:
            # マスクの上半身部分を切り出し
            torso_mask = mask[ty1:ty2, cx1:cx2]  # bool (h', w')
            pixels = torso_crop[torso_mask]     # マスク内ピクセルのみ (N, 3)

            # CLIP 用画像：マスク外を白で塗りつぶす
            torso_vis = torso_crop.copy()
            torso_vis[~torso_mask] = 255
        else:
            # マスクなし: 色抽出(KMeans)用にはさらに中心の40%だけを切り出して背景を排除
            color_cx1 = x1 + int(w * 0.3)
            color_cx2 = x1 + int(w * 0.7)
            color_cx1 = max(0, color_cx1)
            color_cx2 = min(frame.shape[1] - 1, color_cx2)
            
            color_crop = frame[ty1:ty2, color_cx1:color_cx2]
            pixels = color_crop.reshape(-1, 3)
            torso_vis = torso_crop

        if len(pixels) < 10:
            return None, None

        return pixels.astype(np.float32), torso_vis

    # ── 色抽出 (Color Extraction) ─────────────────────────────────────────────
    def extract_color(
        self,
        pixels_bgr: np.ndarray,   # (N, 3) float32
    ) -> tuple[str, str, tuple]:
        """KMeans(n=3) で代表色クラスタを求め、最大クラスタをHSVで色名に変換"""

        # ── KMeans ───────────────────────────────────────────────────────────
        if self.kmeans_ok and len(pixels_bgr) >= 30:
            from sklearn.cluster import KMeans
            n = min(3, len(pixels_bgr))
            km = KMeans(n_clusters=n, n_init=3, max_iter=50, random_state=0)
            labels = km.fit_predict(pixels_bgr)
            # 最大クラスタの中心色を使用
            counts = np.bincount(labels)
            dominant_bgr = km.cluster_centers_[np.argmax(counts)].astype(np.uint8)
        else:
            # フォールバック: 単純平均
            dominant_bgr = np.mean(pixels_bgr, axis=0).astype(np.uint8)

        # ── BGR → HSV 変換 ───────────────────────────────────────────────────
        pixel_img = dominant_bgr.reshape(1, 1, 3)
        hsv = cv2.cvtColor(pixel_img, cv2.COLOR_BGR2HSV)[0, 0]
        H, S, V = int(hsv[0]), int(hsv[1]), int(hsv[2])

        # ── 色名マッチング（無彩色を先に判定） ─────────────────────────
        if V < 45:
            color_ja = "黒"
        elif S < 40 and V > 170:
            color_ja = "白"
        elif S < 45:
            color_ja = "グレー"
        else:
            color_ja = "グレー"   # デフォルト
            for name, h_lo, h_hi in _COLOR_RULES:
                if h_lo <= H <= h_hi:
                    color_ja = name
                    break

        color_en = _COLOR_EN.get(color_ja, color_ja)
        return color_ja, color_en, (H, S, V)

    # ── 柄分類 (Pattern Classification) ───────────────────────────────────────
    @staticmethod
    def classify_pattern(torso_crop: np.ndarray) -> tuple[int, float]:
        import torch
        from PIL import Image

        rgb   = cv2.cvtColor(torso_crop, cv2.COLOR_BGR2RGB)
        pil   = Image.fromarray(rgb)
        inp   = _clip_preprocess(pil).unsqueeze(0).to(_clip_device)

        with torch.no_grad():
            img_feats = _clip_model.encode_image(inp)
            img_feats = img_feats / img_feats.norm(dim=-1, keepdim=True)
            logits    = (img_feats @ _clip_text_features.T).squeeze(0)
            probs     = logits.softmax(dim=-1).cpu().numpy()

        idx = int(np.argmax(probs))
        return idx, float(probs[idx])

    @staticmethod
    def empty() -> ClothingFeatures:
        return ClothingFeatures(
            color_name="不明", color_name_en="unknown",
            color_hsv=(0, 0, 0),
            pattern="不明", pattern_en="unknown",
            pattern_conf=0.0,
        )
