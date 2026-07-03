#!/usr/bin/env python3
"""
VLA/VLM helper for checking seat descriptions from YOLO detections.

This script is intentionally separate from the main empty-seat state. It reads
YOLO detections, builds sofa-based seat candidates, and asks an
OpenAI-compatible VLA/VLM endpoint to describe every detected seating option.
"""
import argparse
import base64
import json
import os
import time
import urllib.error
import urllib.request


SOFA_LABELS = {"sofa", "couch"}
SEATING_LABELS = {"chair", "sofa", "couch", "bench"}


def bbox(det):
    values = det.get("bbox", [0, 0, 0, 0])
    if len(values) != 4:
        return [0.0, 0.0, 0.0, 0.0]
    return [float(v) for v in values]


def center_x(det):
    x1, _, x2, _ = bbox(det)
    return (x1 + x2) / 2.0


def width(det):
    x1, _, x2, _ = bbox(det)
    return max(1.0, x2 - x1)


def confidence(det):
    return float(det.get("confidence", det.get("score", 1.0)))


def primary_front_sofa(seating, image_width):
    sofas = [d for d in seating if d.get("label") in SOFA_LABELS]
    if not sofas:
        return None
    image_center = image_width / 2.0
    return min(
        sofas,
        key=lambda d: (abs(center_x(d) - image_center), -width(d), -confidence(d)),
    )


def sofa_slots(sofa):
    x1, y1, x2, y2 = bbox(sofa)
    w = max(1.0, x2 - x1)
    names = ["left side", "middle", "right side"]
    slots = []
    for i, name in enumerate(names):
        sx1 = x1 + w * i / 3.0
        sx2 = x1 + w * (i + 1) / 3.0
        slots.append({
            "id": f"front_sofa_{name.replace(' ', '_')}",
            "kind": "sofa_slot",
            "description_hint": f"the {name} of the sofa directly in front of me",
            "bbox": [sx1, y1, sx2, y2],
            "source_bbox": [x1, y1, x2, y2],
        })
    return slots


def chair_candidates(seating, primary_sofa):
    chairs = []
    for det in seating:
        if det is primary_sofa or det.get("label") in SOFA_LABELS:
            continue
        chairs.append(det)
    chairs.sort(key=center_x)
    result = []
    for i, det in enumerate(chairs, start=1):
        result.append({
            "id": f"chair_{i}_from_left",
            "kind": "chair",
            "description_hint": f"the {ordinal(i)} chair from the left from my point of view",
            "bbox": bbox(det),
            "source_label": det.get("label", "chair"),
        })
    return result


def ordinal(n):
    return {1: "first", 2: "second", 3: "third", 4: "fourth", 5: "fifth"}.get(n, f"{n}th")


def build_candidates(detections, image_width):
    seating = [
        d for d in detections
        if d.get("label") in SEATING_LABELS and confidence(d) >= 0.0
    ]
    sofa = primary_front_sofa(seating, image_width)
    candidates = []
    if sofa:
        candidates.extend(sofa_slots(sofa))
    candidates.extend(chair_candidates(seating, sofa))
    return {
        "primary_sofa_bbox": bbox(sofa) if sofa else None,
        "seat_candidates": candidates,
        "people": [
            {
                "bbox": bbox(d),
                "confidence": confidence(d),
                "distance_z": d.get("distance_z"),
            }
            for d in detections
            if d.get("label") == "person"
        ],
    }


def image_data_url(path):
    with open(path, "rb") as f:
        data = base64.b64encode(f.read()).decode("ascii")
    ext = os.path.splitext(path)[1].lower()
    mime = "image/png" if ext == ".png" else "image/jpeg"
    return f"data:{mime};base64,{data}"


def compressed_image_data_url(msg):
    fmt = (getattr(msg, "format", "") or "jpeg").lower()
    mime = "image/png" if "png" in fmt else "image/jpeg"
    data = base64.b64encode(bytes(msg.data)).decode("ascii")
    return f"data:{mime};base64,{data}"


def build_prompt(scene):
    return (
        "You are checking seating instructions for a receptionist robot. "
        "Use the sofa directly in front of the robot as the reference whenever it exists. "
        "Describe every seat candidate in concrete language from the robot's point of view. "
        "For sofa slots, say left side, middle, or right side of the sofa directly in front of me. "
        "For chairs, say which chair it is from the left from my point of view. "
        "If people appear seated, describe that by sofa slot or chair order, not by identity. "
        "Return only JSON with keys: per_candidate_descriptions, recommended_instruction, notes.\n\n"
        f"Scene JSON:\n{json.dumps(scene, ensure_ascii=False, indent=2)}"
    )


def call_vla(base_url, model, api_key, prompt, image_path=None, image_url=None, timeout=30.0):
    content = [{"type": "text", "text": prompt}]
    if image_url:
        content.append({"type": "image_url", "image_url": {"url": image_url}})
    elif image_path:
        content.append({"type": "image_url", "image_url": {"url": image_data_url(image_path)}})

    payload = {
        "model": model,
        "messages": [{"role": "user", "content": content}],
        "temperature": 0.0,
    }
    req = urllib.request.Request(
        url=f"{base_url.rstrip('/')}/chat/completions",
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as res:
            data = json.loads(res.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"VLA request failed with status {exc.code}: {detail}") from exc

    return data["choices"][0]["message"]["content"]


def capture_from_yolo_topics(args):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    class YoloCaptureNode(Node):
        def __init__(self):
            super().__init__("test_empty_chair_vla")
            self.latest_detections = None
            self.latest_image_msg = None
            self.create_subscription(String, args.result_topic, self.result_cb, 10)
            self.create_subscription(CompressedImage, args.image_topic, self.image_cb, 10)
            self.get_logger().info(f"Waiting for detections: {args.result_topic}")
            self.get_logger().info(f"Waiting for image: {args.image_topic}")

        def result_cb(self, msg):
            try:
                detections = json.loads(msg.data)
                if isinstance(detections, dict):
                    detections = detections.get("detections", [])
                self.latest_detections = detections
            except Exception as exc:
                self.get_logger().warn(f"Failed to parse YOLO result: {exc}")

        def image_cb(self, msg):
            self.latest_image_msg = msg

    rclpy.init()
    node = YoloCaptureNode()
    deadline = time.time() + args.ros_timeout
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.latest_detections is not None and node.latest_image_msg is not None:
                return node.latest_detections, compressed_image_data_url(node.latest_image_msg)
        raise TimeoutError(
            f"Timed out waiting for {args.result_topic} and {args.image_topic} "
            f"after {args.ros_timeout:.1f}s"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--detections", help="Path to YOLO detections JSON. Omit with --ros.")
    parser.add_argument("--image", help="Optional image shown to the VLA/VLM.")
    parser.add_argument("--ros", action="store_true", help="Read detections and image from YOLO ROS topics.")
    parser.add_argument("--result-topic", default="/yolo_human/result")
    parser.add_argument("--image-topic", default="/yolo_human/debug_image/compressed")
    parser.add_argument("--ros-timeout", type=float, default=10.0)
    parser.add_argument("--image-width", type=int, default=640)
    parser.add_argument("--base-url", default="http://localhost:8000/v1")
    parser.add_argument("--model", default="local-vlm")
    parser.add_argument("--api-key", default=os.environ.get("OPENAI_API_KEY", "local-vlm"))
    parser.add_argument("--call-vla", action="store_true", help="Call the VLA/VLM endpoint.")
    args = parser.parse_args()

    image_url = None
    if args.ros:
        detections, image_url = capture_from_yolo_topics(args)
    else:
        if not args.detections:
            parser.error("--detections is required unless --ros is set.")
        with open(args.detections, "r", encoding="utf-8") as f:
            detections = json.load(f)
        if isinstance(detections, dict):
            detections = detections.get("detections", [])

    scene = build_candidates(detections, args.image_width)
    prompt = build_prompt(scene)

    if not args.call_vla:
        print(json.dumps({"scene": scene, "prompt": prompt}, ensure_ascii=False, indent=2))
        return

    result = call_vla(
        args.base_url,
        args.model,
        args.api_key,
        prompt,
        image_path=args.image,
        image_url=image_url,
    )
    print(result)


if __name__ == "__main__":
    main()
