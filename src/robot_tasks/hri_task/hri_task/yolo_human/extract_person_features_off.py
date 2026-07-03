import base64
import json
import os
import urllib.error
import urllib.request
from typing import Any

import cv2
import numpy as np


def _image_to_data_url(image: np.ndarray, image_format: str) -> str:
    if image is None or getattr(image, "size", 0) == 0:
        raise ValueError("`image` must be a non-empty OpenCV image array.")
    success, buffer = cv2.imencode(image_format, image)
    if not success:
        raise ValueError(f"Failed to encode image with format {image_format!r}.")
    b64_image = base64.b64encode(buffer.tobytes()).decode("ascii")
    mime = "image/png" if image_format.lower() == ".png" else "image/jpeg"
    return f"data:{mime};base64,{b64_image}"


# Default extraction schema for `extract_person_features`, expressed as a
# mapping of field name to a short description. This follows the YAML
# field-list format recommended by the LFM2.5-VL-Extract prompting guide.
"""
DEFAULT_PERSON_FIELDS: dict[str, str] = {
    "clothing": (
        "The clothing the person at the center is wearing, including colors "
        "and style"
    ),
    "hairstyle": "The person's hairstyle, including length and color",
    "accessories": "Visible accessories such as glasses, hats, jewelry, or bags",
    "approximate_age": "The person's approximate age range",
}
"""
DEFAULT_PERSON_FIELDS: dict[str, str] = {
    # ===== Upper body =====
    "upper_clothing_color": (
        "Primary color of the upper-body clothing worn by the person closest to the center of the image. "
        "Use basic color names (black, white, gray, blue, navy, red, green, yellow, brown, etc.). "
        "If not visible, return 'none'."
    ),

    "upper_clothing_sleeve": (
        "Sleeve length of the upper-body clothing worn by the person closest to the center of the image. "
        "Choose one of: short sleeves, long sleeves, sleeveless, unknown. "
        "If not visible, return 'none'."
    ),

    "upper_clothing_pattern": (
        "Pattern or visual design on the upper-body clothing worn by the person closest to the center of the image. "
        "Examples: solid, striped, checkered, logo, graphic print, text print. "
        "If no pattern is visible, return 'none'."
    ),

    "upper_clothing_type": (
        "Type of upper-body clothing worn by the person closest to the center of the image. "
        "Examples: t-shirt, shirt, hoodie, sweater, jacket, coat, uniform. "
        "If not visible, return 'none'."
    ),

    # ===== Lower body =====
    "lower_clothing_color": (
        "Primary color of the lower-body clothing (pants, skirt) worn by the person closest to the center of the image. "
        "Use basic color names. If not visible, return 'none'."
    ),

    "lower_clothing_type": (
        "Type of lower-body clothing worn by the person closest to the center of the image. "
        "Examples: jeans, trousers, shorts, skirt, leggings. "
        "If not visible, return 'none'."
    ),

    "lower_clothing_pattern": (
        "Pattern or design on the lower-body clothing worn by the person closest to the center of the image. "
        "Examples: solid, striped, ripped, patterned, logo. "
        "If none is visible, return 'none'."
    ),

    # ===== Face / head =====
    "hair_color": (
        "Visible hair color of the person closest to the center of the image. "
        "Examples: black, brown, blond, gray, white, dyed color. "
        "If hair is not visible, return 'none'."
    ),

    "hair_style": (
        "Visible hairstyle and approximate length of the person closest to the center of the image. "
        "Examples: short hair, medium hair, long hair, ponytail, bun, curly, straight, buzz cut. "
        "If not visible, return 'none'."
    ),

    "glasses": (
        "Presence and type of glasses worn by the person closest to the center of the image. "
        "Return 'none' if not present."
    ),

    "mask": (
        "Presence and color of face mask worn by the person closest to the center of the image. "
        "Return 'none' if not present."
    ),

    "hat": (
        "Presence and type of headwear worn by the person closest to the center of the image. "
        "Return 'none' if not present."
    ),

    # ===== Accessories =====
    "bag": (
        "Visible bag carried by the person closest to the center of the image. "
        "Examples: backpack, shoulder bag, tote bag, handbag. Include color if visible. "
        "Return 'none' if not present."
    ),
}



def load_image_from_url(
    url: str,
    *,
    timeout: float = 30.0,
) -> np.ndarray:
    """Download an image from a URL and return it as an OpenCV (BGR) array.

    Args:
        url: HTTP/HTTPS URL pointing to an image file.
        timeout: Network timeout in seconds.

    Returns:
        The decoded image as a ``numpy.ndarray`` in BGR format.

    Raises:
        ValueError: If the response body cannot be decoded as an image.
        urllib.error.URLError: If the HTTP request fails at the transport level.
    """
    request = urllib.request.Request(
        url,
        headers={"User-Agent": "Mozilla/5.0"},
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        data = np.frombuffer(response.read(), dtype=np.uint8)

    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Failed to decode image from URL: {url!r}")

    return image


def extract_person_features(
    image: np.ndarray,
    *,
    api_key: str | None = None,
    model: str = "LiquidAI/LFM2.5-VL-1.6B-Extract",
    fields: dict[str, str] | None = None,
    image_format: str = ".jpg",
    detail: str = "auto",
    max_tokens: int = 512,
    temperature: float = 0.0,
    timeout: float = 30.0,
    base_url: str = "http://localhost:8000/v1",
) -> dict[str, str]:
    """Extract structured visual features of the person at the center of an image.

    The image is encoded in memory and sent to an OpenAI-compatible Chat
    Completions (vision) endpoint via the standard library. The request is
    shaped according to the LFM2.5-VL-Extract prompting guide: the extraction
    schema is supplied as a YAML field list in the *system* message, the
    *user* message carries only the image, greedy decoding is used, and the
    model returns a JSON object keyed by the requested field names.

    Args:
        image: Source image as an OpenCV (BGR) ``numpy.ndarray``.
        api_key: API key. If ``None``, the ``OPENAI_API_KEY`` environment
            variable is used instead. (vLLM servers usually accept any value.)
        model: Name of a vision-capable model.
        fields: Mapping of field name to a short description of what to
            extract for that field. Each entry becomes one
            ``name: description`` line in the YAML schema placed in the
            system prompt. Defaults to :data:`DEFAULT_PERSON_FIELDS`.
        image_format: Encoding extension passed to ``cv2.imencode``,
            e.g. ``".jpg"`` or ``".png"``.
        detail: Image detail hint for the API: ``"auto"``, ``"low"``,
            or ``"high"``.
        max_tokens: Upper bound on the number of tokens in the response.
        temperature: Sampling temperature. Defaults to ``0.0`` (greedy
            decoding), as recommended for LFM2.5-VL-Extract.
        timeout: Network timeout in seconds.
        base_url: Base URL of the OpenAI-compatible API.

    Returns:
        A mapping from each requested field name to its extracted value as a
        string.

    Raises:
        ValueError: If the image is empty, the API key is missing, the
            image cannot be encoded, the HTTP request returns an error
            status, or the response cannot be parsed.
        urllib.error.URLError: If the HTTP request fails at the transport
            level (e.g. connection or timeout errors).
    """
    # Local OpenAI-compatible servers such as llama.cpp only require the
    # Authorization header shape; the token value itself is not validated.
    key = api_key or os.environ.get("OPENAI_API_KEY") or "local-vlm"

    data_url = _image_to_data_url(image, image_format)

    # Build the extraction schema following the LFM2.5-VL-Extract prompting
    # guide: the fields to extract are listed as YAML (`name: description`)
    # inside the *system* prompt, and the *user* turn carries only the image.
    schema = fields if fields is not None else DEFAULT_PERSON_FIELDS
    fields_yaml = "\n".join(
        f"{name}: {description}" for name, description in schema.items()
    )
    system_prompt = (
        "Extract the following from the image:\n\n"
        f"{fields_yaml}\n\n"
        "Respond with only a JSON object. Do not include any text outside the JSON."
    )

    # Assemble the request payload. The model is tuned for single-turn,
    # greedy-decoded structured extraction.
    payload: dict[str, Any] = {
        "model": model,
        "messages": [
            {"role": "system", "content": system_prompt},
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {"url": data_url, "detail": detail},
                    },
                ],
            },
        ],
        "max_tokens": max_tokens,
        "temperature": temperature,
        "response_format": {"type": "json_object"},
    }

    request = urllib.request.Request(
        url=f"{base_url}/chat/completions",
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {key}",
        },
        method="POST",
    )

    # Send the request and read the response body.
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            body = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        error_detail = exc.read().decode("utf-8", errors="replace")
        raise ValueError(
            f"OpenAI API request failed with status {exc.code}: {error_detail}"
        ) from exc

    # Parse the model output into a mapping of field name to extracted value.
    try:
        content = body.get("choices", [{}])[0].get("message", {}).get("content", "")
        print(f"--- RAW VLM OUTPUT ---\n{content}\n----------------------")
        extracted = json.loads(content)
    except (json.JSONDecodeError, KeyError, IndexError, TypeError) as exc:
        print(f"VLM Output is not valid JSON. Returning raw string.")
        return {"raw_output": content, "parse_error": str(exc)}

    if not isinstance(extracted, dict):
        return {"raw_output": content, "parse_error": "Response is not a JSON dictionary"}

    return {str(key): str(value) for key, value in extracted.items()}


def match_reference_person(
    reference_image: np.ndarray,
    candidate_a: np.ndarray,
    candidate_b: np.ndarray,
    *,
    api_key: str | None = None,
    model: str = "LiquidAI/LFM2.5-VL-1.6B-Extract",
    image_format: str = ".jpg",
    detail: str = "auto",
    max_tokens: int = 128,
    temperature: float = 0.0,
    timeout: float = 30.0,
    base_url: str = "http://localhost:8000/v1",
) -> dict[str, str]:
    """Return which current candidate best matches the reference person.

    The response is intentionally small so callers can treat low-confidence or
    unparsable answers as a no-op fallback.
    """
    key = api_key or os.environ.get("OPENAI_API_KEY") or "local-vlm"
    ref_url = _image_to_data_url(reference_image, image_format)
    a_url = _image_to_data_url(candidate_a, image_format)
    b_url = _image_to_data_url(candidate_b, image_format)

    prompt = (
        "Image 1 is the reference person. Image 2 is Candidate A. "
        "Image 3 is Candidate B. Decide which candidate is the same person "
        "as the reference, based only on clothing, colors, hair, and visible "
        "accessories. If unsure, answer uncertain. Respond only as JSON: "
        '{"match":"A|B|uncertain","confidence":0.0,"reason":"short reason"}'
    )
    payload: dict[str, Any] = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {"url": ref_url, "detail": detail}},
                    {"type": "image_url", "image_url": {"url": a_url, "detail": detail}},
                    {"type": "image_url", "image_url": {"url": b_url, "detail": detail}},
                ],
            }
        ],
        "max_tokens": max_tokens,
        "temperature": temperature,
        "response_format": {"type": "json_object"},
    }
    request = urllib.request.Request(
        url=f"{base_url}/chat/completions",
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {key}",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            body = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        error_detail = exc.read().decode("utf-8", errors="replace")
        raise ValueError(
            f"OpenAI API request failed with status {exc.code}: {error_detail}"
        ) from exc

    content = body.get("choices", [{}])[0].get("message", {}).get("content", "")
    try:
        parsed = json.loads(content)
    except json.JSONDecodeError as exc:
        return {"match": "uncertain", "confidence": "0.0", "reason": str(exc)}
    if not isinstance(parsed, dict):
        return {"match": "uncertain", "confidence": "0.0", "reason": "not a dict"}
    return {str(k): str(v) for k, v in parsed.items()}
