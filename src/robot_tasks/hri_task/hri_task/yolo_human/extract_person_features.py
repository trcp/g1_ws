import base64
import json
import os
import urllib.error
import urllib.request
from typing import Any

import cv2
import numpy as np


def extract_person_features(
    image: np.ndarray,
    *,
    api_key: str | None = None,
    model: str = "gpt-5.5",
    num_features: int = 4,
    image_format: str = ".jpg",
    detail: str = "auto",
    max_tokens: int = 300,
    temperature: float = 1.0,
    timeout: float = 30.0,
    base_url: str = "https://api.openai.com/v1",
) -> list[str]:
    """Extract salient visual features of the person at the center of an image.

    The image is encoded in memory, sent to the OpenAI Chat Completions
    (vision) endpoint via the standard library, and the model's reply is
    parsed into a list of short descriptive strings.

    Args:
        image: Source image as an OpenCV (BGR) ``numpy.ndarray``.
        api_key: OpenAI API key. If ``None``, the ``OPENAI_API_KEY``
            environment variable is used instead.
        model: Name of a vision-capable model.
        num_features: Number of distinct features to request and return.
        image_format: Encoding extension passed to ``cv2.imencode``,
            e.g. ``".jpg"`` or ``".png"``.
        detail: Image detail hint for the API: ``"auto"``, ``"low"``,
            or ``"high"``.
        max_tokens: Upper bound on the number of tokens in the response.
        temperature: Sampling temperature for the model.
        timeout: Network timeout in seconds.
        base_url: Base URL of the OpenAI-compatible API.

    Returns:
        A list of short feature-description strings. The list normally
        contains ``num_features`` items, matching the request.

    Raises:
        ValueError: If the image is empty, the API key is missing, the
            image cannot be encoded, the HTTP request returns an error
            status, or the response cannot be parsed.
        urllib.error.URLError: If the HTTP request fails at the transport
            level (e.g. connection or timeout errors).
    """
    # Validate the input image.
    if image is None or getattr(image, "size", 0) == 0:
        raise ValueError("`image` must be a non-empty OpenCV image array.")

    # Resolve the API key.
    key = api_key or os.environ.get("OPENAI_API_KEY")
    if not key:
        raise ValueError(
            "An OpenAI API key must be provided via `api_key` or the "
            "`OPENAI_API_KEY` environment variable."
        )

    # Encode the image and wrap it as a base64 data URL.
    success, buffer = cv2.imencode(image_format, image)
    if not success:
        raise ValueError(f"Failed to encode image with format {image_format!r}.")

    b64_image = base64.b64encode(buffer.tobytes()).decode("ascii")
    mime = "image/png" if image_format.lower() == ".png" else "image/jpeg"
    data_url = f"data:{mime};base64,{b64_image}"

    # Build the instruction prompt.
    """
    prompt = (
        f"Identify the single person located at the center of this image and "
        f"describe exactly {num_features} of their most salient visual "
        f"features (for example: clothing, hairstyle, accessories, "
        f"approximate age, facial expression). Respond only with a JSON "
        f'object of the form {{"features": ["...", "..."]}} containing '
        f"exactly {num_features} short strings, and nothing else."
    )
    """
    prompt = (
        f"Identify the single person located at the center of this image and "
        f"describe exactly {num_features} of their most salient clothing and "
        f"accessory features. Focus on clothing type, colors, patterns, sleeve length, "
        f"and accessories (like glasses or hats). Do NOT mention age, gender, "
        f"or facial expressions. Respond only with a JSON "
        f'object of the form {{"features": ["...", "..."]}} containing '
        f"exactly {num_features} short strings, and nothing else."
    )


    # Assemble the request payload.
    payload: dict[str, Any] = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {
                        "type": "image_url",
                        "image_url": {"url": data_url, "detail": detail},
                    },
                ],
            }
        ],
        "max_completion_tokens": max_tokens,
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

    # Parse the model output into a list of feature strings.
    try:
        content = body["choices"][0]["message"]["content"]
        features = json.loads(content)["features"]
    except (KeyError, IndexError, TypeError, json.JSONDecodeError) as exc:
        raise ValueError(f"Unexpected API response format: {exc}") from exc

    if not isinstance(features, list):
        raise ValueError("The 'features' field in the response is not a list.")

    return [str(feature) for feature in features]
