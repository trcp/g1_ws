#!/usr/bin/env python3
"""Local LLM guest name/drink extraction via Ollama."""
import argparse
import json
import os
import re
import sys
import urllib.error
import urllib.request

try:
    from word_sprit import search_keywords
except Exception:
    search_keywords = None


OLLAMA_URL = os.environ.get(
    "OLLAMA_URL", "http://127.0.0.1:11434/api/generate")
OLLAMA_MODEL = os.environ.get("OLLAMA_MODEL", "qwen2.5:7b")


def _log(node, level, message):
    if node is None:
        return
    logger = node.get_logger()
    getattr(logger, level)(message)


def _clean_value(value):
    if value is None:
        return ""
    text = str(value).strip().strip("\"'`.,:;!?")
    if text.lower() in ("", "unknown", "none", "null", "n/a", "not mentioned"):
        return ""
    return " ".join(part.capitalize() for part in text.split())


def _json_from_response(text):
    text = text.strip()
    if text.startswith("```"):
        text = re.sub(r"^```[a-zA-Z]*\s*", "", text)
        text = re.sub(r"\s*```$", "", text)
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", text, re.DOTALL)
        if not match:
            raise
        return json.loads(match.group(0))


def _call_ollama(prompt, keep_alive="5m", timeout=10.0, num_predict=96):
    payload = {
        "model": OLLAMA_MODEL,
        "prompt": prompt,
        "stream": False,
        "format": "json",
        "keep_alive": keep_alive,
        "options": {
            "temperature": 0.0,
            "num_predict": num_predict,
        },
    }
    req = urllib.request.Request(
        OLLAMA_URL,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as res:
        body = json.loads(res.read().decode("utf-8"))
    return body.get("response", "")


def activate_model(timeout=30.0):
    prompt = 'Return JSON only: {"ready": true}'
    _call_ollama(prompt, keep_alive="5m", timeout=timeout, num_predict=16)


def deactivate_model(timeout=5.0):
    _call_ollama("", keep_alive=0, timeout=timeout, num_predict=1)


def extract_guest_info_with_ollama(text, keep_alive=0, timeout=10.0):
    prompt = f"""
You extract information for a robot receptionist.

Return JSON only with exactly these keys:
{{"name": "", "drink": ""}}

Rules:
- Extract the arriving guest's first name.
- Extract the guest's favorite drink.
- Use an empty string for a missing field.
- Do not ask or answer confirmation questions.
- Do not explain.
- Preserve normal English capitalization.

Transcript:
{text}
""".strip()

    raw = _call_ollama(prompt, keep_alive=keep_alive, timeout=timeout)
    parsed = _json_from_response(raw)
    drink = (
        parsed.get("drink", "")
        or parsed.get("favorite_drink", "")
        or parsed.get("favorite drink", "")
    )
    return {
        "name": _clean_value(parsed.get("name", "")),
        "drink": _clean_value(drink),
    }


def extract_guest_info(text, fallback_dict=None, node=None,
                       keep_alive=0, timeout=10.0):
    result = {"name": "", "drink": ""}

    try:
        result.update(extract_guest_info_with_ollama(
            text, keep_alive=keep_alive, timeout=timeout))
        _log(node, "info", f"[LLM] Parsed guest info: {result}")
    except Exception as exc:
        _log(node, "warn", f"[LLM] Ollama extraction failed: {exc}")

    if fallback_dict and search_keywords:
        found = search_keywords(text, fallback_dict)
        if not result["name"] and found.get("name"):
            result["name"] = _clean_value(found["name"][0])
        if not result["drink"] and found.get("drink"):
            result["drink"] = _clean_value(found["drink"][0])

    return result


def _run_text(text, deactivate=False):
    info = extract_guest_info(text, fallback_dict=None, node=None)
    print(json.dumps(info, ensure_ascii=False))
    if deactivate:
        deactivate_model()


def _interactive(deactivate=False):
    print("Type a transcript. Ctrl-D to quit.", file=sys.stderr)
    for line in sys.stdin:
        text = line.strip()
        if not text:
            continue
        _run_text(text, deactivate=deactivate)


def main():
    parser = argparse.ArgumentParser(
        description="Extract HRI guest name/drink with local Ollama.")
    parser.add_argument("--text", help="Transcript text to parse.")
    parser.add_argument("--interactive", action="store_true",
                        help="Read transcript lines from stdin.")
    parser.add_argument("--activate", action="store_true",
                        help="Load the configured Ollama model.")
    parser.add_argument("--deactivate", action="store_true",
                        help="Unload the configured Ollama model after the call.")
    args = parser.parse_args()

    if args.activate:
        activate_model()
        print(json.dumps({"ready": True, "model": OLLAMA_MODEL}))
        if not args.text and not args.interactive and not args.deactivate:
            return

    if args.text:
        _run_text(args.text, deactivate=args.deactivate)
    elif args.interactive:
        _interactive(deactivate=args.deactivate)
    elif args.deactivate:
        deactivate_model()
        print(json.dumps({"deactivated": True, "model": OLLAMA_MODEL}))
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
