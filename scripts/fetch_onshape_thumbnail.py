#!/usr/bin/env python3
"""Download the Onshape thumbnail for a specific Assembly version."""

import argparse
import base64
import datetime as dt
import hashlib
import hmac
import json
import os
import secrets
import string
import struct
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def api_get(base_url: str, path: str, accept: str = "application/json") -> tuple[bytes, str]:
    access_key = os.environ.get("ONSHAPE_ACCESS_KEY")
    secret_key = os.environ.get("ONSHAPE_SECRET_KEY")
    if not access_key or not secret_key:
        raise RuntimeError("Set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY first.")

    nonce = "".join(secrets.choice(string.ascii_letters + string.digits) for _ in range(25))
    date = dt.datetime.now(dt.timezone.utc).strftime("%a, %d %b %Y %H:%M:%S GMT")
    signature_message = f"GET\n{nonce}\n{date}\napplication/json\n{path}\n\n".lower().encode()
    signature = base64.b64encode(
        hmac.new(secret_key.encode(), signature_message, hashlib.sha256).digest()
    ).decode("ascii")
    request = urllib.request.Request(
        f"{base_url.rstrip('/')}{path}",
        headers={
            "Date": date,
            "On-Nonce": nonce,
            "Authorization": f"On {access_key}:HmacSHA256:{signature}",
            "Accept": accept,
            "Content-Type": "application/json",
        },
    )
    try:
        with urllib.request.urlopen(request, timeout=45) as response:
            return response.read(), response.headers.get("Content-Type", "")
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"Onshape API returned HTTP {error.code} for {path}.") from None
    except urllib.error.URLError as error:
        raise RuntimeError(f"Could not reach Onshape API: {error.reason}") from None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--document-id", required=True)
    parser.add_argument("--version-id", required=True)
    parser.add_argument("--element-id", required=True)
    parser.add_argument("--version-name", required=True)
    args = parser.parse_args()

    try:
        base_url = os.environ.get("ONSHAPE_API", "https://cad.onshape.com")
        metadata_path = (
            f"/api/v10/thumbnails/d/{args.document_id}/v/{args.version_id}/e/{args.element_id}"
        )
        metadata_bytes, _ = api_get(base_url, metadata_path)
        metadata = json.loads(metadata_bytes)
        sizes = metadata.get("sizes", [])
        selected = next((item for item in sizes if item.get("size") == "600x340"), None)
        if selected is None and sizes:
            selected = sizes[0]
        if selected is None or not selected.get("href"):
            raise RuntimeError("Onshape returned no thumbnail for the requested Assembly.")

        image_url = urllib.parse.urlparse(selected["href"])
        image_path = image_url.path
        if image_url.query:
            image_path += f"?{image_url.query}"
        image, content_type = api_get(base_url, image_path, "image/*")
        if not image.startswith(b"\x89PNG\r\n\x1a\n"):
            raise RuntimeError(f"Expected PNG thumbnail, got {content_type or 'unknown content type'}.")
        width, height = struct.unpack(">II", image[16:24])

        output_path = ROOT / "sdf" / "robot.png"
        output_path.write_bytes(image)

        print(f"Saved Onshape Assembly thumbnail for {args.version_name}: {output_path} ({width}x{height})")
        return 0
    except (KeyError, ValueError, RuntimeError, struct.error) as error:
        print(f"Onshape thumbnail download failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
