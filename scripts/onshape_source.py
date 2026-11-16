#!/usr/bin/env python3
"""Resolve the document's current workspace and matching saved version."""

import argparse
import base64
import datetime as dt
import hashlib
import hmac
import json
import os
import secrets
import string
import sys
import urllib.error
import urllib.parse
import urllib.request
import uuid
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "sdf" / "config.json"


def onshape_get(base_url: str, path: str, query: dict[str, object] | None = None):
    access_key = os.environ.get("ONSHAPE_ACCESS_KEY")
    secret_key = os.environ.get("ONSHAPE_SECRET_KEY")
    if not access_key or not secret_key:
        raise RuntimeError("Set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY first.")

    query = query or {}
    encoded_query = urllib.parse.urlencode(query)
    nonce = "".join(secrets.choice(string.ascii_letters + string.digits) for _ in range(25))
    date = dt.datetime.now(dt.timezone.utc).strftime("%a, %d %b %Y %H:%M:%S GMT")
    signature_message = (
        f"GET\n{nonce}\n{date}\napplication/json\n{path}\n{encoded_query}\n"
    ).lower().encode("utf-8")
    signature = base64.b64encode(
        hmac.new(secret_key.encode("utf-8"), signature_message, hashlib.sha256).digest()
    ).decode("ascii")

    request = urllib.request.Request(
        f"{base_url.rstrip('/')}{path}" + (f"?{encoded_query}" if encoded_query else ""),
        headers={
            "Date": date,
            "On-Nonce": nonce,
            "Authorization": f"On {access_key}:HmacSHA256:{signature}",
            "Accept": "application/json",
            "Content-Type": "application/json",
        },
    )
    try:
        with urllib.request.urlopen(request, timeout=45) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"Onshape API returned HTTP {error.code} for {path}.") from None
    except urllib.error.URLError as error:
        raise RuntimeError(f"Could not reach Onshape API: {error.reason}") from None


def write_github_outputs(values: dict[str, str]) -> None:
    output_path = os.environ.get("GITHUB_OUTPUT")
    if not output_path:
        return
    with open(output_path, "a", encoding="utf-8") as output:
        for key, value in values.items():
            delimiter = f"ONSHAPE_OUTPUT_{uuid.uuid4().hex}"
            output.write(f"{key}<<{delimiter}\n{value}\n{delimiter}\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="query Onshape and report the source without changing config.json",
    )
    args = parser.parse_args()

    try:
        config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        base_url = os.environ.get("ONSHAPE_API", "https://cad.onshape.com")
        document_id = config.get("document_id", config.get("documentId"))
        if not document_id:
            raise RuntimeError("Set document_id in sdf/config.json.")

        document = onshape_get(base_url, f"/api/v10/documents/{document_id}")
        workspace_id = config.get("workspace_id")
        workspace_name = config.get("workspace_name")
        if not workspace_id:
            raise RuntimeError("Set workspace_id in config.json to select the export branch.")
        workspace = onshape_get(
            base_url,
            f"/api/v10/documents/d/{document_id}/workspaces/{workspace_id}",
        )
        if workspace.get("name") != workspace_name:
            raise RuntimeError(
                f"Configured workspace ID resolves to {workspace.get('name')!r}, "
                f"not the expected branch {workspace_name!r}. Update config.json."
            )

        versions = onshape_get(
            base_url,
            f"/api/v10/documents/d/{document_id}/versions",
            {"offset": 0, "limit": 0},
        )
        if isinstance(versions, dict):
            versions = versions.get("items", [])
        if not isinstance(versions, list):
            raise RuntimeError("Unexpected response while listing document versions.")

        versions_by_id = {version.get("id"): version for version in versions}
        parent_version = versions_by_id.get(workspace.get("parent"))
        # Onshape's workspace microversion is not a reliable way to identify the
        # version from which a branch was created. Use the explicit history edge
        # (workspace.parent), and only consider it the current export state when
        # the workspace has not been modified after that version was saved.
        matching_version = None
        if parent_version:
            workspace_modified = workspace.get("modifiedAt", "")
            version_created = parent_version.get("createdAt", "")
            if workspace_modified and version_created and workspace_modified <= version_created:
                matching_version = parent_version

        if matching_version:
            config.pop("workspace_id", None)
            config["version_id"] = matching_version["id"]
            source_kind = "version"
            source_id = matching_version["id"]
            source_name = matching_version.get("name", "")
            version_id = source_id
            workspace_id = ""
            git_tag = source_name
        else:
            config.pop("version_id", None)
            config["workspace_id"] = workspace["id"]
            source_kind = "workspace"
            source_id = workspace["id"]
            source_name = workspace.get("name", "")
            version_id = ""
            workspace_id = source_id
            git_tag = ""

        wv = "v" if version_id else "w"
        wvid = version_id or workspace["id"]
        elements = onshape_get(
            base_url,
            f"/api/v10/documents/d/{document_id}/{wv}/{wvid}/elements",
        )
        assembly_name = config.get("assembly_name", config.get("assemblyName", "robot"))
        assemblies = [
            element
            for element in elements
            if element.get("name") == assembly_name
            and element.get("elementType") == "ASSEMBLY"
        ]
        if len(assemblies) != 1:
            raise RuntimeError(
                f"Expected one Assembly tab named {assembly_name!r}; found {len(assemblies)}."
            )
        element_id = assemblies[0]["id"]

        if not args.check:
            CONFIG_PATH.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")

        outputs = {
            "source_kind": source_kind,
            "source_id": source_id,
            "source_name": source_name,
            "workspace_id": workspace_id,
            "version_id": version_id,
            "git_tag": git_tag,
            "branch_name": workspace.get("name", ""),
            "branch_workspace_id": workspace["id"],
            "element_id": element_id,
            "document_name": document.get("name", document_id),
            "document_id": document_id,
            "onshape_url": (
                f"{base_url.rstrip('/')}/documents/{document_id}/"
                f"{'v/' + version_id if version_id else 'w/' + workspace['id']}/"
                f"e/{element_id}"
            ),
        }
        if not args.check:
            write_github_outputs(outputs)

        print(f"Document: {document.get('name', document_id)} ({document_id})")
        print(f"Current workspace: {workspace.get('name', '')!r} ({workspace['id']})")
        print(f"Export source: {source_kind} {source_name!r} ({source_id})")
        if matching_version:
            print(f"Matching Onshape version found; Git tag will be {git_tag!r}.")
        else:
            print("Workspace has no matching saved version; export will not create a Git tag.")
        if args.check:
            print("Check only: config.json was not changed.")
        return 0
    except (KeyError, ValueError, RuntimeError) as error:
        print(f"Onshape source resolution failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
