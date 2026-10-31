#!/usr/bin/env python3
"""Copies docs/ into a clone of the GitHub Wiki, rewriting the links for the Wiki.

docs/ is the source of the documentation; the Wiki is only a copy of it:

    git clone https://github.com/gaetanotorella/H1_2_Squat_Locomotion_RL.wiki.git ../wiki
    python3 scripts/publish_wiki.py ../wiki
    cd ../wiki && git add -A && git commit -m "Sync with docs/" && git push

Home.md is made from README.md and _Sidebar.md lists the pages in the README order. Links between
pages become Wiki links; links to the rest of the repository become links to GitHub. Pages of the
Wiki that no longer exist in docs/ are deleted.
"""
from __future__ import annotations

import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
TREE = "https://github.com/gaetanotorella/H1_2_Squat_Locomotion_RL/tree/main/"
RAW = "https://raw.githubusercontent.com/gaetanotorella/H1_2_Squat_Locomotion_RL/main/"
LINK = re.compile(r"\]\(([^)\s]+)\)")
SRC = re.compile(r'src="([^"]+)"')
MEDIA = (".png", ".jpg", ".jpeg", ".gif", ".svg", ".mp4")


def page_name(md: Path) -> str:
    """training_environment.md -> Training-environment (shown by the Wiki as "Training environment")."""
    return md.stem.replace("_", "-").capitalize()


def rewrite_target(target: str, folder: Path) -> str:
    """A relative link of a Markdown file in `folder`, as seen from the Wiki."""
    if re.match(r"[a-z]+:", target) or target.startswith("#"):
        return target
    path, _, anchor = target.partition("#")
    rel = Path(os.path.normpath(folder / path)).relative_to(ROOT)
    if rel.parent == Path("docs") and rel.suffix == ".md":
        new = page_name(rel)
    elif rel.suffix.lower() in MEDIA:
        new = RAW + rel.as_posix()     # images are shown, not linked: the raw file
    else:
        new = TREE + rel.as_posix()
    return f"{new}#{anchor}" if anchor else new


def rewrite_links(text: str, folder: Path) -> str:
    """Rewrites the relative links and image sources of a Markdown file that lives in `folder`."""
    text = LINK.sub(lambda m: f"]({rewrite_target(m.group(1), folder)})", text)
    return SRC.sub(lambda m: f'src="{rewrite_target(m.group(1), folder)}"', text)


def without_title(text: str) -> str:
    """Drops the first "# Title" line: the Wiki already shows the page name as title."""
    lines = text.splitlines(keepends=True)
    if lines and lines[0].startswith("# "):
        lines = lines[1:]
        while lines and not lines[0].strip():
            lines = lines[1:]
    return "".join(lines)


def main() -> None:
    if len(sys.argv) != 2 or not (Path(sys.argv[1]) / ".git").exists():
        sys.exit(f"usage: {sys.argv[0]} <clone of the Wiki repository>")
    wiki = Path(sys.argv[1])

    readme = (ROOT / "README.md").read_text()
    order = [Path(p).name for p in re.findall(r"\]\((docs/[^)#]+\.md)\)", readme)]
    pages = sorted(DOCS.glob("*.md"), key=lambda p: (order.index(p.name) if p.name in order else len(order), p.name))

    written = {"Home.md": rewrite_links(readme, ROOT)}
    sidebar = ["**[Home](Home)**", ""]
    for i, md in enumerate(pages, 1):
        text = md.read_text()
        title = text.splitlines()[0].lstrip("# ").strip()
        written[page_name(md) + ".md"] = rewrite_links(without_title(text), DOCS)
        sidebar.append(f"{i}. [{title}]({page_name(md)})")
    written["_Sidebar.md"] = "\n".join(sidebar) + "\n"

    for old in wiki.glob("*.md"):
        if old.name not in written:
            old.unlink()
            print(f"deleted  {old.name}")
    for name, text in written.items():
        (wiki / name).write_text(text)
        print(f"written  {name}")


if __name__ == "__main__":
    main()
