#!/usr/bin/env python3
"""Enforce the closed repository layout from AGENTS.md, section "Where files go"."""

import os
import re
import subprocess
import unittest
from pathlib import Path
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[2]
RULES = "See AGENTS.md, section 'Where files go'."

ROOT_FILES = {"README.md", "AGENTS.md", "CLAUDE.md", ".gitignore", "firmware-build-environment.json"}
ROOT_DIRS = {"src", "scripts", "tests", "cad", "docs", "lab"}
SRC_DIRS = {"ES-02", "ros2_pc", "navbot_link", "libraries"}
SYSTEMS = {"robot", "ros2_pc", "integration"}
DOCS_DIRS = {"hardware", "software", "plans", "assets"}
MD_NAME = re.compile(r"^[a-z0-9]+(?:[-.][a-z0-9]+)*\.md$")
LAB_NOTE = re.compile(r"^\d{4}-\d{2}-\d{2}-[a-z0-9]+(?:-[a-z0-9]+)*\.md$")
STATUS = re.compile(r"^Status: (open|concluded)$", re.MULTILINE)
ROS_PACKAGE_TEST = re.compile(r"^src/ros2_pc/ros2/src/[^/]+/test/")
LINK = re.compile(r"\]\(([^)\s]+)\)|src=\"([^\"]+)\"")
GERMAN = re.compile(r"\b(und|nicht|wird|werden|ist|sind|der|das|mit|für|oder|eine|auf|bei|wurde)\b", re.IGNORECASE)


def repo_files():
    out = subprocess.run(
        ["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"],
        cwd=ROOT, capture_output=True, text=True, check=True,
    ).stdout
    return sorted(p for p in out.split("\0") if p and (ROOT / p).exists())


def markdown_pages(files):
    return [f for f in files if f.endswith(".md") and "/captures/" not in f]


def slug(heading):
    text = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", heading).replace("`", "").strip().lower()
    return re.sub(r"[^\w\- ]", "", text).replace(" ", "-")


def anchors(path):
    seen, result = {}, set()
    in_code = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("```"):
            in_code = not in_code
        if in_code:
            continue
        m = re.match(r"^#{1,6}\s+(.*?)\s*#*$", line)
        if m:
            base = slug(m.group(1))
            count = seen.get(base, 0)
            seen[base] = count + 1
            result.add(base if count == 0 else f"{base}-{count}")
        result.update(re.findall(r'<a (?:id|name)="([^"]+)"', line))
    return result


class RepoLayoutTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.files = repo_files()

    def test_top_level_entries(self):
        for f in self.files:
            top = f.split("/")[0]
            if "/" not in f:
                self.assertIn(f, ROOT_FILES, f"{f}: no loose files at the repository root. {RULES}")
            else:
                self.assertIn(top, ROOT_DIRS, f"{f}: '{top}/' is not an allowed top-level folder. {RULES}")

    def test_src_targets(self):
        for f in self.files:
            parts = f.split("/")
            if parts[0] == "src":
                self.assertIn(parts[1], SRC_DIRS, f"{f}: 'src/{parts[1]}/' is not an allowed target. {RULES}")

    def test_tests_live_in_tests(self):
        for f in self.files:
            parts = f.split("/")
            if parts[0] == "tests":
                self.assertGreater(len(parts), 2, f"{f}: put tests in tests/<area>/. {RULES}")
            elif parts[-1].startswith("test_") and not ROS_PACKAGE_TEST.match(f):
                self.fail(f"{f}: tests belong in tests/<area>/. {RULES}")

    def test_cad(self):
        for f in self.files:
            parts = f.split("/")
            if parts[0] != "cad":
                continue
            self.assertIn(parts[1], SYSTEMS, f"{f}: cad/ holds only system folders. {RULES}")
            self.assertLessEqual(len(parts), 4, f"{f}: cad/<system>/ is flat with one folder per part. {RULES}")
            self.assertFalse(f.endswith(".md"), f"{f}: no Markdown in cad/. {RULES}")

    def test_docs(self):
        for f in self.files:
            parts = f.split("/")
            if parts[0] != "docs":
                continue
            self.assertIn(parts[1], SYSTEMS, f"{f}: docs/ holds only system folders. {RULES}")
            if len(parts) == 3:
                self.assertEqual(parts[2], "README.md", f"{f}: docs/<system>/ holds only README.md and folders. {RULES}")
                continue
            self.assertEqual(len(parts), 4, f"{f}: folders below docs/<system>/ are flat. {RULES}")
            folder = parts[2]
            self.assertIn(folder, DOCS_DIRS, f"{f}: 'docs/{parts[1]}/{folder}/' is not an allowed folder. {RULES}")
            if folder == "assets":
                self.assertFalse(f.endswith(".md"), f"{f}: no Markdown in assets/. {RULES}")
            else:
                self.assertTrue(f.endswith(".md"), f"{f}: non-Markdown files go to docs/{parts[1]}/assets/. {RULES}")
                self.assertRegex(parts[3], MD_NAME, f"{f}: use lowercase kebab-case. {RULES}")

    def test_docs_pages_are_indexed(self):
        for f in self.files:
            parts = f.split("/")
            if parts[0] == "docs" and len(parts) == 4 and parts[2] in {"hardware", "software", "plans"}:
                index = ROOT / "docs" / parts[1] / "README.md"
                self.assertTrue(index.exists(), f"{f}: docs/{parts[1]}/README.md is missing. {RULES}")
                self.assertIn(f"({parts[2]}/{parts[3]}", index.read_text(encoding="utf-8"),
                              f"{f}: add the page to docs/{parts[1]}/README.md. {RULES}")

    def test_lab(self):
        for f in self.files:
            parts = f.split("/")
            if parts[0] != "lab":
                continue
            self.assertIn(parts[1], SYSTEMS, f"{f}: lab/ holds only system folders. {RULES}")
            if len(parts) > 3:
                self.assertEqual(parts[2], "captures", f"{f}: lab/<system>/ allows only the captures/ folder. {RULES}")
                continue
            self.assertRegex(parts[2], LAB_NOTE, f"{f}: name lab notes YYYY-MM-DD-topic.md. {RULES}")
            self.assertRegex((ROOT / f).read_text(encoding="utf-8"), STATUS,
                             f"{f}: add a line 'Status: open' or 'Status: concluded'. {RULES}")

    def test_no_spaces_in_paths(self):
        for f in self.files:
            self.assertNotIn(" ", f, f"{f}: no spaces in paths. {RULES}")

    def test_docs_and_lab_are_english(self):
        for f in markdown_pages(self.files):
            if f.split("/")[0] in ("docs", "lab"):
                hits = GERMAN.findall((ROOT / f).read_text(encoding="utf-8"))
                self.assertLess(len(hits), 10, f"{f}: write docs and lab notes in English ({len(hits)} German words). {RULES}")

    def test_relative_links_resolve(self):
        for f in markdown_pages(self.files):
            text = (ROOT / f).read_text(encoding="utf-8")
            for m in LINK.finditer(text):
                target = m.group(1) or m.group(2)
                if re.match(r"^[a-z][a-z0-9+.-]*:", target):
                    continue
                path_part, _, anchor = target.partition("#")
                path = (ROOT / f).parent / unquote(path_part) if path_part else ROOT / f
                self.assertTrue(path.exists(), f"{f}: broken link '{target}'.")
                if anchor and path.suffix == ".md":
                    self.assertIn(anchor, anchors(path), f"{f}: link '{target}' points to a missing heading.")


if __name__ == "__main__":
    os.chdir(ROOT)
    unittest.main(verbosity=1)
