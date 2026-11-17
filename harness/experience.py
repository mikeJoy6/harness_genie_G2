"""经验库:experience/*.md 每个文件一条经验(格式见 experience/README.md)。

SYSTEM 里只放目录(名字 + 什么时候用);大模型调用 read_experience(name) 才读全文。
大模型调用 propose_experience 写的新经验进 experience/pending/,人审核后才生效(scripts/experience.py)。
"""
import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FOLDER = ROOT / "experience"
NAME_PATTERN = re.compile(r"^[a-z0-9_]{3,40}$")


def parse(text):
    """'---\\n头部 yaml\\n---\\n正文' → (头部字典, 正文)。没有头部时头部为空字典。"""
    if not text.startswith("---"):
        return {}, text
    parts = text.split("---", 2)
    if len(parts) < 3:
        return {}, text
    head = yaml.safe_load(parts[1]) or {}
    return head, parts[2].strip()


class ExperienceBook:
    def __init__(self, folder=DEFAULT_FOLDER):
        self.folder = Path(folder)
        self.pending = self.folder / "pending"

    def entries(self):
        """正式的经验:[{"name", "when", "status", "path"}],按名字排序。README.md 不算。"""
        found = []
        if not self.folder.is_dir():
            return found
        for path in sorted(self.folder.glob("*.md")):
            if path.name == "README.md":
                continue
            head, _ = parse(path.read_text(encoding="utf-8"))
            found.append({"name": head.get("name", path.stem), "when": head.get("when", ""),
                          "status": head.get("status", ""), "path": path})
        return found

    def names(self):
        result = []
        for entry in self.entries():
            result.append(entry["name"])
        return result

    def catalog_text(self):
        """SYSTEM 里的目录:每条一行。"""
        lines = []
        for entry in self.entries():
            lines.append(f"- {entry['name']}:{entry['when']}")
        return "\n".join(lines)

    def read(self, name):
        for entry in self.entries():
            if entry["name"] == name:
                head, body = parse(entry["path"].read_text(encoding="utf-8"))
                return {"ok": True, "name": name, "when": head.get("when", ""), "status": head.get("status", ""),
                        "content": body}
        return {"ok": False, "error": f"没有经验 '{name}',目录里有:{self.names()}"}

    def propose(self, name, when, content, source=""):
        """大模型提议一条新经验(或改进旧的):写进 pending/,人审核后才生效。"""
        if not isinstance(name, str) or not NAME_PATTERN.match(name):
            return {"ok": False, "error": "name 只能用小写字母、数字、下划线,3~40 个字符,例如 pour_water"}
        if not isinstance(when, str) or not when.strip():
            return {"ok": False, "error": "when 要写一句话:什么时候用这条经验"}
        if not isinstance(content, str) or len(content.strip()) < 20:
            return {"ok": False, "error": "content 太短:要写原则、做法、常见坑、怎么确认成功"}
        self.pending.mkdir(parents=True, exist_ok=True)
        path = self.pending / f"{name}.md"
        head = {"name": name, "when": when.strip(), "status": "大模型提议,待人审核", "source": source}
        text = "---\n" + yaml.safe_dump(head, allow_unicode=True, sort_keys=False) + "---\n" + content.strip() + "\n"
        path.write_text(text, encoding="utf-8")
        replaces = name in self.names()
        return {"ok": True, "saved": str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),
                "note": ("会替换现有的同名经验," if replaces else "") + "人审核通过后才生效,这次任务里还用不上"}
