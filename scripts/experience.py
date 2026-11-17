"""看 / 审核经验(experience/)。大模型用 propose_experience 提的经验先进 experience/pending/,人审核后才生效。

    python3 scripts/experience.py list              # 正式的和待审的
    python3 scripts/experience.py show <名字>        # 看全文(先找待审的,再找正式的)
    python3 scripts/experience.py approve <名字>     # pending/ → 正式(同名的旧经验会被替换,旧的存成 <名字>.md.bak)
    python3 scripts/experience.py reject <名字>      # 删掉待审的
"""
import argparse
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from harness.experience import ExperienceBook, parse  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="看 / 审核经验")
    parser.add_argument("action", choices=["list", "show", "approve", "reject"])
    parser.add_argument("name", nargs="?")
    args = parser.parse_args()
    book = ExperienceBook(ROOT / "experience")

    if args.action == "list":
        print("正式:")
        for entry in book.entries():
            print(f"  {entry['name']:24} {entry['status']:28} {entry['when']}")
        pending = sorted(book.pending.glob("*.md")) if book.pending.is_dir() else []
        print("待审:" if pending else "待审:(没有)")
        for path in pending:
            head, _ = parse(path.read_text(encoding="utf-8"))
            note = "(会替换现有的)" if path.stem in book.names() else ""
            print(f"  {path.stem:24} {head.get('when', '')} {note}")
        return

    if not args.name:
        raise SystemExit("要给经验名字")
    pending_path = book.pending / f"{args.name}.md"
    final_path = book.folder / f"{args.name}.md"

    if args.action == "show":
        for path in (pending_path, final_path):
            if path.exists():
                print(f"# {path.relative_to(ROOT)}\n")
                print(path.read_text(encoding="utf-8"))
                return
        raise SystemExit(f"没有 {args.name}")

    if not pending_path.exists():
        raise SystemExit(f"待审里没有 {args.name}")
    if args.action == "reject":
        pending_path.unlink()
        print(f"已删掉待审的 {args.name}")
        return
    if final_path.exists():                              # approve:替换前留一份旧的
        backup = final_path.with_suffix(".md.bak")
        shutil.copy(final_path, backup)
        print(f"旧的存到 {backup.relative_to(ROOT)}")
    text = pending_path.read_text(encoding="utf-8").replace("status: 大模型提议,待人审核", "status: 人审核通过,未在真机验证")
    final_path.write_text(text, encoding="utf-8")
    pending_path.unlink()
    print(f"已生效:{final_path.relative_to(ROOT)}。可以再改改 status / source")


if __name__ == "__main__":
    main()
