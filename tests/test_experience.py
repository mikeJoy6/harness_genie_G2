"""经验库(harness/experience.py)和经验工具。不需要 ROS。

    python3 -m pytest tests/test_experience.py -q
"""
import re

import pytest
import yaml

from harness.backends.fake import FakeBackend
from harness.experience import DEFAULT_FOLDER, ExperienceBook
from harness.gateway import Gateway
from harness.loop import SYSTEM_BASE, build_system
from harness.tools.experience import make_experience_tools
from harness.tools.objects import make_tools

MOTION = {"max_moves_per_command": 8, "max_wait_s": 30}
SECTIONS = ("## 原则", "## 做法", "## 常见坑", "## 怎么确认成功")


def write(folder, name, when, body="## 原则\n- 慢一点\n## 做法\n1. 先找\n## 常见坑\n- 无\n## 怎么确认成功\n- 看"):
    (folder / f"{name}.md").write_text(f"---\nname: {name}\nwhen: {when}\nstatus: 测试\n---\n{body}\n",
                                       encoding="utf-8")


def test_catalog_and_read(tmp_path):
    write(tmp_path, "pick_up_object", "拿起东西")
    write(tmp_path, "wave_hand", "挥手")
    (tmp_path / "README.md").write_text("说明,不是经验", encoding="utf-8")
    book = ExperienceBook(tmp_path)
    assert book.names() == ["pick_up_object", "wave_hand"]
    assert book.catalog_text() == "- pick_up_object:拿起东西\n- wave_hand:挥手"
    got = book.read("wave_hand")
    assert got["ok"] and got["when"] == "挥手" and "## 做法" in got["content"] and "---" not in got["content"]
    missing = book.read("fly")
    assert not missing["ok"] and "pick_up_object" in missing["error"]


def test_propose_goes_to_pending_not_catalog(tmp_path):
    book = ExperienceBook(tmp_path)
    result = book.propose("pour_water", "倒水", "## 原则\n- 杯口朝上\n## 做法\n1. 慢慢倒\n## 常见坑\n- 洒\n## 怎么确认成功\n- 看杯子")
    assert result["ok"] and "审核" in result["note"]
    assert (tmp_path / "pending" / "pour_water.md").exists()
    assert book.names() == []                                  # 待审的不进目录
    assert not book.propose("Pour Water!", "倒水", "x" * 30)["ok"]
    assert not book.propose("pour_water", "", "x" * 30)["ok"]
    assert not book.propose("pour_water", "倒水", "太短")["ok"]


def test_gateway_handlers_do_not_move(tmp_path):
    write(tmp_path, "wave_hand", "挥手")
    gateway = Gateway(FakeBackend(), MOTION, experience=ExperienceBook(tmp_path))
    assert gateway.execute("read_experience", {"name": "wave_hand"})["ok"]
    assert gateway.execute("propose_experience", {"name": "nod_head", "when": "点头",
                                                  "content": "## 原则\n- 慢\n## 做法\n1. 点头两次"})["ok"]
    assert gateway.moves == 0
    assert not Gateway(FakeBackend(), MOTION).execute("read_experience", {"name": "wave_hand"})["ok"]


def test_system_has_catalog_only_when_given():
    assert build_system() == SYSTEM_BASE
    text = build_system("- a:甲")
    assert text.startswith(SYSTEM_BASE) and text.endswith("# 经验目录\n- a:甲")


def test_tools():
    tools = make_experience_tools(["a", "b"])
    read = tools[0]["function"]
    assert read["name"] == "read_experience" and read["parameters"]["properties"]["name"]["enum"] == ["a", "b"]
    assert [t["function"]["name"] for t in make_experience_tools([])] == ["propose_experience"]


# ---------- 仓库里真的经验文件:格式对、提到的工具都存在 ----------
REAL = ExperienceBook(DEFAULT_FOLDER)
TOOL_NAMES = {t["function"]["name"] for t in make_tools(["init"]) + make_experience_tools(["x"])}


@pytest.mark.parametrize("entry", REAL.entries(), ids=lambda e: e["name"])
def test_real_experience_files(entry):
    assert entry["name"] == entry["path"].stem, "name 要和文件名一致"
    assert entry["when"] and entry["status"]
    content = REAL.read(entry["name"])["content"]
    for section in SECTIONS:
        assert section in content, f"{entry['name']} 缺 {section}"
    called = set(re.findall(r"\b([a-z_]+)\(", content))         # 正文里写的 xxx( 都要是真工具
    unknown = {name for name in called if name not in TOOL_NAMES}
    assert not unknown, f"{entry['name']} 提到了不存在的工具 {unknown}"


# ---------- 手里拿着什么(按夹爪动作推测) ----------
REAL_MOTION = yaml.safe_load((DEFAULT_FOLDER.parent / "configs/harness.yaml").read_text(encoding="utf-8"))["motion"]


def holding_gateway():
    backend = FakeBackend(sam_objects={"mouse": [(0.65, -0.20, 0.72)]})
    gateway = Gateway(backend, REAL_MOTION)
    assert gateway.execute("find_objects", {"query": "mouse"})["ok"]
    return gateway


def test_holding_after_touch_then_close():
    gateway = holding_gateway()
    assert gateway.execute("touch_object", {"object_id": "mouse_1", "arm": "right", "gap_cm": 0})["executed"]
    closed = gateway.execute("gripper", {"arm": "right", "action": "close"})
    assert closed["holding"]["object_id"] == "mouse_1"
    assert gateway.execute("get_robot_state", {})["holding"]["right"]["object_id"] == "mouse_1"
    assert gateway.execute("gripper", {"arm": "right", "action": "open"})["holding"] is None


def test_close_without_touch_is_unknown():
    gateway = holding_gateway()
    closed = gateway.execute("gripper", {"arm": "right", "action": "close"})
    assert closed["holding"]["object_id"] is None and "不知道" in closed["holding"]["basis"]
