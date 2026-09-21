"""Generate a 16x16 Chinese font table for the MC-02 LCD dashboard.

The generated C array is indexed by an enum, so the MCU never needs to do
UTF-8/GB2312 conversion at runtime. Regenerate after changing the label set.
"""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


FONT_PATH = r"C:\Windows\Fonts\simsun.ttc"
OUT_DIR = Path(__file__).resolve().parents[2] / "modules" / "lcd"

# (enum suffix, character) in the fixed order used by the generated enum.
CHARS = [
    ("XI", "系"), ("TONG", "统"), ("AN", "安"), ("QUAN", "全"),
    ("ZHUANG", "状"), ("TAI", "态"), ("GU", "故"), ("ZHANG", "障"),
    ("JI", "急"), ("TING", "停"), ("DIAN", "电"), ("YA", "压"),
    ("REN", "任"), ("WU", "务"), ("YAO", "遥"), ("KONG", "控"),
    ("SHI", "视"), ("JUE", "觉"), ("KAN", "看"), ("MEN", "门"),
    ("GOU", "狗"), ("WEN", "温"), ("DU", "度"), ("MU", "目"),
    ("BIAO", "标"), ("ZHAN", "占"), ("KONG2", "空"), ("JIA", "加"),
    ("RE", "热"), ("ZI", "姿"), ("YOU", "有"), ("XIAO", "效"),
    ("CAI", "裁"), ("PAN", "判"), ("JIE", "阶"), ("DUAN", "段"),
    ("XUE", "血"), ("LIANG", "量"), ("GONG", "功"), ("LV", "率"),
    ("HUAN", "缓"), ("CHONG", "冲"), ("TAN", "弹"), ("SU", "速"),
    ("JIN", "金"), ("BI", "币"), ("DI", "底"), ("PAN2", "盘"),
    ("YUN", "云"), ("FA", "发"), ("SHE", "射"), ("ZAI", "在"),
    ("XIAN", "线"), ("LI", "离"), ("ZONG", "总"), ("LIAN", "链"),
    ("LU", "路"), ("DANG", "当"), ("QIAN", "前"), ("JI2", "机"),
    ("DING", "定"), ("CHU", "初"), ("SHI2", "始"), ("HUA", "化"),
    ("SHI3", "是"), ("FOU", "否"), ("WU2", "无"), ("KAI", "开"),
    ("GUAN", "关"), ("SHU", "数"), ("JU", "据"), ("SHI4", "时"),
    ("JIAN", "间"), ("YE", "页"), ("SHANG", "上"), ("XIA", "下"),
    ("DONG", "冻"), ("JIE2", "结"), ("HUI", "恢"), ("FU", "复"),
    ("CUO", "错"), ("WU3", "误"),
]

LABELS = {
    "SYS": ["XI", "TONG"],
    "SAFE": ["AN", "QUAN"],
    "STATE": ["ZHUANG", "TAI"],
    "FAULT": ["GU", "ZHANG"],
    "ESTOP": ["JI", "TING"],
    "VOLT": ["DIAN", "YA"],
    "TASK": ["REN", "WU"],
    "RC": ["YAO", "KONG"],
    "VISION": ["SHI", "JUE"],
    "WATCHDOG": ["KAN", "MEN", "GOU"],
    "TEMP": ["WEN", "DU"],
    "TARGET": ["MU", "BIAO"],
    "DUTY": ["ZHAN", "KONG2"],
    "HEAT": ["JIA", "RE"],
    "ATTITUDE": ["ZI", "TAI"],
    "VALID": ["YOU", "XIAO"],
    "REFEREE": ["CAI", "PAN"],
    "STAGE": ["JIE", "DUAN"],
    "HP": ["XUE", "LIANG"],
    "POWER": ["GONG", "LV"],
    "BUFFER": ["HUAN", "CHONG"],
    "HEATQ": ["RE", "LIANG"],
    "AMMO": ["TAN", "LIANG"],
    "BULLET_SPEED": ["TAN", "SU"],
    "COIN": ["JIN", "BI"],
    "CHASSIS": ["DI", "PAN2"],
    "GIMBAL": ["YUN", "TAI"],
    "SHOOT": ["FA", "SHE"],
    "ONLINE": ["ZAI", "XIAN"],
    "OFFLINE": ["LI", "XIAN"],
    "BUS": ["ZONG", "XIAN"],
    "LINK": ["LIAN", "LU"],
    "CURRENT": ["DANG", "QIAN"],
    "MOTOR": ["DIAN", "JI2"],
    "INIT": ["CHU", "SHI2", "HUA"],
    "READY": ["JIU", "XU"],
    "CALIB": ["BIAO", "DING"],
    "YES": ["SHI3"],
    "NO": ["FOU"],
    "NONE": ["WU2"],
    "OPEN": ["KAI"],
    "CLOSE": ["GUAN"],
    "PAGE": ["YE"],
    "PREV": ["SHANG", "YE"],
    "NEXT": ["XIA", "YE"],
    "FREEZE": ["DONG", "JIE2"],
    "RESUME": ["HUI", "FU"],
    "ERROR": ["CUO", "WU3"],
}

# Some characters used only by labels below are not in the base list yet.
CHARS += [("JIU", "就"), ("XU", "绪")]

# Keep enum order stable and unique.
seen = {}
ordered = []
for suffix, ch in CHARS:
    if ch in seen:
        continue
    seen[ch] = suffix
    ordered.append((suffix, ch))


def render_char(ch: str) -> list[int]:
    image = Image.new("L", (16, 16), 0)
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype(FONT_PATH, 15)
    draw.text((8, 8), ch, fill=255, font=font, anchor="mm")

    # Match the example LCD font format: column-major, 2 bytes per column,
    # LSB = top pixel of that column.
    data = []
    for col in range(16):
        lo = 0
        hi = 0
        for row in range(8):
            if image.getpixel((col, row)) >= 128:
                lo |= 1 << row
        for row in range(8, 16):
            if image.getpixel((col, row)) >= 128:
                hi |= 1 << (row - 8)
        data.append(lo)
        data.append(hi)
    return data


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    enum_lines = ["typedef enum", "{"]
    for suffix, _ in ordered:
        enum_lines.append(f"    LCD_CN_{suffix},")
    enum_lines.append("    LCD_CN_COUNT,")
    enum_lines.append("} Lcd_CnChar_e;")

    label_lines = []
    for name, suffix_list in LABELS.items():
        entries = ", ".join(f"LCD_CN_{s}" for s in suffix_list)
        label_lines.append(f"#define LCD_LABEL_{name} {{ {entries}, LCD_CN_COUNT }}")

    header = [
        "#ifndef LCD_FONT_CN_H",
        "#define LCD_FONT_CN_H",
        "",
        "#include <stdint.h>",
        "",
        *enum_lines,
        "",
        "extern const uint8_t lcd_cn_font16[LCD_CN_COUNT][32];",
        "",
        "/* 常用标签的枚举序列，末尾 LCD_CN_COUNT 作为结束标记 */",
        *label_lines,
        "",
        "#endif // LCD_FONT_CN_H",
        "",
    ]
    (OUT_DIR / "lcd_font_cn.h").write_text("\n".join(header), encoding="utf-8")

    body = [
        '#include "lcd_font_cn.h"',
        "",
        "const uint8_t lcd_cn_font16[LCD_CN_COUNT][32] = {",
    ]
    for idx, (suffix, ch) in enumerate(ordered):
        body.append(f"    /* {ch} ({suffix}) */")
        body.append("    {")
        data = render_char(ch)
        for i in range(0, len(data), 8):
            body.append("        " + ", ".join(f"0x{b:02X}" for b in data[i:i + 8]) + ",")
        body.append("    },")
    body.append("};")
    body.append("")
    (OUT_DIR / "lcd_font_cn.c").write_text("\n".join(body), encoding="utf-8")

    print(f"generated {len(ordered)} glyphs -> {OUT_DIR}")


if __name__ == "__main__":
    main()
