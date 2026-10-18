#!/usr/bin/env python3
"""离线核对固定DSP文件及保留的Core头指纹；改动或文件缺失时返回非零，不修改源码。"""
import hashlib
import json
from pathlib import Path


def main():
    """读取仓库自带清单，逐文件比较原始字节，供开发机及持续集成调用。"""
    root = Path(__file__).resolve().parents[2]
    vendor = root / 'Middlewares/Third_Party/CMSIS-DSP-1.7.0'
    manifest = json.loads((vendor / 'provenance.json').read_text(encoding='utf-8'))
    if manifest['commit'] != 'a65b7c9a3e6502127fdb80eb288d8cbdf251a6f4':
        raise SystemExit('DSP来源提交不符')
    # 厂商文件检查原始字节；Core头可能由Git转换换行，清单保留其首次记录的原始指纹。
    for base, entries in ((vendor, manifest['files']), (root, manifest['existing_core_headers'])):
        for entry in entries:
            path = base / entry['path']
            data = path.read_bytes()
            actual = hashlib.sha256(data).hexdigest()
            # Core头保留已有版本，允许Git的CRLF/LF差异；厂商DSP仍按原始字节检查。
            lf_match = 'sha256_lf' in entry and hashlib.sha256(data.replace(b'\r\n', b'\n')).hexdigest() == entry['sha256_lf']
            if actual != entry['sha256'] and not lf_match:
                raise SystemExit('文件指纹不符：' + str(path))
    print('DSP来源文件及Core编译器头指纹一致')


if __name__ == '__main__':
    main()
