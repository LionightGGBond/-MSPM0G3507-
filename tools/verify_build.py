"""用真实 SDK 生成临时配置并编译链接；不修改用户的 empty.syscfg。"""
from pathlib import Path
import argparse
import subprocess
import re
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]

def run(command, cwd):
    subprocess.run([str(x) for x in command], cwd=cwd, check=True)

def main():
    sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", type=Path, default=Path("D:/CCS/mspm0_sdk_2_11_00_07"))
    parser.add_argument("--compiler", type=Path, default=Path("D:/CCS/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS"))
    parser.add_argument("--sysconfig", type=Path, default=Path("D:/CCS/sysconfig_1.26.2"))
    parser.add_argument("--output", type=Path, required=True, help="使用工程目录以外的独立输出目录")
    parser.add_argument("--line-mode", action="store_true", help="完整编译原循迹模式")
    parser.add_argument("--without-uart", action="store_true", help="验证尚未添加调试UART的配置")
    args = parser.parse_args()
    out = args.output.resolve()
    if out == ROOT or ROOT in out.parents:
        parser.error("输出目录必须在 CCS 工程外，防止重复编译生成的 C 文件")
    out.mkdir(parents=True, exist_ok=True)
    temporary_cfg = out / "verification.syscfg"
    reference = (ROOT / "tools/sysconfig_reference.txt").read_text(encoding="utf-8")
    if args.without_uart:
        reference = reference.split('const UART = scripting.addModule')[0]
    temporary_cfg.write_text(reference, encoding="utf-8")
    run([args.sysconfig / "nodejs/node.exe", args.sysconfig / "dist/cli.js",
         "--compiler", "ticlang", "--product", args.sdk / ".metadata/product.json",
         "--output", out, temporary_cfg], out)
    compiler = args.compiler / "bin/tiarmclang.exe"
    flags = ["-mcpu=cortex-m0plus", "-march=thumbv6m", "-mthumb", "-mfloat-abi=soft",
             "-std=c11", "-O2", "-g", "-Wall", "-Wextra", "-Werror", "-DROBOT_STANDALONE_STARTUP=1",
             "@device.opt", "-I" + str(ROOT), "-I" + str(out),
             "-I" + str(args.sdk / "source"),
             "-I" + str(args.sdk / "source/third_party/CMSIS/Core/Include")]
    if args.line_mode:
        flags.append("-DROBOT_MOTOR_TEST_MODE=0")
    sources = [ROOT / "empty.c"]
    for folder in ["Motion", "Bsp", "Devices", "App"]:
        sources.extend(sorted((ROOT / folder).glob("*.c")))
    sources.extend(sorted(out.glob("*.c")))
    objects = []
    for index, source in enumerate(sources):
        obj = out / (str(index) + "_" + source.stem + ".o")
        run([compiler, *flags, "-c", source, "-o", obj], out)
        objects.append(obj)
    linker = out / "device_linker.cmd"
    run([compiler, "-mcpu=cortex-m0plus", "-mthumb", "-mfloat-abi=soft", *objects,
         "-Wl,-u,_c_int00", "-Wl,--rom_model", "-Wl,--warn_sections",
         "-Wl,-m,robot.map", "-L" + str(out), "-L" + str(args.sdk / "source"),
         "-L" + str(args.compiler / "lib"), "-ldevice.cmd.genlibs", linker, ROOT / "robot_link_options.cmd",
         "-llibc.a", "-o", out / "robot.out"], out)
    # 直接核对最终ELF向量表，避免只有ISR符号却未接入向量表的假通过。
    elf = (out / "robot.out").read_bytes()
    assert elf[:6] == b"\x7fELF\x01\x01", "期待小端ELF32固件"
    section_offset = struct.unpack_from("<I", elf, 32)[0]
    entry_size, section_count, names_index = struct.unpack_from("<HHH", elf, 46)
    sections = [struct.unpack_from("<10I", elf, section_offset + i * entry_size)
                for i in range(section_count)]
    names_header = sections[names_index]
    names = elf[names_header[4]:names_header[4] + names_header[5]]
    vector = next(s for s in sections if names[s[0]:].split(b"\0", 1)[0] == b".intvecs")
    words = struct.unpack_from("<" + "I" * (vector[5] // 4), elf, vector[4])
    map_text = (out / "robot.map").read_text(encoding="utf-8", errors="replace")
    for slot, handler in [(15, "SysTick_Handler"), (17, "GROUP1_IRQHandler"), (32, "TIMG0_IRQHandler")]:
        found = re.search(r"^([0-9a-fA-F]{8})\s+" + handler + r"\s*$", map_text, re.M)
        assert found and words[slot] == int(found[1], 16), "向量入口错误: " + handler
    stack = next(s for s in sections if names[s[0]:].split(b"\0", 1)[0] == b".stack")
    assert stack[5] == 2048, "栈应为2048字节"
    # 当前默认持续运行；同时核对可选的两项锁存停车分支，避免条件编译失效。
    line_flags = [f for f in flags if not f.startswith("-DROBOT_MOTOR_TEST_MODE=")]
    run([compiler, *line_flags, "-DROBOT_MOTOR_TEST_MODE=0", "-DROBOT_STOP_ON_INVALID_LINE=1",
         "-DROBOT_STOP_ON_ENCODER_STALL=1", "-fsyntax-only",
         ROOT / "Motion/controller.c"], out)
    print("PASS: SysConfig 引脚求解、全部源码严格编译和固件链接。")
    print("PASS: 最终向量入口、2 KB栈、循迹模式可选锁存停车分支编译。")
    print("输出:", out / "robot.out")

if __name__ == "__main__":
    main()
