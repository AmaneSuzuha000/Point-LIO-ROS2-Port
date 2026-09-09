#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1 参数 YAML -> ROS2。
section 和 key 都写成 4 空格缩进，YAML 解析出来是扁平的，
rclcpp 会去读 "satu_gyro" 而不是 "mapping.satu_gyro"，参数全部落回默认值。
"""
import io, os, re
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "point_lio_ros2")

src = io.open(os.path.join(D, "src/parameters.cpp"), encoding="utf-8").read()
types = {}
# 类型名里可能带嵌套模板参数（std::vector<double>），[^>]+ 会在第一个 > 处截断，
# 导致 4 个数组型参数根本没进类型表、YAML 里的整数数组没被改成浮点数组。
PAT = r'get_param<\s*((?:[^<>]|<[^<>]*>)+?)\s*>\(\s*nh\s*,\s*"([^"]+)"'
for T, key in re.findall(PAT, src):
    T = T.strip()
    T = {"std::string": "string", "std::vector<double>": "double_array"}.get(T, T)
    types.setdefault(key, T)

def coerce(val, T):
    if T in ("double", "float"):
        s = val.strip()
        return s + ".0" if re.fullmatch(r"-?\d+", s) else s
    if T == "double_array":
        inner = val.strip()
        if not (inner.startswith("[") and inner.endswith("]")): return val
        parts = [p.strip() for p in inner[1:-1].split(",") if p.strip()]
        return "[" + ", ".join(p + ".0" if re.fullmatch(r"-?\d+", p) else p for p in parts) + "]"
    return val

def logical_lines(raw):
    """把 flow 数组的续行并回上一行（只处理 [ ... ] 未闭合的情况）。
    括号深度必须对已剥离行内注释的整段缓冲重新赋值计算：
    早先用 += 累加会把已数过的括号重复计入，extrinsic_R 那种三行数组
    永远闭合不了，后面所有键被并进同一行。"""
    out, buf = [], ""
    for ln in raw.split("\n"):
        if not ln.strip():
            continue
        buf = ln.rstrip() if not buf else buf + " " + ln.strip()
        code = buf.split("#")[0]
        depth = code.count("[") - code.count("]")
        if depth <= 0:
            out.append(buf); buf = ""
    if buf: out.append(buf)
    return out

def convert(path):
    out = ["point_lio:", "  ros__parameters:"]
    section, n_c = None, 0
    for ln in logical_lines(io.open(path, encoding="utf-8").read()):
        m = re.match(r"^([A-Za-z_]\w*):\s*$", ln)
        if m:
            section = m.group(1); out.append("    %s:" % section)
            continue
        m = re.match(r"^\s*([A-Za-z_]\w*):\s*(.*)$", ln)
        if not m:
            continue
        key, rest = m.group(1), m.group(2)
        comment = ""
        if "#" in rest:
            i = rest.index("#"); comment = "   " + rest[i:].rstrip(); rest = rest[:i].rstrip()
        full = "%s.%s" % (section, key) if section else key
        T = types.get(full) or types.get(key)
        nr = coerce(rest, T) if T else rest
        if T == "double_array" and nr == rest and "[" in rest:
            nr = coerce(rest, T)
        if nr != rest: n_c += 1
        out.append("      %s: %s%s" % (key, nr, comment))
    io.open(path, "w", encoding="utf-8", newline="\n").write("\n".join(out) + "\n")
    print("  %-14s 类型改写 %d 处" % (os.path.basename(path), n_c))

for f in sorted(os.listdir(os.path.join(D, "config"))):
    if f.endswith(".yaml"): convert(os.path.join(D, "config", f))
