#!/usr/bin/env python3
"""从厂商编译好的 typesupport 库里反推出 .msg / .srv 的字段定义

**为什么需要它。** 开发机上挂的 `aimdk_msgs`（`~/aimdk/install`）比运控二进制
里带的那份**旧**。结果是：运控注册了某个服务，`ros2 service list` 也看得到，
但 `ros2 service call` 会报 "The passed service type is invalid" —— 本地没有
这个类型的定义。厂商发的包里只有编译产物，没有 .srv 源文件，也没有 Python 绑定。

2026-09-29 卡在这上面：`SetMcLocomotionSpeedMode` / `GetMcLocomotionSpeedStatus`
这两个服务只在运控那份 aimdk_msgs 里有，而不设速度档位机器人就一步都不走
（`none_speed` 档三个方向的上下界全是 0）。

好在 rosidl 的 **introspection typesupport** 库里存着完整的运行期反射信息：
字段名、类型、是不是数组、嵌套的子消息。这个脚本用 ctypes 把那些结构体走一遍，
按 .msg 的写法打印出来。拿到定义之后就能自己补一份 interface 文件重新生成。

用法
----
列出某个库里所有能反推的类型::

    tools/dump_vendor_idl.py --lib <introspection_c.so> --list

反推指定的服务 / 消息（不带包名，大小写照原样写）::

    tools/dump_vendor_idl.py --lib <introspection_c.so> \\
        --srv SetMcLocomotionSpeedMode --msg McLocomotionSpeedStatus

X2 的运控库在这里::

    ~/mc-x86-v1.0.0-20260522/lib/libaimdk_msgs__rosidl_typesupport_introspection_c.so

已知的坑
--------
1. **dlopen 之前要先 source /opt/ros/humble/setup.bash**，否则会报找不到
   `libgeometry_msgs__rosidl_typesupport_introspection_c.so` —— 厂商的库依赖
   上游的 introspection 库。**但别挂 `~/aimdk/install`**：那会把旧的
   `libaimdk_msgs__*` 拽进来，本脚本要读的恰恰是新的那份。
   所以把厂商的 lib 目录放在 `LD_LIBRARY_PATH` 最前面。
2. **字段顺序就是序列化顺序，一个都不能调。** 自己补 interface 文件时必须
   照这里打印的顺序写，否则类型名对得上、字节对不上，调用会静默地读到垃圾。
3. 嵌套消息只展开第一次出现的那处（后面同类型的只打类型名），避免递归类型
   把输出刷爆。所以看 `low_speed_status` 这种重复字段要往上翻。
4. **反推出来的定义不是权威的。** 它描述的是"这个二进制怎么序列化"，
   够用来通信，但默认值、注释、常量（`uint8 FOO=1` 这类）都拿不到 ——
   introspection 里 `default_value_` 基本是空的。真要准确还是得找厂商要源文件。
"""

import argparse
import ctypes as C
import subprocess
import sys

# rosidl_typesupport_introspection_c/field_types.h。非 0 的 members_ 指针才是
# 判断"是不是嵌套消息"的可靠依据，type_id 各版本对不上，只拿来认基础类型。
TYPE_NAMES = {
    1: 'float32', 2: 'float64', 3: 'long double', 4: 'char', 5: 'wchar',
    6: 'bool', 7: 'octet', 8: 'uint8', 9: 'int8', 10: 'uint16', 11: 'int16',
    12: 'uint32', 13: 'int32', 14: 'uint64', 15: 'int64', 16: 'string',
    17: 'wstring',
}


class MessageMember(C.Structure):
    """rosidl_typesupport_introspection_c__MessageMember

    字段顺序和类型必须和 Humble 的头文件一字不差，否则 ctypes 算出来的
    偏移量是错的，读出来的名字会是乱码或者直接段错误。
    """

    _fields_ = [
        ('name', C.c_char_p),
        ('type_id', C.c_uint8),
        ('string_upper_bound', C.c_size_t),
        ('members', C.c_void_p),
        ('is_array', C.c_bool),
        ('array_size', C.c_size_t),
        ('is_upper_bound', C.c_bool),
        ('offset', C.c_uint32),
        ('default_value', C.c_void_p),
        ('size_function', C.c_void_p),
        ('get_const_function', C.c_void_p),
        ('get_function', C.c_void_p),
        ('fetch_function', C.c_void_p),
        ('assign_function', C.c_void_p),
        ('resize_function', C.c_void_p),
    ]


class MessageMembers(C.Structure):
    """rosidl_typesupport_introspection_c__MessageMembers"""

    _fields_ = [
        ('message_namespace', C.c_char_p),
        ('message_name', C.c_char_p),
        ('member_count', C.c_uint32),
        ('size_of', C.c_size_t),
        ('members', C.POINTER(MessageMember)),
        ('init_function', C.c_void_p),
        ('fini_function', C.c_void_p),
    ]


class TypeSupport(C.Structure):
    """rosidl_message_type_support_t"""

    _fields_ = [
        ('typesupport_identifier', C.c_char_p),
        ('data', C.c_void_p),
        ('func', C.c_void_p),
    ]


def array_suffix(member):
    if not member.is_array:
        return ''
    if member.array_size == 0:
        return '[]'
    if member.is_upper_bound:
        return '[<=%d]' % member.array_size
    return '[%d]' % member.array_size


def walk(members_addr, out, indent='', seen=None):
    """把 MessageMembers 走一遍，按 .msg 的写法追加到 out。"""
    seen = set() if seen is None else seen
    members = C.cast(members_addr, C.POINTER(MessageMembers)).contents
    for i in range(members.member_count):
        field = members.members[i]
        name = field.name.decode()
        suffix = array_suffix(field)

        if not field.members:
            out.append('%s%s%s %s' % (indent, TYPE_NAMES.get(
                field.type_id, '<未知 type_id=%d>' % field.type_id), suffix, name))
            continue

        nested_ts = C.cast(field.members, C.POINTER(TypeSupport)).contents
        nested = C.cast(nested_ts.data, C.POINTER(MessageMembers)).contents
        type_name = '%s/%s' % (nested.message_namespace.decode().replace('__', '/'),
                               nested.message_name.decode())
        out.append('%s%s%s %s' % (indent, type_name, suffix, name))
        # 坑 3：同一个嵌套类型只展开一次
        if type_name not in seen:
            seen.add(type_name)
            walk(nested_ts.data, out, indent + '    ', seen)


def handle(lib, symbol):
    try:
        fn = getattr(lib, symbol)
    except AttributeError:
        return None
    fn.restype = C.POINTER(TypeSupport)
    fn.argtypes = []
    return fn().contents.data


def dump_message(lib, pkg, name):
    addr = handle(lib, 'rosidl_typesupport_introspection_c'
                       '__get_message_type_support_handle__%s__msg__%s' % (pkg, name))
    if addr is None:
        print('✗ 库里没有消息 %s/msg/%s' % (pkg, name))
        return False
    print('# %s/msg/%s.msg' % (pkg, name))
    lines = []
    walk(addr, lines)
    print('\n'.join(lines))
    print()
    return True


def dump_service(lib, pkg, name):
    parts = {}
    for part in ('Request', 'Response'):
        addr = handle(lib, 'rosidl_typesupport_introspection_c'
                           '__get_message_type_support_handle__%s__srv__%s_%s'
                           % (pkg, name, part))
        if addr is None:
            print('✗ 库里没有服务 %s/srv/%s（缺 %s）' % (pkg, name, part))
            return False
        lines = []
        walk(addr, lines)
        parts[part] = lines
    print('# %s/srv/%s.srv' % (pkg, name))
    print('\n'.join(parts['Request']))
    print('---')
    print('\n'.join(parts['Response']))
    print()
    return True


def list_types(path, pkg):
    """从符号表里列出所有可反推的类型。nm 比 dlopen 便宜，也不用管依赖。"""
    try:
        out = subprocess.run(['nm', '-D', '--defined-only', path],
                             capture_output=True, text=True, timeout=120).stdout
    except (OSError, subprocess.SubprocessError) as exc:
        print('✗ 跑 nm 失败：%s' % exc)
        return 1
    prefix = ('rosidl_typesupport_introspection_c'
              '__get_message_type_support_handle__%s__' % pkg)
    msgs, srvs = set(), set()
    for line in out.splitlines():
        sym = line.split()[-1]
        if not sym.startswith(prefix):
            continue
        rest = sym[len(prefix):]
        kind, _, tail = rest.partition('__')
        if kind == 'msg':
            msgs.add(tail)
        elif kind == 'srv':
            # Request / Response 两个句柄归到同一个服务名下
            for part in ('_Request', '_Response'):
                if tail.endswith(part):
                    srvs.add(tail[:-len(part)])
                    break
    if not msgs and not srvs:
        print('✗ 符号表里没有 %s 的 introspection 句柄。库选错了？' % pkg)
        return 1
    print('# %s 的消息（%d 个）' % (pkg, len(msgs)))
    for m in sorted(msgs):
        print('  %s' % m)
    print('\n# %s 的服务（%d 个）' % (pkg, len(srvs)))
    for s in sorted(srvs):
        print('  %s' % s)
    return 0


def main():
    ap = argparse.ArgumentParser(
        description='从厂商 introspection typesupport 库反推 msg/srv 定义')
    ap.add_argument('--lib', required=True,
                    help='...__rosidl_typesupport_introspection_c.so 的路径')
    ap.add_argument('--package', default='aimdk_msgs', help='接口包名')
    ap.add_argument('--msg', action='append', default=[], help='要反推的消息名，可多次')
    ap.add_argument('--srv', action='append', default=[], help='要反推的服务名，可多次')
    ap.add_argument('--list', action='store_true', help='只列出库里有哪些类型')
    args = ap.parse_args()

    if args.list:
        return list_types(args.lib, args.package)

    if not args.msg and not args.srv:
        print('没指定要反推什么。用 --list 先看看库里有哪些类型。')
        return 2

    try:
        lib = C.CDLL(args.lib)
    except OSError as exc:
        print('✗ dlopen 失败：%s' % exc)
        print('  见模块头部坑 1：先 source /opt/ros/humble/setup.bash，')
        print('  并把厂商的 lib 目录放在 LD_LIBRARY_PATH 最前面（别挂 ~/aimdk/install）。')
        return 1

    ok = True
    for name in args.srv:
        ok = dump_service(lib, args.package, name) and ok
    for name in args.msg:
        ok = dump_message(lib, args.package, name) and ok

    if ok:
        print('# 注意：字段顺序就是序列化顺序，自己补 interface 文件时一个都别调。')
        print('# 默认值和常量反推不出来，见模块头部坑 4。')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
