# 脚本

## 常规使用

| 脚本 | 作用 |
|------|------|
| `verify_mjcf.py` | 校验 MJCF 可加载并打印关节摘要 |

## Dora 节点打包

| 脚本 | 作用 |
|------|------|
| `build_pyinstaller.sh` | 构建 `dist/robots_zerith_h1_pro` 单文件 Dora 节点（内置 H1 robot/camera SDK `.so`） |
| `zerith_h1_pro.spec` | PyInstaller spec |
| `dora_entry.py` | 打包入口（`multiprocessing.freeze_support` + `zerith_h1_pro.main`） |

构建前确认 `third_party/H1_SDK_1.3.7` 下 robot/camera Python 3.12 扩展已编译。
