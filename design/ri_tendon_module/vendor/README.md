# RI20 / RI30 官方 CAD 来源与几何审查

获取日期：2026-11-03。官方原件保持不变；`sources.json` 保存下载地址与 SHA-256。

## 官方产品与下载

- RI20 产品：https://www.cubemars.com/cn/product/ri20-kv600-frameless-torque-motor.html
- RI30 产品：https://www.cubemars.com/product/ri30-kv260-frameless-torque-motor.html
- RI20 STEP ZIP：https://www.cubemars.com/data/cms/202608/ri20-kv600-frameless-torque-motor-without-hall-sensor-3d-drawing.zip
- RI30 STEP ZIP：https://www.cubemars.com/data/cms/202608/ri30-kv260-frameless-torque-motor-without-hall-sensor-3d-drawing.zip

网页原始下载 href 写作 `data/cms/...`，浏览器按产品 URL 解析得到错误路径并返回 404。使用同一厂家域名根目录下的原始文件路径后，两型号 PDF 与 ZIP 均 HTTP 200 下载成功。

## 官方工程图

两张 PDF 的制图日期均为 2026-08-13，Rev A，单位 mm。官方注明此为有限尺寸图纸，其他尺寸公差按要求提供。

| 项目 | RI20 | RI30 |
|---|---:|---:|
| 定子外径 | 25，公差 −0.02/−0.04 | 38，公差 0/−0.06 |
| 最大轴向包络 | 14.5 | 14 |
| 转子孔径 | 11.6 ±0.02 | 17 ±0.02 |
| 转子轴向长度 | 10 | 8 |
| 铁芯叠厚 | 8 | 6 |
| 厂家标称总质量 | 20 g | 40 g |

网页 RI30 宣传中段仍带有 RI20 的 16 V、72 mNm、9600 rpm、11.6 mm 数据，不能用于 RI30 选型。RI30 核心数据及规格表为 24 V、0.23 Nm 峰值、6240 rpm 空载、17 mm 孔径。

## STEP 几何

使用 CadQuery 2.8 / OpenCascade 7.9 导入并测量，未缩放，电机轴为 Z。具体各实体 bbox、面类型与体积见 `step_geometry.json`。

- `ri20/RI20 3D Drawing.STEP`：5 个 solid。0 为定子简化体；1–3 为三根短引线；4 为转子环。定子外包络 Z = −8…6.5，转子 Z = −6…4。转子内径 11.6、外径 15.8。定子有效内径 16.6，几何径向间隙 0.4。
- `ri30/RI30 3D Drawing.STEP`：2 个 solid。0 为定子连同短引线简化体；1 为转子环。定子外包络 Z = −3…11，转子 Z = 0…8。转子内径 17、外径 22.38。定子有效内径 23，几何径向间隙 0.31。
- RI30 定子 solid 的 Y 最小值 −25 包含引线。主体圆柱直径仍为 38，不能将含引线 bbox 当作定子直径。
- 官方模型没有轴承、输出轴、用户定制轮毂、外壳、绕线器或编码器。模型未细分真实绕组、叠片与磁片，且填充了若干真实内部空间；其体积不能直接用于准确质量或转动惯量核算。

## 安装方法依据的边界

未检索到 RI20 / RI30 专用官方装配手册。定子胶合至铝壳、转子胶合至用户轮毂、用独立轴承定位是行业可行的集成方法；具体胶层、公差、固化过程和轴向承力面仍须根据 RI 实物与厂家要求确认。

可参考厂商通用资料：

- Novanta / Celera Motion：https://novanta.com/robotics-automation/technical-paper/how-to-install-a-frameless-motor-installation-design-guide/
- maxon 安装手册：https://www.maxongroup.com/medias/sys_master/root/8873283846174/ecXXframeless-en-21-03.pdf

这些资料支持装配原理，不能替代 CubeMars 对其具体零件的安装公差批准。
