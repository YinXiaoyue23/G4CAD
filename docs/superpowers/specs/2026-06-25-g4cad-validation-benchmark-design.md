# G4CAD Validation & Benchmark Application — Design Spec

**Date:** 2026-06-25  
**Project:** G4CAD (`/home/yinxy/work/IR3/library/G4CAD`)  
**Location:** `examples/validation_benchmark/`  
**Status:** Approved, ready for implementation

---

## 1. 目标

构建一个可复现的测试套件，对比原生 Geant4 几何（`G4Box`/`G4Sphere`/`G4Tubs`/`G4SubtractionSolid`）与通过 G4CAD 导入的等价 STEP 几何，在以下三个层次进行验证：

1. **几何层**：体积、包围盒、solid 数量、tolerance 信息
2. **导航层**：点分类一致性、射线距离偏差
3. **物理层**：能量沉积、径迹长度、步数、边界穿越次数分布对比

同时支持多线程性能 scaling 测试，输出用于论文作图的机器可读结果。

4. **横截面可视化**：在选定平面（XY/XZ/YZ）上采样规则网格点，调用 `Inside()` 分类，对比 native 与 STEP 几何的内部边界、曲面、孔洞结构和 assembly 接触面。

---

## 2. 架构：方案 B（模块化应用）

### 2.1 目录结构

```
examples/validation_benchmark/
├── CMakeLists.txt
├── src/
│   ├── main.cc                          # 命令行解析 + 模块调度
│   ├── GeometryRegistry.hh/cc           # native/STEP 几何工厂
│   ├── NavigationValidator.hh/cc        # 随机点分类 + 射线距离对比
│   ├── CrossSectionSampler.hh/cc        # 规则网格采样 + 横截面 CSV 输出
│   ├── PhysicsRunner.hh/cc              # G4 仿真包装器
│   ├── BenchmarkRunner.hh/cc            # 多线程 scaling 循环
│   ├── OutputWriter.hh/cc               # CSV (+ 可选 ROOT) 写入
│   └── geant4/
│       ├── DetectorConstruction.hh/cc   # 接受外部 G4VSolid*
│       ├── ActionInitialization.hh/cc
│       ├── RunAction.hh/cc
│       ├── EventAction.hh/cc            # 累积 Edep/步数/边界穿越
│       └── PrimaryGeneratorAction.hh/cc
├── step_files/                          # 预生成 STEP 文件（提交到仓库）
│   ├── box.step
│   ├── sphere.step
│   ├── cylinder.step
│   ├── box_hole.step
│   └── touching_boxes.step
├── tools/
│   └── gen_step_files.py                # FreeCAD 脚本，用于重新生成 STEP 文件
└── python/
    ├── plot_validation.py
    ├── plot_scaling.py
    └── plot_cross_section.py
```

### 2.2 可执行文件

单一二进制：`g4cad_bench`

### 2.3 命令行接口

```
g4cad_bench
  --geometry      box|sphere|cylinder|box_hole|touching_boxes
  --mode          native|step
  --step-file     <path>          (mode=step 时必须)
  --events        <N>             (默认 1000)
  --threads       <N>             (默认 1)
  --particle      gamma|e-|proton (默认 e-)
  --energy        <MeV>           (默认 10.0)
  --seed          <value>         (默认 42)
  --output        <prefix>        (默认 "output")
  --navigation-test               (启用导航验证；此标志要求必须同时提供 --step-file，
                                   无论 --mode 取何值；程序始终同时构造 native 和 STEP solid 进行对比)
  --enable-navigation-dump        (将有问题的点坐标写入 CSV，用于 FreeCAD 调试)
  --benchmark                     (启用 scaling 测试，自动遍历线程数 1/2/4/8/16)
  --physics-list  FTFP_BERT|QBBC  (默认 FTFP_BERT)
  --cross-section                 (启用横截面采样；与 --navigation-test 相同，
                                   要求同时提供 --step-file，始终对比 native 和 STEP)
  --cross-section-plane  xy|xz|yz|all  (默认 all，即生成三个切面)
  --cross-section-resolution <N>  (每个方向网格点数，默认 500，即 500×500 网格)
```

---

## 3. 几何参数（GeometryRegistry）

五种几何的尺寸，native 和 STEP 完全一致：

| 几何 | Native 实现 | 参数 |
|------|------------|------|
| `box` | `G4Box` | 半边长 50×50×50 mm |
| `sphere` | `G4Sphere` | 内径 0，外径 50 mm，全球（θ: 0→π，φ: 0→2π） |
| `cylinder` | `G4Tubs` | 内径 0，外径 50 mm，半高 50 mm |
| `box_hole` | `G4SubtractionSolid(G4Box - G4Tubs)` | 100×100×100 mm 箱，减去沿 Z 轴半径 20 mm 圆柱 |
| `touching_boxes` | 两个 `G4Box` + `G4PVPlacement` | 各 50×100×100 mm，沿 X 轴相接，共享 X=0 面 |

### 3.1 GeometryRegistry 接口

```cpp
enum class GeometryType { Box, Sphere, Cylinder, BoxHole, TouchingBoxes };

struct GeometryInfo {
    std::string  name;
    double       analytical_volume_mm3;
    G4ThreeVector bbox_min, bbox_max;
    int          n_solids;
};

class GeometryRegistry {
public:
    static std::vector<G4VSolid*> CreateNative(GeometryType type);
    static std::vector<G4VSolid*> CreateFromStep(const std::string& step_file);
    static GeometryInfo           GetInfo(GeometryType type);
    static GeometryType           Parse(const std::string& name);
};
```

- `touching_boxes` 的 native 模式返回两个 `G4Box`（各自独立 solid）
- `CreateFromStep` 调用 `G4StepLoader::Load()`，对每个 `TopoDS_Shape` 构造 `G4StepSolid`
- `GetInfo` 返回解析体积（用于与 solid 计算值对比）

---

## 4. 导航验证（NavigationValidator）

### 4.1 输入

- `native_solid`：`G4VSolid*`（或列表，for touching_boxes）
- `step_solid`：`G4VSolid*`（或列表）
- `N_points`：随机点数（默认 100000）
- `N_rays`：随机射线数（默认 50000）
- `tolerance`：距离偏差阈值（默认 1e-3 mm）

### 4.2 点分类测试

```
bbox 扩展 10% 生成采样区域
对 N_points 个均匀随机点 p：
    classify_native = native->Inside(p)   // kInside/kOutside/kSurface
    classify_step   = step->Inside(p)
    if classify_native != classify_step:
        ++mismatch_count
        if dump_enabled: 记录 p, classify_native, classify_step
mismatch_rate = mismatch_count / N_points
```

### 4.3 射线距离测试

```
对 N_rays 条随机射线 (p_outside, v_unit)：
    d_in_native  = native->DistanceToIn(p, v)
    d_in_step    = step->DistanceToIn(p, v)
    dev_in       = |d_in_native - d_in_step|

    // 对进入点内部再测 DistanceToOut
    p_inside = p + d_in_native * v * 1.001
    if native->Inside(p_inside) == kInside:
        d_out_native = native->DistanceToOut(p_inside, v)
        d_out_step   = step->DistanceToOut(p_inside, v)
        dev_out      = |d_out_native - d_out_step|

收集所有 dev_in 和 dev_out，统计 mean/median/max
n_problematic = count(dev > tolerance)
```

### 4.4 输出 `navigation_test.csv`

```
geometry,mode_pair,n_points,n_rays,mismatch_rate,
mean_dev_mm,median_dev_mm,max_dev_mm,n_problematic_in,n_problematic_out
```

---

## 4b. 横截面采样（CrossSectionSampler）

### 4b.1 设计原则

- 采样完全在 **C++ 端**完成，直接调用 `G4VSolid::Inside()`，不依赖 Python
- Python 端仅负责读取 CSV 并渲染 PNG，二者职责分离
- native 和 STEP 使用**完全相同**的网格范围和分辨率，确保结果可直接相减

### 4b.2 接口

```cpp
enum class SlicePlane { XY, XZ, YZ };

struct CrossSectionConfig {
    int         resolution = 500;     // 每方向网格点数（500×500）
    double      padding    = 0.05;    // bbox 扩展比例（5%）
    std::vector<SlicePlane> planes = { SlicePlane::XY, SlicePlane::XZ, SlicePlane::YZ };
};

class CrossSectionSampler {
public:
    // 对单个 solid 采样指定切面，写入 CSV
    // 文件名格式：<prefix>_<plane>.csv
    static void Sample(G4VSolid*               solid,
                       const G4ThreeVector&     bbox_min,
                       const G4ThreeVector&     bbox_max,
                       const CrossSectionConfig& cfg,
                       const std::string&       output_prefix);
};
```

### 4b.3 采样逻辑（以 XY 切面为例）

```
bbox 扩展 padding 后确定采样范围 [x0,x1] × [y0,y1]
dx = (x1 - x0) / (resolution - 1)
dy = (y1 - y0) / (resolution - 1)
z_fixed = 0  (z=0 中心切面)

对每个 (i, j)：
    x = x0 + i * dx
    y = y0 + j * dy
    p = G4ThreeVector(x, y, z_fixed)
    cls = solid->Inside(p)        // kInside=2, kSurface=1, kOutside=0
    写入一行：x, y, z_fixed, cls
```

XZ 切面：固定 y=0，网格 [x0,x1] × [z0,z1]  
YZ 切面：固定 x=0，网格 [y0,y1] × [z0,z1]

`touching_boxes` 有两个 solid：对每个点依次查询两个 solid，取 union（任一为 kInside 则结果为 kInside，任一为 kSurface 则为 kSurface，否则 kOutside）。

### 4b.4 输出文件

每种几何、每种模式、每个切面各一个 CSV：

```
cross_section_<geometry>_<mode>_xy.csv
cross_section_<geometry>_<mode>_xz.csv
cross_section_<geometry>_<mode>_yz.csv
```

每个文件格式（无表头，节省空间；Python 读取时直接 `usecols`）：

```
x_mm, y_mm, z_mm, classification
```

`classification` 编码：`0`=kOutside，`1`=kSurface，`2`=kInside

文件大小估算：500×500 × 4列 × ~10 字节/行 ≈ 10 MB/文件，共 5 几何 × 2 模式 × 3 切面 = 30 个 CSV（≈ 300 MB）。若存储受限，可将 resolution 降至 200（≈ 5 MB 总量）。

### 4b.5 Python 渲染（plot_cross_section.py）

**输入：** 同一几何的 native 和 STEP CSV 文件对

**输出（每种几何 × 每个切面 = 15 张图）：**

```
cross_section_<geometry>_<plane>_comparison.png
```

每张图为 **1×3 子图布局**：

```
[native 分类图] | [STEP 分类图] | [差异图]
```

- **分类图**：二维热图，颜色映射：kInside=蓝色，kSurface=红色，kOutside=白色/浅灰
- **差异图**：逐点对比，颜色映射：
  - 两者一致 → 白色
  - native=kInside, STEP=kOutside → 橙色（G4CAD 将内部误判为外部）
  - native=kOutside, STEP=kInside → 紫色（G4CAD 将外部误判为内部）
  - 含 kSurface 的不一致 → 黄色（边界带差异）

图形规格（适合论文）：
- DPI ≥ 300
- 子图尺寸约 4×4 英寸，总图宽约 14 英寸
- 坐标轴标注单位 mm，colorbar 标注分类名称
- 标题格式：`<geometry> — <plane> slice at <axis>=0`

**典型调用：**

```bash
python python/plot_cross_section.py \
    --geometry box_hole \
    --plane xy \
    --native  results/cross_section_box_hole_native_xy.csv \
    --step    results/cross_section_box_hole_step_xy.csv \
    --output  figures/
```

或批量处理所有几何和切面：

```bash
python python/plot_cross_section.py --all --data-dir results/ --output figures/
```

---

## 5. 物理仿真（PhysicsRunner）

### 5.1 仿真配置

| 参数 | 值 |
|------|-----|
| 世界体 | `G4Box` 600×600×600 mm，`G4_AIR` |
| 测试体材料 | `G4_Si`（默认） |
| 物理列表 | `FTFP_BERT`（默认）或 `QBBC` |
| 初级粒子 | `e-`（默认），10 MeV，沿 +z，距中心 -200 mm |
| 随机种子 | `--seed` 控制，native/STEP 使用相同种子 |
| cuts | 默认 Geant4 cuts（1 mm） |

### 5.2 UserAction 设计

**EventAction：** 在每个 step 上累积：
- `Edep` (MeV)：`step->GetTotalEnergyDeposit()`
- `track_length` (mm)：`step->GetStepLength()`
- `n_steps`：计数
- `n_boundary`：`step->GetPostStepPoint()->GetStepStatus() == fGeomBoundary`

**RunAction：** event 结束时将一行写入 `event_summary.csv`

### 5.3 DetectorConstruction 接口

```cpp
class DetectorConstruction : public G4VUserDetectorConstruction {
public:
    // 接受外部构造好的 solid 列表（native 或 STEP）
    DetectorConstruction(std::vector<G4VSolid*> solids,
                         const std::string& material = "G4_Si");
    G4VPhysicalVolume* Construct() override;
};
```

对 `touching_boxes`，两个 solid 分别置于 `(−50,0,0)` 和 `(+50,0,0)`。

### 5.4 输出 `event_summary.csv`

```
geometry,mode,event_id,edep_MeV,track_length_mm,n_steps,n_boundary_crossings
```

可选：`--output` 指定前缀，ROOT 模式时同时写 `event_summary.root`（TTree）。

---

## 6. 多线程 Scaling 测试（BenchmarkRunner）

### 6.1 流程

```
thread_counts = [1, 2, 4, 8, 16]，超过 std::thread::hardware_concurrency() 时跳过
对每个 n_threads：
    构造 G4RunManager（MT 模式，n_threads > 1 时用 G4MTRunManager）
    t_start = steady_clock::now()
    runManager->BeamOn(n_events)
    t_end   = steady_clock::now()
    wall_time = t_end - t_start
    events_per_sec = n_events / wall_time
    speedup    = events_per_sec / baseline_events_per_sec  (1线程基准)
    efficiency = speedup / n_threads
```

### 6.2 额外指标

- 内存：`/proc/self/status` 的 `VmRSS`（Linux only，不可用时填 -1）
- Navigation warnings：重定向 `G4cerr` 到计数器，统计含 "stuck" 或 "navigation" 关键字的行数

### 6.3 输出 `scaling_summary.csv`

```
geometry,mode,n_threads,n_events,wall_time_s,events_per_sec,speedup,efficiency_pct,mem_rss_mb,nav_warnings
```

---

## 7. 几何汇总输出 `geometry_summary.csv`

```
geometry,mode,volume_mm3,analytical_volume_mm3,volume_rel_error,
bbox_xmin_mm,bbox_xmax_mm,bbox_ymin_mm,bbox_ymax_mm,bbox_zmin_mm,bbox_zmax_mm,
n_solids,tolerance_mm
```

体积通过 `G4VSolid::GetCubicVolume()` 获取（native 和 STEP 均适用）。
tolerance 直接使用 `G4StepSolid` 构造函数传入的参数值（默认 1e-7 mm），记录时原样输出。

---

## 8. STEP 文件生成（tools/gen_step_files.py）

FreeCAD Python 脚本（在 FreeCAD 的 Python 控制台中运行），按第 3 节的尺寸创建所有五种几何并导出到 `step_files/`。

预生成结果直接提交到仓库，无需 FreeCAD 即可运行验证程序。

---

## 9. Python 绘图脚本

### plot_validation.py

输入：`geometry_summary.csv`、`navigation_test.csv`、`event_summary.csv`

输出：
- 图 1：native vs STEP 体积对比（条形图 + 相对误差）
- 图 2：导航偏差分布（CDF 或直方图，每种几何一条曲线）
- 图 3：能量沉积分布对比（native vs STEP，每种几何一个子图）
- 图 4：步长直方图对比

### plot_scaling.py

输入：`scaling_summary.csv`

输出：
- 图 1：wall time vs 线程数（log-log）
- 图 2：speedup vs 线程数（含理想线性参考）
- 图 3：parallel efficiency vs 线程数

---

## 10. CMake 配置

```cmake
cmake_minimum_required(VERSION 3.16)
project(G4CADValidationBench VERSION 1.0.0)

find_package(Geant4 REQUIRED)
find_package(G4CAD CONFIG REQUIRED
    HINTS $ENV{G4CAD_DIR}
          /home/yinxy/work/IR3/library/G4CAD/install/lib/cmake/G4CAD)

option(ENABLE_ROOT "Enable ROOT output (histogram + TTree)" OFF)
if(ENABLE_ROOT)
    find_package(ROOT REQUIRED COMPONENTS Core Hist Tree)
    message(STATUS "ROOT output enabled: ${ROOT_VERSION}")
endif()

set(SOURCES
    src/main.cc
    src/GeometryRegistry.cc
    src/NavigationValidator.cc
    src/CrossSectionSampler.cc
    src/PhysicsRunner.cc
    src/BenchmarkRunner.cc
    src/OutputWriter.cc
    src/geant4/DetectorConstruction.cc
    src/geant4/ActionInitialization.cc
    src/geant4/RunAction.cc
    src/geant4/EventAction.cc
    src/geant4/PrimaryGeneratorAction.cc
)

add_executable(g4cad_bench ${SOURCES})

target_include_directories(g4cad_bench PRIVATE src)
target_link_libraries(g4cad_bench PRIVATE
    G4CAD::G4CAD
    ${Geant4_LIBRARIES}
)

if(ENABLE_ROOT)
    target_link_libraries(g4cad_bench PRIVATE ROOT::Core ROOT::Hist ROOT::Tree)
    target_compile_definitions(g4cad_bench PRIVATE ENABLE_ROOT)
endif()
```

**推荐构建命令（IR3 conda 环境）：**

```bash
conda activate IR3
cd examples/validation_benchmark
cmake -S . -B build \
    -DG4CAD_DIR=/home/yinxy/work/IR3/library/G4CAD/install/lib/cmake/G4CAD \
    -DENABLE_ROOT=ON
cmake --build build -j$(nproc)
```

---

## 11. 典型使用示例

```bash
# 导航验证：native box vs STEP box
./build/g4cad_bench --geometry box --mode native \
    --navigation-test --step-file step_files/box.step \
    --events 0 --output results/box

# 横截面采样：box_hole，所有三个切面，500×500 分辨率
./build/g4cad_bench --geometry box_hole --mode native \
    --cross-section --cross-section-plane all --cross-section-resolution 500 \
    --step-file step_files/box_hole.step \
    --events 0 --output results/box_hole
# 生成：results/cross_section_box_hole_native_xy.csv 等 6 个 CSV

# 渲染横截面对比图
python python/plot_cross_section.py --all \
    --data-dir results/ --output figures/

# 物理对比：10 MeV e- on cylinder，native 和 STEP
./build/g4cad_bench --geometry cylinder --mode native \
    --events 10000 --seed 42 --output results/cyl_native
./build/g4cad_bench --geometry cylinder --mode step \
    --step-file step_files/cylinder.step \
    --events 10000 --seed 42 --output results/cyl_step

# Scaling benchmark：box_hole，8 线程，1000 events/thread
./build/g4cad_bench --geometry box_hole --mode step \
    --step-file step_files/box_hole.step \
    --benchmark --events 1000 --output results/bench

# 绘图
python python/plot_validation.py results/
python python/plot_scaling.py results/scaling_summary.csv
```

---

## 12. 论文输出对照表

| 论文需求 | 来源文件 | 生成方式 |
|----------|---------|---------|
| native vs STEP 体积/包围盒对比表 | `geometry_summary.csv` | `plot_validation.py` 表 1 |
| 点分类失配率与射线偏差表 | `navigation_test.csv` | `plot_validation.py` 表 2 |
| 能量沉积/步长分布图 | `event_summary.csv` | `plot_validation.py` 图 3/4 |
| runtime 与并行效率图 | `scaling_summary.csv` | `plot_scaling.py` 图 1/2/3 |
| 横截面内部边界/曲面/孔洞/接触面对比图 | `cross_section_<geom>_<mode>_<plane>.csv` | `plot_cross_section.py` 图 5 |

---

## 13. 依赖摘要

| 依赖 | 版本要求 | 来源 |
|------|---------|------|
| Geant4 | ≥ 11.0（需 MT 支持） | conda IR3 |
| OpenCASCADE | ≥ 7.7 | conda IR3（G4CAD 传递依赖） |
| G4CAD | 当前版本 | `/home/yinxy/work/IR3/library/G4CAD/install` |
| ROOT | ≥ 6.26（可选） | conda IR3 |
| Python | ≥ 3.9 | conda IR3 |
| matplotlib, pandas, numpy | 最新 | conda IR3 |
| FreeCAD | ≥ 0.21（可选，仅生成 STEP） | 独立安装 |
