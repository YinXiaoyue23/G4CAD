# G4CAD Validation & Benchmark — 实现计划

**日期：** 2026-06-25  
**规格：** `docs/superpowers/specs/2026-06-25-g4cad-validation-benchmark-design.md`  
**目标目录：** `examples/validation_benchmark/`  
**环境：** conda IR3（Geant4、OCCT、ROOT 均已安装）

---

## 阶段概览

| 阶段 | 内容 | 任务 |
|------|------|------|
| 1 | 骨架与构建系统 | T01–T02 |
| 2 | 核心基础设施 | T03–T04 |
| 3 | 验证模块 | T05–T06 |
| 4 | Geant4 UserActions | T07–T11 |
| 5 | 仿真与性能测试 | T12–T13 |
| 6 | 入口与集成 | T14 |
| 7 | STEP 文件与构建验证 | T15–T16 |
| 8 | Python 绘图脚本 | T17–T19 |

---

## 任务列表

### T01 — 目录骨架

**创建所有目录和空占位文件（不含实现内容）：**

```
examples/validation_benchmark/
├── CMakeLists.txt                   (空，T02 填充)
├── src/
│   ├── main.cc
│   ├── GeometryRegistry.hh / .cc
│   ├── NavigationValidator.hh / .cc
│   ├── CrossSectionSampler.hh / .cc
│   ├── PhysicsRunner.hh / .cc
│   ├── BenchmarkRunner.hh / .cc
│   ├── OutputWriter.hh / .cc
│   └── geant4/
│       ├── DetectorConstruction.hh / .cc
│       ├── ActionInitialization.hh / .cc
│       ├── RunAction.hh / .cc
│       ├── EventAction.hh / .cc
│       └── PrimaryGeneratorAction.hh / .cc
├── step_files/                      (空目录，T16 填充)
├── tools/
│   └── gen_step_files.py
└── python/
    ├── plot_validation.py
    ├── plot_scaling.py
    └── plot_cross_section.py
```

**验收：** `find examples/validation_benchmark -type f | sort` 输出结构完整。

---

### T02 — CMakeLists.txt

**实现 `examples/validation_benchmark/CMakeLists.txt`：**

- `cmake_minimum_required(VERSION 3.16)`
- `project(G4CADValidationBench VERSION 1.0.0)`
- `find_package(Geant4 REQUIRED)`
- `find_package(G4CAD CONFIG REQUIRED HINTS $ENV{G4CAD_DIR} /home/yinxy/work/IR3/library/G4CAD/install/lib/cmake/G4CAD)`
- `option(ENABLE_ROOT "Enable ROOT output" OFF)` + 可选 `find_package(ROOT ...)`
- 收集所有 `.cc` 源文件为 `SOURCES` 变量
- `add_executable(g4cad_bench ${SOURCES})`
- `target_include_directories(g4cad_bench PRIVATE src)`
- `target_link_libraries(g4cad_bench PRIVATE G4CAD::G4CAD ${Geant4_LIBRARIES})`
- ROOT 条件链接 + `ENABLE_ROOT` 编译定义
- `install(TARGETS g4cad_bench)`

**验收：** `cmake -S . -B build -DG4CAD_DIR=... -DENABLE_ROOT=ON` 配置成功，无报错。

---

### T03 — GeometryRegistry

**文件：** `src/GeometryRegistry.hh` + `src/GeometryRegistry.cc`

**实现内容：**

```cpp
enum class GeometryType { Box, Sphere, Cylinder, BoxHole, TouchingBoxes };

struct GeometryInfo {
    std::string   name;
    double        analytical_volume_mm3;
    G4ThreeVector bbox_min, bbox_max;
    int           n_solids;
};

class GeometryRegistry {
public:
    static std::vector<G4VSolid*> CreateNative(GeometryType type);
    static std::vector<G4VSolid*> CreateFromStep(const std::string& step_file);
    static GeometryInfo           GetInfo(GeometryType type);
    static GeometryType           Parse(const std::string& name);  // 解析命令行字符串
};
```

**几何参数（规格 §3）：**

| 几何 | Native 实现 |
|------|-----------|
| Box | `new G4Box("box", 50, 50, 50)` （mm） |
| Sphere | `new G4Sphere("sphere", 0, 50, 0, CLHEP::twopi, 0, CLHEP::pi)` |
| Cylinder | `new G4Tubs("cylinder", 0, 50, 50, 0, CLHEP::twopi)` |
| BoxHole | `new G4SubtractionSolid("box_hole", new G4Box(...100,100,100), new G4Tubs(...20mm), nullptr, {})` |
| TouchingBoxes | 两个 `new G4Box("box_L/R", 50, 100, 100)` |

**`CreateFromStep`：** 调用 `G4StepLoader::Load(step_file)`，对每个 `TopoDS_Shape` 构造 `new G4StepSolid("part_N", shape)`

**`GetInfo`：** 返回解析体积（Box=1e6 mm³，Sphere=π×50³×4/3 等）和 bbox

**验收：** 单元测试（在 `main.cc` 中 `--mode native --events 0` 时打印 solid 名称和体积，无崩溃）。

---

### T04 — OutputWriter

**文件：** `src/OutputWriter.hh` + `src/OutputWriter.cc`

**职责：** 统一管理所有 CSV 文件的写入（可选 ROOT TTree/TH1）。

**接口：**

```cpp
class OutputWriter {
public:
    explicit OutputWriter(const std::string& prefix, bool enable_root = false);

    // 写入一行到对应 CSV
    void WriteGeometrySummary(const GeometrySummaryRow& row);
    void WriteNavigationTest(const NavigationTestRow& row);
    void WriteEventSummary(const EventSummaryRow& row);
    void WriteScalingSummary(const ScalingSummaryRow& row);
    void WriteCrossSection(const std::string& geometry, const std::string& mode,
                           const std::string& plane,
                           const std::vector<CrossSectionPoint>& points);

    void Flush();  // 关闭所有文件句柄
};
```

每个 `*Row` 结构体包含对应 CSV 的所有字段（规格 §4b.4, §4.4, §5.4, §6.3, §7）。

**CSV 文件头：**

```
geometry_summary.csv:
  geometry,mode,volume_mm3,analytical_volume_mm3,volume_rel_error,
  bbox_xmin_mm,bbox_xmax_mm,bbox_ymin_mm,bbox_ymax_mm,bbox_zmin_mm,bbox_zmax_mm,
  n_solids,tolerance_mm

navigation_test.csv:
  geometry,mode_pair,n_points,n_rays,mismatch_rate,
  mean_dev_mm,median_dev_mm,max_dev_mm,n_problematic_in,n_problematic_out

event_summary.csv:
  geometry,mode,event_id,edep_MeV,track_length_mm,n_steps,n_boundary_crossings

scaling_summary.csv:
  geometry,mode,n_threads,n_events,wall_time_s,events_per_sec,
  speedup,efficiency_pct,mem_rss_mb,nav_warnings

cross_section CSV: x_mm,y_mm,z_mm,classification
```

**验收：** 构造 OutputWriter，写入一行，Flush 后文件存在且格式正确。

---

### T05 — NavigationValidator

**文件：** `src/NavigationValidator.hh` + `src/NavigationValidator.cc`

**实现（规格 §4）：**

```cpp
struct NavValidationResult {
    std::string geometry_name;
    int         n_points, n_rays;
    double      mismatch_rate;
    double      mean_dev_in, median_dev_in, max_dev_in;
    double      mean_dev_out, median_dev_out, max_dev_out;
    int         n_problematic_in, n_problematic_out;
    std::vector<G4ThreeVector> dump_points;  // --enable-navigation-dump 时填充
};

class NavigationValidator {
public:
    NavigationValidator(int n_points = 100000, int n_rays = 50000,
                        double tol = 1e-3, bool dump = false);

    NavValidationResult Run(
        const std::vector<G4VSolid*>& native_solids,
        const std::vector<G4VSolid*>& step_solids,
        const G4ThreeVector& bbox_min,
        const G4ThreeVector& bbox_max,
        long seed);
};
```

**点分类测试：** CLHEP::HepRandom 采样 bbox 扩展 10% 区域，比较 `Inside()` 结果

**射线距离测试：** 从 bbox 外部随机出发，沿随机方向测 DistanceToIn；进入后测 DistanceToOut；统计偏差的 mean/median/max（用 `std::vector<double>` + `std::sort` 求 median）

**`touching_boxes` 多 solid 处理：** 两个 solid 分别查询，取最小距离（对 outside 点）或 union（对 inside 判断）

**验收：** 对 native box 自身比较（native vs native），mismatch_rate 应为 0，距离偏差为 0。

---

### T06 — CrossSectionSampler

**文件：** `src/CrossSectionSampler.hh` + `src/CrossSectionSampler.cc`

**实现（规格 §4b）：**

```cpp
enum class SlicePlane { XY, XZ, YZ };

struct CrossSectionConfig {
    int resolution = 500;
    double padding = 0.05;
    std::vector<SlicePlane> planes = { SlicePlane::XY, SlicePlane::XZ, SlicePlane::YZ };
};

struct CrossSectionPoint {
    float x, y, z;
    int8_t classification;  // 0=Outside, 1=Surface, 2=Inside
};

class CrossSectionSampler {
public:
    static std::vector<CrossSectionPoint> Sample(
        const std::vector<G4VSolid*>& solids,
        const G4ThreeVector& bbox_min,
        const G4ThreeVector& bbox_max,
        SlicePlane plane,
        int resolution,
        double padding);
};
```

**采样逻辑：**

- XY 切面：固定 z=0，x ∈ [xmin*(1+padding), xmax*(1+padding)]，y 同理
- XZ 切面：固定 y=0
- YZ 切面：固定 x=0
- `touching_boxes` union：对每个点查询两个 solid，取结果最"内"者（Inside > Surface > Outside）
- 分类编码：kInside→2，kSurface→1，kOutside→0

**验收：** 对 G4Box(50,50,50) 在 XY 切面采样，圆形区域内应为 kInside，外为 kOutside，边缘为 kSurface。

---

### T07 — DetectorConstruction

**文件：** `src/geant4/DetectorConstruction.hh` + `.cc`

```cpp
class DetectorConstruction : public G4VUserDetectorConstruction {
public:
    DetectorConstruction(std::vector<G4VSolid*> solids,
                         const std::string& material = "G4_Si",
                         double world_half_size_mm = 300.0);
    G4VPhysicalVolume* Construct() override;
};
```

**构造逻辑：**
- 世界体：`G4Box(600×600×600 mm, G4_AIR)`
- 对每个 solid：`new G4LogicalVolume(solid, mat, name)` + `new G4PVPlacement`
- `touching_boxes` 两个 solid 置于 `(±50, 0, 0) mm`
- 单 solid 置于原点

**验收：** 构造后 `G4VPhysicalVolume::GetLogicalVolume()->GetNoDaughters()` 等于 solid 数量。

---

### T08 — PrimaryGeneratorAction

**文件：** `src/geant4/PrimaryGeneratorAction.hh` + `.cc`

```cpp
class PrimaryGeneratorAction : public G4VUserPrimaryGeneratorAction {
public:
    PrimaryGeneratorAction(const std::string& particle = "e-",
                           double energy_MeV = 10.0,
                           const G4ThreeVector& direction = {0,0,1},
                           const G4ThreeVector& position  = {0,0,-200});
    void GeneratePrimaries(G4Event* event) override;
private:
    G4ParticleGun* fGun;
};
```

**验收：** 粒子名、能量、位置、方向均可通过构造函数配置。

---

### T09 — EventAction

**文件：** `src/geant4/EventAction.hh` + `.cc`

**职责：** 在每个 step 累积 Edep、track length、n_steps、n_boundary；event 结束时将结果推送给 RunAction。

```cpp
class EventAction : public G4UserEventAction {
public:
    void BeginOfEventAction(const G4Event*) override;
    void EndOfEventAction(const G4Event*) override;
    void AccumulateStep(double edep, double length, bool is_boundary);
};
```

步长累积在 **SteppingAction** 中（需新增 `src/geant4/SteppingAction.hh/.cc`，T09 一并实现）：

```cpp
class SteppingAction : public G4UserSteppingAction {
public:
    explicit SteppingAction(EventAction* ea);
    void UserSteppingAction(const G4Step* step) override;
};
```

**验收：** 10 个事件的 Edep 总和大于 0，n_steps 大于 0。

---

### T10 — RunAction

**文件：** `src/geant4/RunAction.hh` + `.cc`

**职责：** 接收来自 EventAction 的每事件数据，通过 OutputWriter 写入 `event_summary.csv`。

MT 模式下：每个 worker thread 独立写入，用 mutex 保护 OutputWriter（或每线程独立文件后合并）。

```cpp
class RunAction : public G4UserRunAction {
public:
    RunAction(OutputWriter* writer,
              const std::string& geometry_name,
              const std::string& mode);
    void BeginOfRunAction(const G4Run*) override;
    void EndOfRunAction(const G4Run*) override;
};
```

**验收：** 运行 100 个事件后，`event_summary.csv` 有 100 行数据。

---

### T11 — ActionInitialization

**文件：** `src/geant4/ActionInitialization.hh` + `.cc`

```cpp
class ActionInitialization : public G4VUserActionInitialization {
public:
    ActionInitialization(OutputWriter* writer,
                         const PrimaryConfig& primary_cfg,
                         const std::string& geometry_name,
                         const std::string& mode);
    void BuildForMaster() const override;  // MT: RunAction only
    void Build() const override;           // worker: all actions
};
```

**验收：** 与 `G4MTRunManager` 配合，worker 线程各自构造独立 Action 对象。

---

### T12 — PhysicsRunner

**文件：** `src/PhysicsRunner.hh` + `src/PhysicsRunner.cc`

**职责：** 封装完整 G4 仿真，接受外部构造好的 solid 列表和仿真参数，运行 `BeamOn(N)`。

```cpp
struct PhysicsConfig {
    std::string geometry_name;
    std::string mode;            // "native" or "step"
    std::string particle;        // "e-", "gamma", "proton"
    double      energy_MeV = 10.0;
    int         n_events   = 1000;
    int         n_threads  = 1;
    long        seed       = 42;
    std::string physics_list = "FTFP_BERT";
    std::string material     = "G4_Si";
};

class PhysicsRunner {
public:
    PhysicsRunner(OutputWriter* writer, const PhysicsConfig& cfg);
    void Run(std::vector<G4VSolid*> solids);
};
```

**实现细节：**
- `n_threads == 1`：使用 `G4RunManager`（sequential）
- `n_threads > 1`：使用 `G4MTRunManager`，`SetNumberOfThreads(n_threads)`
- 通过 `G4Random::setTheSeed(seed)` 设置随机种子
- 物理列表通过 `G4PhysListFactory` 构造

**验收：** 运行 100 个事件无崩溃，`event_summary.csv` 数据合理（Edep > 0）。

---

### T13 — BenchmarkRunner

**文件：** `src/BenchmarkRunner.hh` + `src/BenchmarkRunner.cc`

**职责：** 自动遍历线程数 `[1, 2, 4, 8, 16]`，依次调用 PhysicsRunner，记录 wall time。

```cpp
class BenchmarkRunner {
public:
    BenchmarkRunner(OutputWriter* writer,
                    const PhysicsConfig& base_cfg,
                    std::vector<G4VSolid*> solids);
    void Run();
private:
    size_t ReadMemRSS_MB();   // 读 /proc/self/status VmRSS，Linux only
};
```

**线程数约束：** `n_threads <= std::thread::hardware_concurrency()`；超出时跳过并输出提示。

**Speedup 基准：** 1 线程的 events_per_sec；efficiency = speedup / n_threads。

**验收：** `scaling_summary.csv` 至少有 1 行（单线程），speedup 值合理（≥ 0.5 at 2 threads）。

---

### T14 — main.cc

**文件：** `src/main.cc`

**职责：** 命令行解析 + 模块调度。使用标准 `<getopt_long>` 或手写简单解析器（无需第三方库）。

**完整命令行参数（规格 §2.3）：**

```
--geometry, --mode, --step-file, --events, --threads,
--particle, --energy, --seed, --output,
--navigation-test, --enable-navigation-dump,
--benchmark,
--cross-section, --cross-section-plane, --cross-section-resolution,
--physics-list
```

**调度逻辑：**

```
1. 解析参数，校验必填项
2. 构造 OutputWriter(output_prefix, enable_root)
3. 构造几何 solids（native 或 STEP）
4. 写入 geometry_summary.csv（始终执行）
5. if --navigation-test: NavigationValidator::Run() → 写 navigation_test.csv
6. if --cross-section: CrossSectionSampler::Sample() → 写 cross_section_*.csv
7. if n_events > 0 && !--benchmark: PhysicsRunner::Run() → 写 event_summary.csv
8. if --benchmark: BenchmarkRunner::Run() → 写 scaling_summary.csv
9. OutputWriter::Flush()
```

**验收：** `./g4cad_bench --help` 输出所有选项；`--geometry box --mode native --events 0` 不崩溃，生成 `geometry_summary.csv`。

---

### T15 — tools/gen_step_files.py

**文件：** `tools/gen_step_files.py`

**用途：** 在 FreeCAD Python 控制台中运行，生成规格 §3 所有五种几何的 STEP 文件。

**实现：**

```python
import FreeCAD, Part

OUTPUT_DIR = "step_files"  # 相对于脚本位置

# box: 100×100×100 mm (全尺寸，对应半边长 50mm)
box = Part.makeBox(100, 100, 100)
box.translate(FreeCAD.Vector(-50, -50, -50))
Part.export([box], f"{OUTPUT_DIR}/box.step")

# sphere: radius 50mm
sphere = Part.makeSphere(50)
Part.export([sphere], f"{OUTPUT_DIR}/sphere.step")

# cylinder: radius 50mm, height 100mm
cyl = Part.makeCylinder(50, 100)
cyl.translate(FreeCAD.Vector(0, 0, -50))
Part.export([cyl], f"{OUTPUT_DIR}/cylinder.step")

# box_hole: 200×200×200 mm box - radius 20mm cylinder along Z
big_box = Part.makeBox(200, 200, 200)
big_box.translate(FreeCAD.Vector(-100, -100, -100))
hole = Part.makeCylinder(20, 200, FreeCAD.Vector(0, 0, -100))
box_hole = big_box.cut(hole)
Part.export([box_hole], f"{OUTPUT_DIR}/box_hole.step")

# touching_boxes: two 100×200×200 mm boxes adjacent along X at X=0
box_L = Part.makeBox(100, 200, 200)
box_L.translate(FreeCAD.Vector(-100, -100, -100))
box_R = Part.makeBox(100, 200, 200)
box_R.translate(FreeCAD.Vector(0, -100, -100))
Part.export([box_L, box_R], f"{OUTPUT_DIR}/touching_boxes.step")
```

脚本顶部注释说明运行方式（FreeCAD → Tools → Macros 或 FreeCAD Python console）。

**验收：** 运行后 `step_files/` 目录含 5 个 `.step` 文件，大小均 > 1 KB。

---

### T16 — 预生成 STEP 文件并提交

**操作步骤：**

1. 在安装了 FreeCAD 的机器上运行 `tools/gen_step_files.py`
2. 将生成的 5 个 STEP 文件复制到 `examples/validation_benchmark/step_files/`
3. `git add step_files/*.step && git commit`

**若无 FreeCAD 环境：** 使用 pyocc（OCCT Python 绑定）在 conda IR3 中脚本化生成（备选方案，T16b）。

**验收：** `step_files/` 目录含 `box.step`、`sphere.step`、`cylinder.step`、`box_hole.step`、`touching_boxes.step`。

---

### T17 — python/plot_validation.py

**输入：** 命令行传入 results 目录，自动发现 CSV 文件。

**输出图：**
- 图 1（`volume_comparison.pdf`）：native vs STEP 体积条形图 + 相对误差，每种几何一组
- 图 2（`nav_deviation_cdf.pdf`）：DistanceToIn 偏差 CDF，每种几何一条曲线
- 图 3（`energy_deposition.pdf`）：Edep 分布直方图，native vs STEP 叠加，5 个子图
- 图 4（`step_length.pdf`）：步长直方图对比

**要求：** matplotlib，DPI ≥ 300，提供 `--output-dir`，`--data-dir` 参数。

---

### T18 — python/plot_scaling.py

**输入：** `scaling_summary.csv`

**输出图：**
- 图 1（`scaling_walltime.pdf`）：wall time vs n_threads（log-log）
- 图 2（`scaling_speedup.pdf`）：speedup vs n_threads + 理想线性参考线
- 图 3（`scaling_efficiency.pdf`）：parallel efficiency (%) vs n_threads

**要求：** 支持 `--geometry` 过滤，DPI ≥ 300。

---

### T19 — python/plot_cross_section.py

**模式：**
- 单文件模式：`--native <csv> --step <csv> --output <dir>`
- 批量模式：`--all --data-dir <dir> --output <dir>`（自动匹配所有 geometry × plane 对）

**每张图（`cross_section_<geometry>_<plane>_comparison.png`）：**

1×3 子图（约 14×5 英寸）：

| 子图 1：native | 子图 2：STEP | 子图 3：差异图 |
|:--------------:|:------------:|:--------------:|
| kInside=蓝，kSurface=红，kOutside=浅灰 | 同左 | 一致=白，native内/STEP外=橙，native外/STEP内=紫，surface不一致=黄 |

**实现细节：**
- 用 `numpy.frompyfunc` 或直接 `np.reshape` 将 CSV 转为 2D 数组
- 差异图：`np.where` 条件运算，不逐点 Python 循环
- `plt.imshow`，`origin='lower'`，`aspect='equal'`，坐标轴 mm 单位
- 标题格式：`{geometry} — {plane} slice at {axis}=0  |  native  ||  STEP  ||  diff`

**验收：** 对 box 的 XY 切面，native 和 STEP 图均显示清晰矩形边界，差异图为全白（或极少黄色像素在边缘）。

---

### T20 — 构建与集成验证

**步骤：**

```bash
conda activate IR3
cd /home/yinxy/work/IR3/library/G4CAD/examples/validation_benchmark
cmake -S . -B build \
    -DG4CAD_DIR=/home/yinxy/work/IR3/library/G4CAD/install/lib/cmake/G4CAD \
    -DENABLE_ROOT=ON \
    -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

**冒烟测试：**

```bash
cd build

# 1. 几何摘要（native，无仿真）
./g4cad_bench --geometry box --mode native --events 0 --output smoke/box

# 2. 导航验证（native vs STEP）
./g4cad_bench --geometry box --mode native \
    --navigation-test --step-file ../step_files/box.step \
    --events 0 --output smoke/box_nav

# 3. 横截面采样
./g4cad_bench --geometry cylinder --mode native \
    --cross-section --cross-section-resolution 200 \
    --step-file ../step_files/cylinder.step \
    --events 0 --output smoke/cyl_cs

# 4. 物理仿真（100 events）
./g4cad_bench --geometry sphere --mode native \
    --events 100 --seed 42 --output smoke/sphere_phys

# 5. Python 横截面渲染（快速验证）
python ../python/plot_cross_section.py \
    --native smoke/cross_section_cylinder_native_xy.csv \
    --step   smoke/cross_section_cylinder_step_xy.csv \
    --output smoke/
```

**验收标准：**
- 所有步骤无 crash，无 Geant4 navigation error
- `smoke/geometry_summary.csv` 体积相对误差 < 1%
- `smoke/navigation_test.csv` mismatch_rate < 0.1%（box 几何）
- `smoke/cross_section_cylinder_native_xy.png` 显示圆形截面

---

## 依赖关系图

```
T01 (骨架)
  └─ T02 (CMake)
       └─ T03 (GeometryRegistry)
            ├─ T04 (OutputWriter)
            │    ├─ T05 (NavigationValidator) ──┐
            │    ├─ T06 (CrossSectionSampler) ──┤
            │    └─ T07-T11 (G4 Actions)        │
            │         └─ T12 (PhysicsRunner)    │
            │              └─ T13 (BenchmarkRunner)
            └─ T14 (main.cc) ←─────────────────┘
T15 (gen_step_files.py)
  └─ T16 (STEP 文件提交)
       └─ T20 (构建验证)
T17-T19 (Python 脚本，独立，T20 后验证)
```

---

## 注意事项

1. **MT 安全**：`G4StepSolid` 的线程安全性需参考 G4CAD 源码确认；每个 worker thread 应有独立的 `G4StepSolid` 实例（通过 `G4VUserDetectorConstruction::Construct()` 的 clone 机制）。

2. **STEP 文件路径**：`CreateFromStep` 支持绝对路径和相对路径；运行时通过 `--step-file` 传入，CI 中默认指向 `step_files/<geometry>.step`。

3. **ROOT 条件编译**：所有 ROOT 相关代码用 `#ifdef ENABLE_ROOT ... #endif` 包围，不影响纯 CSV 构建。

4. **`touching_boxes` 特殊处理**：该几何在 G4 detector construction 中是两个独立 PV，但在 NavigationValidator 和 CrossSectionSampler 中需作为 solid 列表处理（union 语义）。

5. **内存估算**：500×500 分辨率的 CrossSection CSV ≈ 10 MB/文件；如需降低，`--cross-section-resolution 200` 约 1.6 MB/文件。
