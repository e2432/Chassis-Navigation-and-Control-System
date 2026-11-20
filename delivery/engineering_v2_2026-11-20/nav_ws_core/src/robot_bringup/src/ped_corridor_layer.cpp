// 行人横穿走廊注入层（9-3）
//
// 背景：Nav2 的 `static_layer` 在 `subscribe_to_updates: true` 时理论上能吃
// `map_msgs/OccupancyGridUpdate`，但实测（2026-11-17，两种 costmap、两种 QoS）
// 局部（滚动窗口）与全局都不响应——写进去的 100 完全不出现。所以"行人横穿走廊"
// 改由本层负责：**自己记住写过的格子，并在下一拍先恢复再重写**，擦除完全可控。
//
// 为什么放在 voxel_layer 之后、inflation_layer 之前：
//   * 在 voxel 之后 → 不会被点云的 raytrace 清掉；
//   * 在 inflation 之前 → 走廊能被正常膨胀，且恢复时拿到的正是"原始代价"。
//
// 话题：`/ped/corridor`（nav_msgs/OccupancyGrid，frame 用 costmap 的全局坐标系，
// 分辨率/原点任意，本层按世界坐标换算到 master 栅格；>=50 视为要占用）。

#include <cmath>
#include <cstdlib>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <thread>

namespace robot_bringup
{

class PedCorridorLayer : public nav2_costmap_2d::Layer
{
public:
  PedCorridorLayer() = default;

  void onInitialize() override
  {
    const char * st = std::getenv("PED_LAYER_STAGE");
    stage_ = st ? std::atoi(st) : 4;
    RCLCPP_INFO(logger_, "[ped_corridor] stage=%d", stage_);
    if (stage_ >= 1) {
      // **不调用 declare_parameter**：实测在 costmap 层的 onInitialize 里声明参数，
      // controller_server 的控制循环就再不产速度（机器人原地不动，日志也无报错）。
      // 参数改由环境变量提供，代价是配置不写在 YAML 里（见 docs/9-3行人动态障碍.md §7.6）。
      const char * t = std::getenv("PED_LAYER_TOPIC");
      if (t) {
        topic_ = t;
      }
      const char * th = std::getenv("PED_LAYER_THRESHOLD");
      if (th) {
        lethal_threshold_ = std::atoi(th);
      }
      RCLCPP_INFO(logger_, "[ped_corridor] 参数(env): topic=%s thr=%d",
                  topic_.c_str(), lethal_threshold_);
    }
    if (stage_ >= 2) {
      // **不要**在 costmap 节点上建订阅：实测（2026-11-17，stage 逐级对照）只要在
      // costmap 节点上多一个订阅，controller_server 的控制回调就不再产速度、
      // 机器人原地不动（连回调体都不执行也一样）。改成自带一个独立节点 + 自己的
      // SingleThreadedExecutor 线程，与 costmap 节点彻底解耦。
      std::string nname = "ped_corridor_reader_" + name_;
      for (auto & ch : nname) {
        if (ch == '.' || ch == '/' || ch == '-') {ch = '_';}
      }
      reader_node_ = std::make_shared<rclcpp::Node>(nname);
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.best_effort();
      sub_ = reader_node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_, qos,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          grid_ = msg;
          ++recv_count_;
          if (recv_count_ == 1 || (recv_count_ % 100) == 0) {
            RCLCPP_INFO(logger_, "[ped_corridor] 收到走廊消息 #%zu：%ux%u 原点(%.2f,%.2f)",
                        recv_count_, msg->info.width, msg->info.height,
                        msg->info.origin.position.x, msg->info.origin.position.y);
          }
        });
      exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      exec_->add_node(reader_node_);
      spin_thread_ = std::thread([this] {exec_->spin();});
      RCLCPP_INFO(logger_, "[ped_corridor] 已用独立节点 %s 订阅 %s", nname.c_str(), topic_.c_str());
    }
    // **必须显式打开**：基类 `Layer::enabled_` 在本环境被初始化成 false
    // （实测 updateCosts 入口打印 enabled=0），本层所有写操作会被静默跳过，
    // 走廊注入等于完全没生效。这里强制置 true。
    enabled_ = true;
    // current_ 会在 updateBounds/updateCosts 里每拍置真（见下）。
    current_ = false;
    RCLCPP_INFO(logger_, "[ped_corridor] enabled 已强制置 true");
  }

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * /*min_x*/, double * /*min_y*/, double * /*max_x*/, double * /*max_y*/) override
  {
    // **必须置 current_ = true**。Nav2 的 controller_server 每一拍先等
    // LayeredCostmap::isCurrent()（= 所有 layer 的 current_ 逻辑与）为真才
    // 进入 computeAndPublishVelocity；基类 Layer::current_ 默认 false，
    // 自研层若从不置真，控制器会永远在 isCurrent() 上空转（实测表现为：
    // 收到 follow_path 后 /cmd_vel_raw 一条不发、日志一句不打、机器人原地不动）。
    current_ = true;

    // 记下机器人位姿（master 坐标系），updateCosts 里用来留"安全岛"：
    // 栏杆带一旦把机器人自己所在的格子也标成 LETHAL，DWA 的每条轨迹都会判撞，
    // 机器人会永久卡在带子里出不来（mixed seed 502 实测停在 x=7.4 达 25 s）。
    robot_x_ = robot_x;
    robot_y_ = robot_y;
    robot_yaw_ = robot_yaw;
    robot_ok_ = true;

    // 不扩张更新窗口：局部代价地图是滚动窗口，7 m 外的走廊本来就写不进去，
    // 扩张窗口没有收益（早期误以为是它导致冻结，实为 current_ 未置真）。
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override
  {
    if ((++entry_n_ % 100) == 1) {
      RCLCPP_INFO(logger_, "[ped_corridor] updateCosts #%zu enabled=%d stage=%d grid=%d recv=%zu",
                  entry_n_, static_cast<int>(enabled_), stage_,
                  grid_ ? 1 : 0, recv_count_);
    }
    current_ = true;  // 同上：isCurrent() 的必要条件，不能只在 updateBounds 里置
    if (!enabled_ || stage_ < 3) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);

    // 1) 先按**世界坐标**把上一拍写过的格子擦回 FREE（滚动窗口原点每拍都在变，
    //    存栅格索引会指向别的格子——实测会把静态墙擦成空地）。
    //    只看**当前值**，不再回写 old_cost：old_cost 可能是 LETHAL（静态墙，或者别的层
    //    这一拍刚写的），回写它会让栏杆永久残留；反过来把 LETHAL 一律还原成 FREE 又会
    //    把静态墙擦掉（旧版就是这个 bug，墙在 FREE/LETHAL 之间来回抖）。
    //    这里的约定是：栏杆带的格子都在马路中间，除本层外不该有人写成 LETHAL/INSCRIBED，
    //    所以"当前值是本层写的那种高代价 → 还原成 FREE"是安全的。
    unsigned char * master = master_grid.getCharMap();
    for (const auto & rec : cells_) {
      unsigned int mx = 0, my = 0;
      if (!master_grid.worldToMap(rec.wx, rec.wy, mx, my)) {
        continue;
      }
      const size_t idx = master_grid.getIndex(mx, my);
      if (master[idx] == nav2_costmap_2d::LETHAL_OBSTACLE ||
        master[idx] == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        master[idx] = nav2_costmap_2d::FREE_SPACE;
      }
    }
    cells_.clear();

    // 1b) 把**上一拍走廊区域**里由 inflation 留下的 INSCRIBED 清回空闲。
    //     背景：Nav2 的 inflation 只加不减，走廊移走后它的膨胀环带会永久残留
    //     （实测车道点 (7.5,0) 的代价从 20 s 起一直是 99 不再回落，机器人就在那里卡死）。
    //     只清 INSCRIBED，不动 LETHAL（静态墙的致命格不能被擦），清完本拍的 inflation
    //     会立刻按当前走廊重新算一遍。
    if (prev_has_region_) {
      const double pad = master_grid.getResolution() * 2.0;
      unsigned int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
      if (master_grid.worldToMap(prev_min_x_ - pad, prev_min_y_ - pad, x0, y0) &&
        master_grid.worldToMap(prev_max_x_ + pad, prev_max_y_ + pad, x1, y1))
      {
        for (unsigned int yy = y0; yy <= y1; ++yy) {
          for (unsigned int xx = x0; xx <= x1; ++xx) {
            const size_t ii = master_grid.getIndex(xx, yy);
            if (master[ii] == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
              master[ii] = nav2_costmap_2d::FREE_SPACE;
            }
          }
        }
      }
    }

    if (stage_ < 4 || !grid_) {
      return;
    }

    // 2) 再把本拍落在更新窗口内的走廊格子写成 LETHAL
    const auto & info = grid_->info;
    if (info.width == 0 || info.height == 0) {
      return;
    }
    const double res = info.resolution;
    const double ox = info.origin.position.x;
    const double oy = info.origin.position.y;
    // 记录本拍走廊区域（下一拍据此清理 inflation 残留）；留出 0.7 m 余量覆盖膨胀半径
    bool any = false;
    size_t skip_out = 0, skip_win = 0, skip_keep = 0;
    double mnx = 0.0, mny = 0.0, mxx = 0.0, mxy = 0.0;
    for (unsigned int j = 0; j < info.height; ++j) {
      for (unsigned int i = 0; i < info.width; ++i) {
        const size_t gi = static_cast<size_t>(j) * info.width + i;
        if (gi >= grid_->data.size()) {
          continue;
        }
        if (static_cast<int>(grid_->data[gi]) < lethal_threshold_) {
          continue;
        }
        const double wx = ox + (static_cast<double>(i) + 0.5) * res;
        const double wy = oy + (static_cast<double>(j) + 0.5) * res;
        unsigned int mx = 0, my = 0;
        if (!master_grid.worldToMap(wx, wy, mx, my)) {
          ++skip_out;
          continue;
        }
        if (static_cast<int>(mx) < min_i || static_cast<int>(mx) > max_i ||
          static_cast<int>(my) < min_j || static_cast<int>(my) > max_j)
        {
          ++skip_win;
          continue;
        }
        // **身后退路**：机器人身后 ESCAPE_M 以内不落栏杆，正前方/两侧照常落。
        // 为什么不是"以机器人为圆心的安全岛"：那样机器人每一拍都能把正前方 0.85 m
        // 内的栏杆擦掉，于是它像推土机一样**一点点拱进落着的栏杆带**
        // （2026-11-17 实测 mixed_01/DWA：x 从 3.19 蹭到 4.69 用了 13 s，
        // 与行人最小间距只有 0.10 m）。只放身后，则"进不去、退得出"：
        // 机器人被罩住时还能倒出来，但绝不可能穿过栏杆带。
        if (robot_ok_) {
          const double dx = wx - robot_x_;
          const double dy = wy - robot_y_;
          const double dist = std::hypot(dx, dy);
          const double proj = dx * std::cos(robot_yaw_) + dy * std::sin(robot_yaw_);
          if (dist < escape_m_ && proj < 0.0) {
            ++skip_keep;
            continue;
          }
        }
        const size_t idx = master_grid.getIndex(mx, my);
        if (!any) {mnx = wx; mny = wy; mxx = wx; mxy = wy; any = true;}
        mnx = std::min(mnx, wx); mny = std::min(mny, wy);
        mxx = std::max(mxx, wx); mxy = std::max(mxy, wy);
        cells_.push_back({wx, wy, master[idx]});
        master[idx] = nav2_costmap_2d::LETHAL_OBSTACLE;
      }
    }
    if (any) {
      prev_min_x_ = mnx - 0.7; prev_min_y_ = mny - 0.7;
      prev_max_x_ = mxx + 0.7; prev_max_y_ = mxy + 0.7;
      prev_has_region_ = true;
    }
    // 回读校验：写完之后窗口里到底有多少 LETHAL / INSCRIBED（确认写进去了）
    size_t m_let = 0, m_ins = 0;
    for (int jj = min_j; jj <= max_j; ++jj) {
      for (int ii = min_i; ii <= max_i; ++ii) {
        const unsigned char v = master[master_grid.getIndex(
            static_cast<unsigned int>(ii), static_cast<unsigned int>(jj))];
        if (v == nav2_costmap_2d::LETHAL_OBSTACLE) {++m_let;} else if (
          v == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {++m_ins;}
      }
    }
    // 层内日志（诊断用）：确认每拍到底写没写进代价地图
    ++dbg_n_;
    if ((dbg_n_ % 100) == 1) {
      RCLCPP_INFO(logger_,
                  "[ped_corridor] 写入 %zu 格；栅格 %ux%u 原点(%.2f,%.2f) 收到 %zu 条；"
                  "跳过 窗外%zu 窗内越界%zu 身后退路%zu；窗口 i=%d..%d j=%d..%d；"
                  "机器人(%.2f,%.2f)；master 内 LETHAL=%zu INSCRIBED=%zu",
                  cells_.size(), info.width, info.height, ox, oy, recv_count_,
                  skip_out, skip_win, skip_keep, min_i, max_i, min_j, max_j,
                  robot_x_, robot_y_, m_let, m_ins);
    }
  }

  void reset() override
  {
    // 不拿 mutex_：本函数由"清空代价地图"服务线程在持 costmap 锁时调用，
    // 与更新线程会形成 ABBA 死锁
    cells_.clear();
  }

  void matchSize() override
  {
    cells_.clear();
  }

  bool isClearable() override {return true;}



  ~PedCorridorLayer() override
  {
    if (exec_) {
      exec_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

private:
  struct CellRec {double wx; double wy; unsigned char old_cost;};
  std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_;
  std::vector<CellRec> cells_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  std::shared_ptr<rclcpp::Node> reader_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread spin_thread_;
  std::string topic_{"/ped/corridor"};
  int lethal_threshold_{50};
  int stage_{4};
  bool prev_has_region_{false};
  double robot_x_{0.0}, robot_y_{0.0}, robot_yaw_{0.0};
  bool robot_ok_{false};
  double escape_m_{1.2};
  double prev_min_x_{0.0}, prev_min_y_{0.0}, prev_max_x_{0.0}, prev_max_y_{0.0};
  size_t recv_count_{0};
  size_t dbg_n_{0};
  size_t entry_n_{0};
};

}  // namespace robot_bringup

PLUGINLIB_EXPORT_CLASS(robot_bringup::PedCorridorLayer, nav2_costmap_2d::Layer)

// ---------------------------------------------------------------------------
// 对照实验用的**完全空**的层：什么都不做（不订阅、不读写代价地图）。
// 用来判断"局部代价地图多挂一个层"这件事本身会不会让 controller_server 停摆。
// ---------------------------------------------------------------------------
namespace robot_bringup
{
class EmptyProbeLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    const char * d = std::getenv("PED_EMPTY_DECLARE");
    bool do_decl = (d && std::string(d) == "1");
    RCLCPP_INFO(logger_, "[empty_probe] 初始化（declare=%d）", static_cast<int>(do_decl));
    if (do_decl) {
      auto node = node_.lock();
      // 只声明一个参数，用来判断"在 costmap 层里 declare_parameter"本身是不是触发器
      auto v = node->declare_parameter(name_ + ".enabled", true);
      RCLCPP_INFO(logger_, "[empty_probe] enabled=%d", static_cast<int>(v));
    }
  }
  void updateBounds(
    double, double, double, double *, double *, double *, double *) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override {}
  void reset() override {}
  bool isClearable() override {return true;}
};
}  // namespace robot_bringup

PLUGINLIB_EXPORT_CLASS(robot_bringup::EmptyProbeLayer, nav2_costmap_2d::Layer)
