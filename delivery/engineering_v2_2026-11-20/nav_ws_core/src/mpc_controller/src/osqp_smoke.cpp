// W5 D4: standalone OSQP C smoke with a 160-variable sparse QP (N=40).
// Mirrors the W4 condensed QP layout: 3 controls per step + one slack per step.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

extern "C" {
#include "osqp.h"
}

namespace
{

constexpr int kHorizon = 40;
constexpr int kControlVars = 3 * kHorizon;             // 120
constexpr int kSlackVars = kHorizon;                   // 40
constexpr int kNumVars = kControlVars + kSlackVars;    // 160
constexpr int kBoxRows = kControlVars;                 // 120
constexpr int kRateRows = kControlVars;                // 120
constexpr int kSlackRows = kSlackVars;                 // 40
constexpr int kObstacleRows = kHorizon;                // 40
constexpr int kNumCons = kBoxRows + kRateRows + kSlackRows + kObstacleRows;  // 320
constexpr int kRuns = 1000;
constexpr double kDt = 0.05;
constexpr double kAccMax = 1.0;
constexpr double kAlphaMax = 2.0;
constexpr double kVxMax = 1.0;
constexpr double kVyMax = 1.0;
constexpr double kWzMax = 1.5;
constexpr double kRhoSlack = 80.0;
constexpr double kSlackReg = 1e-8;  // same slack regularization as the W4 prototype
constexpr const char * kOsqpCommit = "0dd00a578cf1c2691c5c379965d504c75bf6cfad";
constexpr const char * kQdldlCommit = "7d16b70a10a152682204d745d814b6eb63dc5cd2";

struct SparseMatrix
{
  std::vector<c_float> x;
  std::vector<c_int> i;
  std::vector<c_int> p;
};

struct Workspace
{
  OSQPWorkspace * work{nullptr};
  csc * P{nullptr};
  csc * A{nullptr};
  std::vector<c_float> q;
  std::vector<c_float> l;
  std::vector<c_float> u;
};

SparseMatrix buildP()
{
  SparseMatrix matrix;
  matrix.x.resize(kNumVars);
  matrix.i.resize(kNumVars);
  matrix.p.resize(kNumVars + 1);
  for (int k = 0; k < kNumVars; ++k) {
    matrix.p[k] = k;
    matrix.i[k] = k;
    if (k < kControlVars) {
      matrix.x[k] = (k % 3 == 2) ? 1.0 : 2.0;
    } else {
      matrix.x[k] = kSlackReg;
    }
  }
  matrix.p[kNumVars] = kNumVars;
  return matrix;
}

SparseMatrix buildA()
{
  std::vector<std::vector<std::pair<int, double>>> columns(kNumVars);
  constexpr int obstacle_start = kBoxRows + kRateRows + kSlackRows;

  for (int k = 0; k < kHorizon; ++k) {
    for (int component = 0; component < 3; ++component) {
      const int column = 3 * k + component;
      columns[column].push_back({column, 1.0});  // box row
      columns[column].push_back({kBoxRows + column, 1.0});  // rate row: u_k
      if (k > 0) {
        // rate row of step k is u_k - u_{k-1}: the -1 belongs to the
        // previous step's column, not the current one.
        columns[column - 3].push_back({kBoxRows + column, -1.0});
      }
      if (component < 2) {
        const double theta = 0.1 * k;
        columns[column].push_back(
          {obstacle_start + k, component == 0 ? std::cos(theta) : std::sin(theta)});
      }
    }

    const int slack_column = kControlVars + k;
    columns[slack_column].push_back({kBoxRows + kRateRows + k, 1.0});  // slack >= 0
    columns[slack_column].push_back({obstacle_start + k, 1.0});         // soft obstacle
  }

  SparseMatrix matrix;
  matrix.p.resize(kNumVars + 1);
  for (int column = 0; column < kNumVars; ++column) {
    auto & entries = columns[column];
    std::sort(
      entries.begin(), entries.end(),
      [](const auto & lhs, const auto & rhs) { return lhs.first < rhs.first; });
    matrix.p[column] = static_cast<c_int>(matrix.x.size());
    for (const auto & entry : entries) {
      matrix.i.push_back(static_cast<c_int>(entry.first));
      matrix.x.push_back(static_cast<c_float>(entry.second));
    }
  }
  matrix.p[kNumVars] = static_cast<c_int>(matrix.x.size());
  return matrix;
}

std::vector<c_float> buildLowerBounds()
{
  std::vector<c_float> lower(kNumCons, 0.0);
  for (int column = 0; column < kControlVars; ++column) {
    const int component = column % 3;
    lower[column] = component == 2 ? -kWzMax : (component == 1 ? -kVyMax : -kVxMax);
    const double rate = component == 2 ? kAlphaMax * kDt : kAccMax * kDt;
    lower[kBoxRows + column] = static_cast<c_float>(-rate);
  }
  for (int k = 0; k < kSlackVars; ++k) {
    lower[kBoxRows + kRateRows + k] = 0.0;
    lower[kBoxRows + kRateRows + kSlackRows + k] = 0.5;
  }
  return lower;
}

std::vector<c_float> buildUpperBounds()
{
  std::vector<c_float> upper(kNumCons, OSQP_INFTY);
  for (int column = 0; column < kControlVars; ++column) {
    const int component = column % 3;
    upper[column] = component == 2 ? kWzMax : (component == 1 ? kVyMax : kVxMax);
    const double rate = component == 2 ? kAlphaMax * kDt : kAccMax * kDt;
    upper[kBoxRows + column] = static_cast<c_float>(rate);
  }
  for (int k = 0; k < kSlackVars; ++k) {
    upper[kBoxRows + kRateRows + k] = OSQP_INFTY;
    upper[kBoxRows + kRateRows + kSlackRows + k] = OSQP_INFTY;
  }
  return upper;
}

void updateLinearCost(std::vector<c_float> & q, int iteration)
{
  for (int k = 0; k < kHorizon; ++k) {
    const double phase = 0.02 * iteration + 0.05 * k;
    const double ref_vx = 0.4 * std::sin(phase);
    const double ref_vy = 0.2 * std::cos(phase);
    const double ref_wz = 0.3 * std::sin(0.5 * phase);
    q[3 * k] = static_cast<c_float>(-2.0 * ref_vx);
    q[3 * k + 1] = static_cast<c_float>(-2.0 * ref_vy);
    q[3 * k + 2] = static_cast<c_float>(-1.0 * ref_wz);
  }
  for (int k = 0; k < kSlackVars; ++k) {
    q[kControlVars + k] = static_cast<c_float>(kRhoSlack);
  }
}

OSQPSettings makeSettings(bool verbose = false, int max_iter = 10000)
{
  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = verbose ? 1 : 0;
  settings.eps_abs = 1e-3;
  settings.eps_rel = 1e-3;
  settings.max_iter = max_iter;
  settings.polish = 1;
  settings.warm_start = 1;
  return settings;
}

const char * statusName(c_int status)
{
  switch (status) {
    case OSQP_SOLVED: return "solved";
    case OSQP_SOLVED_INACCURATE: return "solved_inaccurate";
    case OSQP_MAX_ITER_REACHED: return "max_iter_reached";
    case OSQP_PRIMAL_INFEASIBLE: return "primal_infeasible";
    case OSQP_DUAL_INFEASIBLE: return "dual_infeasible";
    case OSQP_SIGINT: return "sigint";
    case OSQP_NON_CVX: return "non_cvx";
    case OSQP_UNSOLVED: return "unsolved";
    default: return "other";
  }
}

struct Stats
{
  double mean{0.0};
  double p95{0.0};
  double max{0.0};
};

Stats computeStats(const std::vector<double> & values)
{
  Stats stats;
  if (values.empty()) {
    return stats;
  }
  stats.mean = std::accumulate(values.begin(), values.end(), 0.0) /
    static_cast<double>(values.size());
  std::vector<double> sorted = values;
  std::sort(sorted.begin(), sorted.end());
  stats.p95 = sorted[static_cast<std::size_t>(0.95 * (sorted.size() - 1))];
  stats.max = sorted.back();
  return stats;
}

bool solveFailurePath()
{
  // Deliberately infeasible but row-wise valid:
  // x0 + x1 >= 1 and -(x0 + x1) >= 0  =>  x0 + x1 >= 1 and x0 + x1 <= 0.
  constexpr int n = 2;
  constexpr int m = 2;
  std::vector<c_float> P_x{1.0, 1.0};
  std::vector<c_int> P_i{0, 1};
  std::vector<c_int> P_p{0, 1, 2};
  std::vector<c_float> q{0.0, 0.0};
  std::vector<c_float> A_x{1.0, 1.0, 1.0, 1.0};
  std::vector<c_int> A_i{0, 1, 0, 1};
  std::vector<c_int> A_p{0, 2, 4};
  std::vector<c_float> l{1.0, -OSQP_INFTY};
  std::vector<c_float> u{OSQP_INFTY, 0.0};

  csc * P = csc_matrix(n, n, 2, P_x.data(), P_i.data(), P_p.data());
  csc * A = csc_matrix(m, n, 2, A_x.data(), A_i.data(), A_p.data());
  OSQPData data;
  data.n = n;
  data.m = m;
  data.P = P;
  data.q = q.data();
  data.A = A;
  data.l = l.data();
  data.u = u.data();

  OSQPSettings settings = makeSettings();
  OSQPWorkspace * work = nullptr;
  const c_int setup_status = osqp_setup(&work, &data, &settings);
  if (setup_status != 0 || work == nullptr) {
    std::printf(
      "FAILURE PATH: setup error %lld\n", static_cast<long long>(setup_status));
    c_free(P);
    c_free(A);
    return false;
  }

  osqp_solve(work);
  const c_int status = work->info->status_val;
  std::printf(
    "FAILURE PATH: status=%lld (%s), iterations=%lld\n",
    static_cast<long long>(status), statusName(status),
    static_cast<long long>(work->info->iter));
  const bool failed_as_expected =
    status != OSQP_SOLVED && status != OSQP_SOLVED_INACCURATE;
  osqp_cleanup(work);
  c_free(P);
  c_free(A);
  return failed_as_expected;
}

bool setupFailureDetected()
{
  // Valid QP data, invalid settings (scaling must be nonnegative).
  constexpr int n = 2;
  constexpr int m = 1;
  std::vector<c_float> P_x{1.0, 1.0};
  std::vector<c_int> P_i{0, 1};
  std::vector<c_int> P_p{0, 1, 2};
  std::vector<c_float> q{0.0, 0.0};
  std::vector<c_float> A_x{1.0, 1.0};
  std::vector<c_int> A_i{0, 0};
  std::vector<c_int> A_p{0, 1, 2};
  std::vector<c_float> l{-OSQP_INFTY};
  std::vector<c_float> u{OSQP_INFTY};

  csc * P = csc_matrix(n, n, 2, P_x.data(), P_i.data(), P_p.data());
  csc * A = csc_matrix(m, n, 2, A_x.data(), A_i.data(), A_p.data());
  OSQPData data;
  data.n = n;
  data.m = m;
  data.P = P;
  data.q = q.data();
  data.A = A;
  data.l = l.data();
  data.u = u.data();

  OSQPSettings settings = makeSettings();
  settings.scaling = -1;  // deliberate validation failure
  OSQPWorkspace * work = nullptr;
  const c_int setup_status = osqp_setup(&work, &data, &settings);
  const bool detected = setup_status != 0 && work == nullptr;
  std::printf(
    "SETUP FAILURE PATH: setup_status=%lld (%s), detected=%d\n",
    static_cast<long long>(setup_status),
    setup_status == OSQP_SETTINGS_VALIDATION_ERROR ? "settings_validation" : "other",
    detected ? 1 : 0);
  if (work != nullptr) {
    osqp_cleanup(work);
  }
  c_free(P);
  c_free(A);
  return detected;
}

}  // namespace

int main(int argc, char ** argv)
{
  int runs = kRuns;
  int max_iter = 10000;
  bool verbose = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--verbose") {
      verbose = true;
    } else if (arg == "--runs" && i + 1 < argc) {
      runs = std::atoi(argv[++i]);
    } else if (arg == "--max-iter" && i + 1 < argc) {
      max_iter = std::atoi(argv[++i]);
    }
  }
  if (runs <= 0 || max_iter <= 0) {
    std::printf("invalid --runs/--max-iter value\n");
    return 1;
  }

  SparseMatrix P_data = buildP();
  SparseMatrix A_data = buildA();
  std::vector<c_float> q(kNumVars, 0.0);
  std::vector<c_float> lower = buildLowerBounds();
  std::vector<c_float> upper = buildUpperBounds();
  updateLinearCost(q, 0);

  csc * P = csc_matrix(
    kNumVars, kNumVars, static_cast<c_int>(P_data.x.size()),
    P_data.x.data(), P_data.i.data(), P_data.p.data());
  csc * A = csc_matrix(
    kNumCons, kNumVars, static_cast<c_int>(A_data.x.size()),
    A_data.x.data(), A_data.i.data(), A_data.p.data());

  OSQPData data;
  data.n = kNumVars;
  data.m = kNumCons;
  data.P = P;
  data.q = q.data();
  data.A = A;
  data.l = lower.data();
  data.u = upper.data();

  OSQPSettings settings = makeSettings(verbose, max_iter);
  OSQPWorkspace * work = nullptr;
  const c_int setup_status = osqp_setup(&work, &data, &settings);
  if (setup_status != 0 || work == nullptr) {
    std::printf("OSQP setup failed: %lld\n", static_cast<long long>(setup_status));
    c_free(P);
    c_free(A);
    return 1;
  }

  std::vector<double> solve_ms;
  std::vector<double> update_ms;
  std::vector<double> cycle_ms;
  std::vector<int> iterations;
  solve_ms.reserve(runs);
  update_ms.reserve(runs);
  cycle_ms.reserve(runs);
  iterations.reserve(runs);
  int failures = 0;
  int inaccurate = 0;

  for (int run = 0; run < runs; ++run) {
    updateLinearCost(q, run);

    const auto update_start = std::chrono::steady_clock::now();
    const c_int update_status = osqp_update_lin_cost(work, q.data());
    const auto update_stop = std::chrono::steady_clock::now();
    const double update_elapsed_ms =
      std::chrono::duration<double, std::milli>(update_stop - update_start).count();
    if (update_status != 0) {
      ++failures;
      continue;
    }

    const auto solve_start = std::chrono::steady_clock::now();
    const c_int solve_status = osqp_solve(work);
    const auto solve_stop = std::chrono::steady_clock::now();
    const double solve_elapsed_ms =
      std::chrono::duration<double, std::milli>(solve_stop - solve_start).count();

    solve_ms.push_back(solve_elapsed_ms);
    update_ms.push_back(update_elapsed_ms);
    cycle_ms.push_back(update_elapsed_ms + solve_elapsed_ms);
    iterations.push_back(static_cast<int>(work->info->iter));

    const c_int status = work->info->status_val;
    if (status == OSQP_SOLVED_INACCURATE) {
      ++inaccurate;
    }
    if (solve_status != 0 || (status != OSQP_SOLVED && status != OSQP_SOLVED_INACCURATE)) {
      ++failures;
    }
  }

  const Stats solve_stats = computeStats(solve_ms);
  const Stats update_stats = computeStats(update_ms);
  const Stats cycle_stats = computeStats(cycle_ms);
  const double mean_iter = iterations.empty() ? 0.0 :
    std::accumulate(iterations.begin(), iterations.end(), 0.0) /
    static_cast<double>(iterations.size());
  const int max_iterations = iterations.empty() ? 0 :
    *std::max_element(iterations.begin(), iterations.end());

  std::printf("OSQP C version: %s\n", osqp_version());
  std::printf("vendored OSQP commit: %s\n", kOsqpCommit);
  std::printf("vendored qdldl commit: %s\n", kQdldlCommit);
  std::printf(
    "QP: n=%d m=%d nnz(P)=%zu nnz(A)=%zu N=%d\n",
    kNumVars, kNumCons, P_data.x.size(), A_data.x.size(), kHorizon);
  std::printf(
    "settings: eps_abs=%.1e eps_rel=%.1e max_iter=%d warm_start=1 polish=1 "
    "slack_reg=%.1e\n",
    1e-3, 1e-3, max_iter, kSlackReg);
  std::printf(
    "runs=%d hard_failures=%d solved_inaccurate=%d\n", runs, failures, inaccurate);
  std::printf(
    "solver-only solve_ms mean=%.3f p95=%.3f max=%.3f\n",
    solve_stats.mean, solve_stats.p95, solve_stats.max);
  std::printf(
    "update-only update_ms mean=%.3f p95=%.3f max=%.3f\n",
    update_stats.mean, update_stats.p95, update_stats.max);
  std::printf(
    "update+solve_ms mean=%.3f p95=%.3f max=%.3f\n",
    cycle_stats.mean, cycle_stats.p95, cycle_stats.max);
  std::printf("iterations mean=%.2f max=%d\n", mean_iter, max_iterations);
  std::printf(
    "last solve: status=%lld (%s) pri_res=%.2e dua_res=%.2e\n",
    static_cast<long long>(work->info->status_val), work->info->status,
    work->info->pri_res, work->info->dua_res);
  if (solve_stats.p95 >= 50.0 || cycle_stats.p95 >= 50.0) {
    std::printf("WARN: p95 >= 50 ms (20 Hz budget; whole W6 cycle must be measured)\n");
  }
  if (inaccurate > 0) {
    std::printf(
      "WARN: OSQP_SOLVED_INACCURATE counted separately; W6 must judge residuals/safety\n");
  }

  osqp_cleanup(work);
  c_free(P);
  c_free(A);

  const bool setup_failure_ok = setupFailureDetected();
  const bool failure_ok = solveFailurePath();
  std::printf("failure paths return status (caller must fall back to zero velocity)\n");
  const bool pass = setup_failure_ok && failure_ok && failures == 0;
  const char * result = !pass ? "FAIL" :
    (inaccurate > 0 ? "PASS_WITH_WARNINGS" : "PASS");
  std::printf("RESULT: %s\n", result);
  return pass ? 0 : 1;
}
