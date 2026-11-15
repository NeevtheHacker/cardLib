// fake_pros.cpp
// Why: lets the chassis code run on a laptop: every PROS task is a real thread, time is the
//      laptop's clock, and mutexes and task notifications are standard-library ones.
// Owns: the fake tasks and their stop flag. Test-only; never part of the robot build.
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "fakes/fake_devices.hpp"
#include "pros/adi.h"
#include "pros/device.h"
#include "pros/misc.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

namespace fake {
namespace {

using Clock = std::chrono::steady_clock;
const Clock::time_point START = Clock::now();

// Thrown inside a task thread by delay() once stopTasks() has been called, so the task's
// endless loop unwinds and the thread can be joined.
struct TaskStopped {};

struct FakeTask {
  std::thread thread;
  std::mutex mutex;
  std::condition_variable notified;
  std::uint32_t notifications = 0;
};

std::mutex g_tasksMutex;
std::vector<std::unique_ptr<FakeTask>> g_tasks;
std::atomic<bool> g_stopping{false};
FakeTask g_mainTask;
thread_local FakeTask* t_current = nullptr;

FakeTask& currentTask() {
  return t_current != nullptr ? *t_current : g_mainTask;
}

void throwIfStopping() {
  if (t_current != nullptr && g_stopping.load()) {
    throw TaskStopped{};
  }
}

void sleepUntil(Clock::time_point wake) {
  constexpr auto SLICE = std::chrono::milliseconds(5);
  while (Clock::now() < wake) {
    throwIfStopping();
    std::this_thread::sleep_for(std::min<Clock::duration>(SLICE, wake - Clock::now()));
  }
  throwIfStopping();
}

}  // namespace

void stopTasks() {
  g_stopping.store(true);
  std::vector<std::unique_ptr<FakeTask>> tasks;
  {
    const std::lock_guard<std::mutex> lock(g_tasksMutex);
    tasks.swap(g_tasks);
  }
  for (const auto& task : tasks) {
    task->notified.notify_all();
    task->thread.join();
  }
  g_stopping.store(false);
}

}  // namespace fake

namespace pros {
namespace c {
extern "C" {

uint32_t millis(void) {
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(fake::Clock::now() - fake::START)
          .count());
}

uint64_t micros(void) {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(fake::Clock::now() - fake::START)
          .count());
}

void delay(const uint32_t milliseconds) {
  fake::sleepUntil(fake::Clock::now() + std::chrono::milliseconds(milliseconds));
}

task_t task_create(task_fn_t function, void* const parameters, uint32_t, const uint16_t,
                   const char* const) {
  auto task = std::make_unique<fake::FakeTask>();
  fake::FakeTask* raw = task.get();
  raw->thread = std::thread([raw, function, parameters] {
    fake::t_current = raw;
    try {
      function(parameters);
    } catch (const fake::TaskStopped&) {
    }
  });
  const std::lock_guard<std::mutex> lock(fake::g_tasksMutex);
  fake::g_tasks.push_back(std::move(task));
  return raw;
}

task_t task_get_current() {
  return &fake::currentTask();
}

uint32_t task_notify(task_t task) {
  auto* target = static_cast<fake::FakeTask*>(task);
  {
    const std::lock_guard<std::mutex> lock(target->mutex);
    ++target->notifications;
  }
  target->notified.notify_all();
  return 1;
}

uint32_t task_notify_take(bool clear_on_exit, uint32_t timeout) {
  fake::FakeTask& self = fake::currentTask();
  std::unique_lock<std::mutex> lock(self.mutex);
  const auto wake = fake::Clock::now() + std::chrono::milliseconds(timeout);
  while (self.notifications == 0 && fake::Clock::now() < wake) {
    self.notified.wait_until(lock,
                             std::min(wake, fake::Clock::now() + std::chrono::milliseconds(5)));
    if (fake::t_current != nullptr && fake::g_stopping.load()) {
      throw fake::TaskStopped{};
    }
  }
  const uint32_t count = self.notifications;
  self.notifications = clear_on_exit ? 0 : (count > 0 ? count - 1 : 0);
  return count;
}

double battery_get_capacity(void) {
  return fake::batteryPercent();
}

v5_device_e_t get_plugged_type(uint8_t port) {
  return static_cast<v5_device_e_t>(fake::pluggedType(port));
}

adi_port_config_e_t adi_port_get_config(uint8_t) {
  return E_ADI_DIGITAL_OUT;
}

}  // extern "C"
}  // namespace c

namespace rtos {

mutex_t Mutex::lazy_init() {
  mutex_t existing = mutex.load();
  if (existing != nullptr) {
    return existing;
  }
  auto* created = new std::timed_mutex();
  mutex_t expected = nullptr;
  if (!mutex.compare_exchange_strong(expected, created)) {
    delete created;
    return expected;
  }
  return created;
}

Mutex::~Mutex() {
  delete static_cast<std::timed_mutex*>(mutex.load());
}

bool Mutex::take() {
  return take(TIMEOUT_MAX);
}

bool Mutex::take(std::uint32_t timeout) {
  auto* raw = static_cast<std::timed_mutex*>(lazy_init());
  if (timeout == TIMEOUT_MAX) {
    raw->lock();
    return true;
  }
  return raw->try_lock_for(std::chrono::milliseconds(timeout));
}

bool Mutex::give() {
  static_cast<std::timed_mutex*>(lazy_init())->unlock();
  return true;
}

void Mutex::lock() {
  take();
}

void Mutex::unlock() {
  give();
}

bool Mutex::try_lock() {
  return take(0);
}

void Task::delay_until(std::uint32_t* const prev_time, const std::uint32_t delta) {
  const std::uint32_t wakeMs = *prev_time + delta;
  fake::sleepUntil(fake::START + std::chrono::milliseconds(wakeMs));
  *prev_time = wakeMs;
}

}  // namespace rtos
}  // namespace pros
