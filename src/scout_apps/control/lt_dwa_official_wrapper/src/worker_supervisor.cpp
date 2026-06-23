#include "lt_dwa_official_wrapper/worker_supervisor.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace lt_dwa_official_wrapper {
namespace {

constexpr int kExecFailureCode = 127;

void SetNonBlocking(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  }
}

void DrainPipe(int fd, std::string* output) {
  char buffer[4096];
  while (true) {
    const ssize_t n = read(fd, buffer, sizeof(buffer));
    if (n > 0) {
      output->append(buffer, static_cast<std::size_t>(n));
      continue;
    }
    break;
  }
}

std::vector<char*> MakeArgv(const std::string& executable,
                            const std::vector<std::string>& args,
                            std::vector<std::string>* storage) {
  storage->clear();
  storage->reserve(args.size() + 1);
  storage->push_back(executable);
  for (const auto& arg : args) {
    storage->push_back(arg);
  }

  std::vector<char*> argv;
  argv.reserve(storage->size() + 1);
  for (auto& value : *storage) {
    argv.push_back(&value[0]);
  }
  argv.push_back(nullptr);
  return argv;
}

}  // namespace

WorkerRunResult WorkerSupervisor::Run(const std::string& executable,
                                      const std::vector<std::string>& args,
                                      double timeout_sec) const {
  WorkerRunResult result;

  int pipe_fd[2];
  if (pipe(pipe_fd) != 0) {
    result.status = WrapperStatus::kCoreProcessExited;
    result.reason = std::string("pipe failed: ") + std::strerror(errno);
    return result;
  }

  const pid_t pid = fork();
  if (pid < 0) {
    close(pipe_fd[0]);
    close(pipe_fd[1]);
    result.status = WrapperStatus::kCoreProcessExited;
    result.reason = std::string("fork failed: ") + std::strerror(errno);
    return result;
  }

  if (pid == 0) {
    close(pipe_fd[0]);
    dup2(pipe_fd[1], STDOUT_FILENO);
    dup2(pipe_fd[1], STDERR_FILENO);
    close(pipe_fd[1]);

    std::vector<std::string> storage;
    std::vector<char*> argv = MakeArgv(executable, args, &storage);
    execv(executable.c_str(), argv.data());
    _exit(kExecFailureCode);
  }

  close(pipe_fd[1]);
  SetNonBlocking(pipe_fd[0]);

  int wait_status = 0;
  bool exited = false;
  const auto start = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::duration<double>(timeout_sec > 0.0 ? timeout_sec : 1.0);

  while (!exited) {
    DrainPipe(pipe_fd[0], &result.output);

    const pid_t wait_result = waitpid(pid, &wait_status, WNOHANG);
    if (wait_result == pid) {
      exited = true;
      break;
    }

    const auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed > timeout) {
      result.timed_out = true;
      kill(pid, SIGKILL);
      waitpid(pid, &wait_status, 0);
      exited = true;
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  DrainPipe(pipe_fd[0], &result.output);
  close(pipe_fd[0]);

  if (WIFEXITED(wait_status)) {
    result.exit_code = WEXITSTATUS(wait_status);
  } else if (WIFSIGNALED(wait_status)) {
    result.term_signal = WTERMSIG(wait_status);
  }

  if (result.timed_out) {
    result.status = WrapperStatus::kCoreProcessExited;
    result.reason = "worker timed out";
    result.valid_response = false;
    return result;
  }

  const auto response = ParseWorkerResponse(result.output);
  result.valid_response = response.valid;
  result.status = response.status;
  result.reason = response.reason;
  result.has_command = response.has_command;
  result.command_v = response.command_v;
  result.command_w = response.command_w;
  result.has_core_return = response.has_core_return;
  result.core_return = response.core_return;

  if (!response.valid) {
    result.status = WrapperStatus::kCoreProcessExited;
    if (result.term_signal != 0) {
      result.reason = "worker terminated by signal";
    } else if (result.exit_code == kExecFailureCode) {
      result.reason = "worker exec failed";
    } else {
      result.reason = response.reason;
    }
  }

  return result;
}

}  // namespace lt_dwa_official_wrapper
