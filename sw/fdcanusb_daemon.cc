// Copyright 2020 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This can be compiled with:
//
// g++ -Wall -g -O2 fdcanusb_daemon.cc -o fdcanusb_daemon
//
//
// To create a virtual CAN interface, one option would be to use:
//
//  sudo ip link add name vcan0 type vcan
//  sudo ip link set vcan0 up


#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/serial.h>
#include <net/if.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <syslog.h>
#include <unistd.h>
#include <termios.h>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

namespace {
void FakeSyslog(int priority, const char* format, ...) {
  va_list ap;

  ::printf("[%d] ", priority);
  va_start(ap, format);
  ::vprintf(format, ap);
  va_end(ap);
  ::printf("\n");
}

void PrintUsage(char* prg) {
  std::cerr << prg << " - userspace daemon for mjbots fdcanusb\n";
  std::cerr << "\nUsage: " << prg << " [options] <tty> [canif-name]\n\n";
  std::cerr << "  -F  (stay in foreground; no daemonize)\n";
  std::cerr << "  -v  (verbose; print the contents of all frames)\n";
  std::cerr << "  -h  (show this usage message)\n";
  std::exit(EXIT_FAILURE);
}

void ErrorIf(bool value, const char* format, ...) {
  va_list ap;

  if (value) {
    va_start(ap, format);
    ::vfprintf(stderr, format, ap);
    va_end(ap);
    ::fprintf(stderr, "\n");

    ::perror("");
    std::exit(EXIT_FAILURE);
  } else {
    // Just consume this.
    char buf[1] = {};
    va_start(ap, format);
    ::vsnprintf(buf, sizeof(buf), format, ap);
    va_end(ap);
  }
}

void SetNonblock(int fd) {
  int flags = ::fcntl(fd, F_GETFL, 0);
  ErrorIf(flags < 0, "error getting flags");
  flags |= O_NONBLOCK;
  ErrorIf(::fcntl(fd, F_SETFL, flags), "error setting flags");
}

void Append(std::ostream* ostr, const char* format, ...) {
  char buf[1024] = {};
  va_list ap;

  va_start(ap, format);
  int count = ::vsnprintf(buf, sizeof(buf), format, ap);
  va_end(ap);
  ostr->write(buf, count);
}

int ParseHexNybble(char c) {
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
  if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  return -1;
}

int ParseHexByte(const char* value) {
  int high = ParseHexNybble(value[0]);
  if (high < 0) { return high; }
  int low = ParseHexNybble(value[1]);
  if (low < 0) { return low; }
  return (high << 4) | low;
}

int ParseCanData(const std::string& data, uint8_t* out) {
  size_t to_read = std::min<size_t>(64 * 2, data.size());
  for (size_t i = 0; i < to_read; i += 2) {
    out[i / 2] = ParseHexByte(&data[i]);
  }
  return data.size() / 2;
}

bool StartsWith(const std::string& haystack, const std::string& needle) {
  return haystack.substr(0, needle.size()) == needle;
}

class Tokenizer {
 public:
  Tokenizer(const std::string& source, const char* delimiters)
      : source_(source),
        delimiters_(delimiters),
        position_(source_.cbegin()) {}

  std::string next() {
    if (position_ == source_.end()) { return std::string(); }

    const auto start = position_;
    auto my_next = position_;
    bool found = false;
    for (; my_next != source_.end(); ++my_next) {
      if (std::strchr(delimiters_, *my_next) != nullptr) {
        position_ = my_next;
        ++position_;
        found = true;
        break;
      }
    }
    if (!found) { position_ = my_next; }
    return std::string(&*start, my_next - start);
  }

  std::string remaining() const {
    return std::string(&*position_, source_.end() - position_);
  }

 private:
  const std::string source_;
  const char* const delimiters_;
  std::string::const_iterator position_;
};

}

int main(int argc, char** argv) {
  std::string tty;
  std::string ifname;
  bool run_as_daemon = true;
  bool verbose = false;

  typedef void (*syslog_t)(int priority, const char* format, ...);
  syslog_t syslogger = ::syslog;

  int opt = 0;
  while ((opt = getopt(argc, argv, "hFv")) != -1) {
    switch (opt) {
      case 'F': {
        run_as_daemon = false;
        break;
      }
      case 'h': {
        PrintUsage(argv[0]);
        break;
      }
      case 'v': {
        verbose = true;
        break;
      }
    }
  }

  if (run_as_daemon) {
    openlog("fdcanusb_daemon", LOG_PID, LOG_LOCAL5);
  } else {
    syslogger = FakeSyslog;
  }

  if (argv[optind] == nullptr) {
    PrintUsage(argv[0]);  // does not return
  }
  tty = argv[optind];

  if (argv[optind + 1] != nullptr) {
    ifname = argv[optind + 1];
  }

  // Open the CDC device.

  syslogger(LOG_INFO, "starting on device %s", tty.c_str());

  int fd = ::open(tty.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
  ErrorIf(fd < 0, "failed to open device %s", tty.c_str());

  // Set low-latency.
  {
    struct serial_struct serial = {};
    ErrorIf(::ioctl(fd, TIOCGSERIAL, &serial) < 0, "error getting serial flags");
    serial.flags |= ASYNC_LOW_LATENCY;
    ErrorIf(::ioctl(fd, TIOCSSERIAL, &serial) < 0, "error setting low latency");
  }

  // Disable all kernel-side character processing.
  {
    struct termios tty;

    ErrorIf(::tcgetattr(fd, &tty) != 0, "error getting termios");

    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    ErrorIf(::tcsetattr(fd, TCSANOW, &tty) != 0, "error setting termios");
  }

  SetNonblock(fd);


  // Now open the CAN interface.
  int socket = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  ErrorIf(socket < 0, "error opening CAN socket");

  SetNonblock(socket);

  struct ifreq ifr = {};
  std::strncpy(&ifr.ifr_name[0], ifname.c_str(), sizeof(ifr.ifr_name) - 1);
  ErrorIf(::ioctl(socket, SIOCGIFINDEX, &ifr) < 0,
          "could not find CAN '%s'", ifname.c_str());

  int enable_canfd = 1;
  ErrorIf(::setsockopt(socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                       &enable_canfd, sizeof(enable_canfd)) != 0,
          "could not set CAN-FD mode");

  struct sockaddr_can addr = {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  ErrorIf(::bind(socket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0,
          "could not bind to CAN if");

  if (run_as_daemon) {
    if (daemon(0, 0)) {
      syslogger(LOG_ERR, "failed to daemonize");
      std::exit(EXIT_FAILURE);
    }
  }

  fd_set rfds = {};
  struct canfd_frame recv_frame = {};
  std::ostringstream fdcanusb_buf;
  while (true) {
    // Look for things to read from either interface, bridge them as
    // necessary.
    FD_ZERO(&rfds);
    FD_SET(socket, &rfds);
    FD_SET(fd, &rfds);
    const int ret = ::select(
        std::max(fd, socket) + 1, &rfds, nullptr, nullptr, nullptr);
    ErrorIf(ret < 0, "error in select");

    if (FD_ISSET(socket, &rfds)) {
      // Read a CAN message, forward to fdcanusb.
      ErrorIf(::read(socket, &recv_frame, sizeof(recv_frame)) < 0,
              "error reading CAN frame");
      std::ostringstream cmd;
      Append(&cmd, "can send %X ", recv_frame.can_id & 0x1fffffff);
      for (int i = 0; i < recv_frame.len; i++) {
        Append(&cmd, "%02X", static_cast<int>(recv_frame.data[i]));
      }
      Append(&cmd, "\n");

      if (verbose) {
        // DEBUG
        std::cout << "Writing: " << cmd.str();
      }

      ErrorIf(::write(fd, cmd.str().c_str(), cmd.str().size()) < 0,
              "error writing to fdcanusb");
    } else if (FD_ISSET(fd, &rfds)) {
      // Read something from fdcanusb, if we have a frame, forward to CAN.
      char buf[1024] = {};
      int count = 0;
      ErrorIf((count = ::read(fd, buf, sizeof(buf))) < 0,
              "error reading fdcanusb");
      fdcanusb_buf.write(buf, count);

      size_t line_end = 0;
      while ((line_end = fdcanusb_buf.str().find_first_of("\r\n"))
             != std::string::npos) {
        std::string all_buf = fdcanusb_buf.str();
        std::string this_msg = all_buf.substr(0, line_end);
        fdcanusb_buf.str(all_buf.substr(line_end + 1));

        if (this_msg == "") {
          continue;
        }

        if (verbose) {
          // DEBUG
          std::cout << "Received: " << this_msg << "\n";
        }

        if (this_msg == "OK") {
          // Also do nothing.
        } else if (StartsWith(this_msg, "rcv ")) {
          // A frame we can use.
          Tokenizer tokenizer(this_msg, " ");
          const auto rcv = tokenizer.next();
          const auto address = tokenizer.next();
          const auto data = tokenizer.next();
          if (address.empty() || data.empty()) { continue; }

          struct canfd_frame send_frame = {};
          send_frame.can_id = std::stoul(address, nullptr, 16);
          send_frame.len = ParseCanData(data, send_frame.data);

          ErrorIf(::write(socket, &send_frame, sizeof(send_frame)) < 0,
                  "error writing CAN");

        } else {
          // Something unknown, ignore for now.
        }
      }
    }
  }

  return 0;
}
