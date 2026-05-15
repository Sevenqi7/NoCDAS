/*
 * TraceLogger.hpp
 *
 */

#ifndef TRACELOGGER_HPP_
#define TRACELOGGER_HPP_

#include <fstream>
#include <string>

class Packet;

class TraceLogger {
public:
  static TraceLogger& instance();

  void enable(const std::string& path);
  bool isEnabled() const;
  void logPacket(const Packet& packet);
  void close();

private:
  TraceLogger();
  ~TraceLogger();

  std::ofstream out_file;
  bool enabled;
};

#endif /* TRACELOGGER_HPP_ */
