#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>

#include "websocket_server.hpp"

// Initialize static members
LogLevel Logger::current_level = LogLevel::INFO;
std::mutex Logger::log_mutex;

std::unique_ptr<WebSocketServer> server;

void signalHandler(int signum) {
  LOG_INFO("Main",
           "Received signal " + std::to_string(signum) + ", shutting down...");
  if (server) {
    server->stop();
  }
  exit(0);
}

void printUsage(const char* program_name) {
  std::cout << "Usage: " << program_name << " [OPTIONS]\n"
            << "Options:\n"
            << "  --debug    Enable debug logging\n"
            << "  --help     Show this help message\n";
}

int main(int argc, char* argv[]) {
  // Parse command line arguments
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--debug") == 0) {
      Logger::setLevel(LogLevel::DEBUG);
      std::cout << "Debug logging enabled" << std::endl;
    } else if (strcmp(argv[i], "--help") == 0) {
      printUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown option: " << argv[i] << std::endl;
      printUsage(argv[0]);
      return 1;
    }
  }

  // Register signal handlers
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  LOG_INFO("Main", "IMX519 Camera WebSocket Server");
  LOG_INFO("Main", "==============================");

  try {
    server = std::make_unique<WebSocketServer>();
    server->run();
  } catch (const std::exception& e) {
    LOG_ERROR("Main", "Fatal error: " + std::string(e.what()));
    return 1;
  }

  return 0;
}
