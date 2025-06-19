#include <csignal>
#include <iostream>
#include <memory>

#include "websocket_server.hpp"

std::unique_ptr<WebSocketServer> server;

void signalHandler(int signum) {
  std::cout << "\nReceived signal " << signum << ", shutting down..."
            << std::endl;
  if (server) {
    server->stop();
  }
  exit(0);
}

int main(int argc, char* argv[]) {
  // Register signal handlers
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  std::cout << "IMX296 Camera WebSocket Server" << std::endl;
  std::cout << "==============================" << std::endl;

  try {
    server = std::make_unique<WebSocketServer>();
    server->run();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
