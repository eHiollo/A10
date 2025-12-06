#ifndef A10_TCP_SERVER_HPP_
#define A10_TCP_SERVER_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Thread-safe TCP Server for A10 robot.
// Acts as a server that accepts a connection from another framework.
// Protocol: Line-delimited JSON.
class A10TcpServer
{
public:
    A10TcpServer();
    ~A10TcpServer();

    // Start listening on the specified port
    bool start(uint16_t port);
    void stop();

    // Send joint data to the connected client
    bool send_set_joints(const std::vector<double> &q);

    // Get latest joint state received from any client
    std::vector<double> get_target_q();

private:
    void acceptLoop();
    void readerLoop(int client_sock);
    bool send_line(const std::string &line);
    void send_leader_state(int client_sock); // Helper to send current state to a specific client
    void send_follower_state(int client_sock);
    void process_line(int client_sock, const std::string &line);

    int server_sockfd_;
    std::vector<int> client_sockfds_; // List of connected clients
    std::atomic<bool> running_;
    std::thread accept_thread_;
    
    std::mutex clients_mutex_; // Protects client_sockfds_ access
    
    std::mutex q_mutex_;
    std::vector<double> target_q_; // Command from client

    std::mutex robot_q_mutex_;
    std::vector<double> robot_q_; // Feedback from robot
};

extern A10TcpServer* g_tcp_server;

#endif // A10_TCP_SERVER_HPP_
