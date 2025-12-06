#include "a10_tcp_server.hpp"
#include <algorithm>

A10TcpServer::A10TcpServer()
  : server_sockfd_(-1), running_(false)
{}

A10TcpServer::~A10TcpServer()
{
    stop();
}

bool A10TcpServer::start(uint16_t port)
{
    stop();

    server_sockfd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_sockfd_ < 0)
    {
        perror("socket");
        return false;
    }

    int opt = 1;
    if (setsockopt(server_sockfd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        perror("setsockopt");
        return false;
    }

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (::bind(server_sockfd_, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        perror("bind");
        return false;
    }

    if (::listen(server_sockfd_, 5) < 0)
    {
        perror("listen");
        return false;
    }

    running_.store(true);
    accept_thread_ = std::thread(&A10TcpServer::acceptLoop, this);
    return true;
}

void A10TcpServer::stop()
{
    running_.store(false);

    // Close server socket to unblock accept
    if (server_sockfd_ >= 0)
    {
        ::shutdown(server_sockfd_, SHUT_RDWR);
        ::close(server_sockfd_);
        server_sockfd_ = -1;
    }

    // Close all client sockets
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (int sock : client_sockfds_)
        {
            ::shutdown(sock, SHUT_RDWR);
            ::close(sock);
        }
        client_sockfds_.clear();
    }

    if (accept_thread_.joinable())
        accept_thread_.join();
}

void A10TcpServer::acceptLoop()
{
    while (running_.load())
    {
        struct sockaddr_in address;
        int addrlen = sizeof(address);
        int new_socket = ::accept(server_sockfd_, (struct sockaddr *)&address, (socklen_t*)&addrlen);
        
        if (new_socket < 0)
        {
            if (running_.load()) {
                perror("accept");
            }
            break;
        }

        std::cout << "New connection accepted: " << new_socket << std::endl;

        // Add to list
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            client_sockfds_.push_back(new_socket);
        }

        // Start reader thread for this client
        std::thread(&A10TcpServer::readerLoop, this, new_socket).detach();
    }
}


// 持续监听，检测header
void A10TcpServer::readerLoop(int client_sock)
{
    std::string buffer;
    buffer.reserve(1024);

    while (running_.load())
    {
        char tmp[1024];
        ssize_t n = ::recv(client_sock, tmp, sizeof(tmp), 0);
        if (n > 0)
        {
            buffer.append(tmp, tmp + n);
            size_t pos;
            // 当接收的数据出现“/n”时，表示一行数据接收完毕，转入process_line处理
            while ((pos = buffer.find('\n')) != std::string::npos)
            {
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);
                process_line(client_sock, line);
            }
        }
        else
        {
            if (n == 0) std::cout << "Client disconnected: " << client_sock << std::endl;
            else perror("recv");
            break;
        }
    }

    // Cleanup
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        auto it = std::find(client_sockfds_.begin(), client_sockfds_.end(), client_sock);
        if (it != client_sockfds_.end())
        {
            client_sockfds_.erase(it);
        }
        ::close(client_sock);
    }
}

bool A10TcpServer::send_set_joints(const std::vector<double> &q)
{
    // 将获取的q存在robot_q_中，Aris是一毫秒推进来一次
    {
        std::lock_guard<std::mutex> lk(robot_q_mutex_);
        robot_q_ = q;
    }

    // 不需要主动广播
    // std::string payload = "{\"q\": [";
    // for (size_t i = 0; i < q.size(); ++i)
    // {
    //     if (i) payload += ", ";
    //     payload += std::to_string(q[i]);
    // }
    // payload += "]}\n";
    // std::string cmd = std::string("SET_JOINTS ") + payload;
    // return send_line(cmd);
    return true;
}

bool A10TcpServer::send_line(const std::string &line)
{
    std::lock_guard<std::mutex> lock(clients_mutex_);
    if (client_sockfds_.empty())
        return false;

    bool any_success = false;
    const char *buf = line.c_str();
    size_t len = line.size();

    for (int sock : client_sockfds_)
    {
        ssize_t total = 0;
        size_t left = len;
        bool success = true;
        while (left > 0)
        {
            ssize_t n = ::send(sock, buf + total, left, MSG_NOSIGNAL);
            if (n <= 0)
            {
                success = false;
                break;
            }
            total += n;
            left -= n;
        }
        if (success) any_success = true;
    }
    return any_success;
}

std::vector<double> A10TcpServer::get_target_q()
{
    std::lock_guard<std::mutex> lk(q_mutex_);
    return target_q_;
    //std::cout << "get_target_q函数输出:[";
    // for ( int i = 0; i < target_q_.size(); ++i)
    // {   
    //     std::cout << target_q_[i] << (i < target_q_.size() - 1 ? ",":"" );
    // }
    // std::cout << "\n";
}

void A10TcpServer::send_leader_state(int client_sock)
{
    std::vector<double> current_q;
    {
        std::lock_guard<std::mutex> lk(robot_q_mutex_);
        current_q = robot_q_;
    }
    
    if (current_q.empty())
    {
        current_q.resize(12, 0.0);
    }

    // Only send the first 6 joints
    size_t send_count = 6;

    std::string payload = "{\"q\": [";
    for (size_t i = 0; i < send_count; ++i)
    {
        if (i) payload += ", ";
        payload += std::to_string(current_q[i+6]);
    }
    payload += "]}\n";
    
    // Send directly to the requesting client
    ssize_t total = 0;
    const char *buf = payload.c_str();
    size_t left = payload.size();
    while (left > 0)
    {
        ssize_t n = ::send(client_sock, buf + total, left, MSG_NOSIGNAL);
        if (n <= 0) break;
        total += n;
        left -= n;
    }
}

void A10TcpServer::send_follower_state(int client_sock)
{
    std::vector<double> current_q;
    {
        std::lock_guard<std::mutex> lk(robot_q_mutex_);
        current_q = robot_q_;
    }
    
    if (current_q.size() < 12)
    {
        current_q.resize(12, 0.0);
    }

    // 
    size_t start_idx = 0;
    size_t end_idx = 6;

    std::string payload = "{\"q\": [";
    for (size_t i = start_idx; i < end_idx; ++i)
    {
        if (i > start_idx) payload += ", ";
        payload += std::to_string(current_q[i]);
    }
    payload += "]}\n";
    
    // Send directly to the requesting client
    ssize_t total = 0;
    const char *buf = payload.c_str();
    size_t left = payload.size();
    while (left > 0)
    {
        ssize_t n = ::send(client_sock, buf + total, left, MSG_NOSIGNAL);
        if (n <= 0) break;
        total += n;
        left -= n;
    }
}

void A10TcpServer::process_line(int client_sock, const std::string &line)
{
    // 响应“”GET_LEADER_STATE”请求
    if (line.find("GET_LEADER_STATE") != std::string::npos)
    {
        send_leader_state(client_sock);
        return;
    }

    // 响应“”GET_FOLLOWER_STATE”请求
    if (line.find("GET_FOLLOWER_STATE") != std::string::npos)
    {
        send_follower_state(client_sock);
        return;
    }

    // 响应“SET_JOINTS {....}”请求
    size_t qpos = line.find("\"q\"");
    if (qpos == std::string::npos)
    {
        std::cout << "Received unknown command: " << line << std::endl;
        return;
    }
    size_t lbr = line.find('[', qpos);
    size_t rbr = line.find(']', lbr == std::string::npos ? 0 : lbr);
    if (lbr == std::string::npos || rbr == std::string::npos || rbr <= lbr)
        return;

    std::string arr = line.substr(lbr + 1, rbr - lbr - 1);
    std::vector<double> parsed;
    size_t idx = 0;
    //逗号分隔符解析器，用于解析收到的JSON数据
    while (idx < arr.size())
    {
        while (idx < arr.size() && isspace((unsigned char)arr[idx])) ++idx;
        if (idx >= arr.size()) break;
        size_t comma = arr.find(',', idx);
        std::string token;
        if (comma == std::string::npos)
        {
            token = arr.substr(idx);
            idx = arr.size();
        }
        else
        {
            token = arr.substr(idx, comma - idx);
            idx = comma + 1;
        }
        size_t b = 0, e = token.size();
        while (b < e && isspace((unsigned char)token[b])) ++b;
        while (e > b && isspace((unsigned char)token[e-1])) --e;
        if (e <= b) continue;
        std::string numstr = token.substr(b, e-b);
        try
        {
            double v = std::stod(numstr);
            parsed.push_back(v);
        }
        catch (...) { }
    }

    //将解析的数据存在了私有变量q_中，供外部获取
    if (!parsed.empty())
    {
        std::lock_guard<std::mutex> lk(q_mutex_);
        target_q_ = parsed;

        // static int printcount = 0;
        // if( printcount++ % 1000 ==0)
        // {   std::cout << "Received target_q:[";
        //     for ( int i = 0; i < target_q_.size(); ++i)
        //     {
        //         std::cout << target_q_[i] << (i < target_q_.size() - 1 ? ",":"" );
        //     }
        //     std::cout << "]\n";
        // }
    }
}
