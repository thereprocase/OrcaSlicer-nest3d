#include "LocalAgentApi.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address_v4.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/beast/core/flat_buffer.hpp>
#include <boost/beast/http/empty_body.hpp>
#include <boost/beast/http/field.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/beast/http/parser.hpp>
#include <boost/beast/http/read.hpp>
#include <boost/beast/http/status.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/beast/http/verb.hpp>
#include <boost/beast/http/write.hpp>
#include <boost/system/error_code.hpp>
#include <nlohmann/json.hpp>

namespace Slic3r::GUI {
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;

struct LocalAgentApi::State : std::enable_shared_from_this<State>
{
    struct Session;
    asio::io_context io;
    tcp::acceptor acceptor{io};
    std::mutex gate;
    bool running = true;
    std::string authorization;
    Handler handler;
    std::set<std::shared_ptr<Session>> sessions;

    void accept();
    void shutdown();
};

struct LocalAgentApi::State::Session : std::enable_shared_from_this<Session>
{
    std::shared_ptr<State> state;
    tcp::socket socket;
    asio::steady_timer deadline;
    beast::flat_buffer buffer{8192};
    http::request_parser<http::empty_body> parser;
    http::response<http::string_body> response;
    bool closed = false;
    bool replied = false;

    Session(std::shared_ptr<State> owner, tcp::socket connection)
        : state(std::move(owner)), socket(std::move(connection)), deadline(state->io)
    {
        parser.header_limit(8192);
        parser.body_limit(0);
    }

    void close()
    {
        if (closed)
            return;
        closed = true;
        boost::system::error_code ignored;
        deadline.cancel(ignored);
        socket.close(ignored);
        state->sessions.erase(shared_from_this());
    }

    void send(int status, std::string body)
    {
        if (closed || replied)
            return;
        replied = true;
        if (status < 200 || status > 599 || body.size() > 1024 * 1024) {
            status = 500;
            body = R"({"error":"invalid_response"})";
        }
        response.version(11);
        response.result(static_cast<http::status>(status));
        response.set(http::field::content_type, "application/json");
        response.set(http::field::cache_control, "no-store");
        response.keep_alive(false);
        response.body() = std::move(body);
        response.prepare_payload();
        http::async_write(socket, response, [self = shared_from_this()](boost::system::error_code, std::size_t) {
            self->close();
        });
    }

    void read()
    {
        deadline.expires_after(std::chrono::seconds(10));
        deadline.async_wait([self = shared_from_this()](boost::system::error_code error) {
            if (!error)
                self->close();
        });
        http::async_read_header(socket, buffer, parser,
            [self = shared_from_this()](boost::system::error_code error, std::size_t) {
                if (error) {
                    self->close();
                    return;
                }
                self->dispatch();
            });
    }

    void dispatch()
    {
        if (closed)
            return;
        const auto &request = parser.get();
        if (request.count(http::field::origin)) {
            send(403, R"({"error":"origin_not_allowed"})");
            return;
        }
        // Compare every byte after the length check; never log credentials.
        const auto auth = request[http::field::authorization];
        unsigned difference = 0;
        if (request.count(http::field::authorization) != 1 || auth.size() != state->authorization.size()) {
            send(401, R"({"error":"unauthorized"})");
            return;
        }
        for (std::size_t i = 0; i < auth.size(); ++i)
            difference |= static_cast<unsigned char>(auth[i]) ^ static_cast<unsigned char>(state->authorization[i]);
        if (difference) {
            send(401, R"({"error":"unauthorized"})");
            return;
        }
        if (request.method() != http::verb::get && request.method() != http::verb::post) {
            send(405, R"({"error":"method_not_allowed"})");
            return;
        }
        if (request.count(http::field::transfer_encoding) || parser.content_length().value_or(0) != 0) {
            send(400, R"({"error":"body_not_allowed"})");
            return;
        }
        const std::string target(request.target());
        if (target.empty() || target.front() != '/' || target.size() > 2048) {
            send(400, R"({"error":"invalid_target"})");
            return;
        }
        const std::weak_ptr<State> owner = state;
        const std::weak_ptr<Session> connection = shared_from_this();
        Reply reply = [owner, connection](int status, const nlohmann::json &value) {
            auto live = owner.lock();
            if (!live)
                return;
            {
                std::lock_guard<std::mutex> lock(live->gate);
                if (!live->running)
                    return;
            }
            std::string body;
            try {
                body = value.dump();
            } catch (...) {
                status = 500;
                body = R"({"error":"invalid_response"})";
            }
            if (body.size() > 1024 * 1024) {
                status = 500;
                body = R"({"error":"invalid_response"})";
            }
            std::lock_guard<std::mutex> lock(live->gate);
            if (!live->running)
                return;
            asio::post(live->io, [connection, status, body = std::move(body)]() mutable {
                if (auto session = connection.lock())
                    session->send(status, std::move(body));
            });
        };
        try {
            state->handler(std::string(request.method_string()), target, std::move(reply));
        } catch (...) {
            send(500, R"({"error":"handler_failed"})");
        }
    }
};

void LocalAgentApi::State::accept()
{
    acceptor.async_accept([self = shared_from_this()](boost::system::error_code error, tcp::socket socket) {
        if (!error && self->sessions.size() < 16) {
            auto session = std::make_shared<Session>(self, std::move(socket));
            self->sessions.insert(session);
            session->read();
        }
        // Excess connections are closed by the socket destructor.
        if (self->acceptor.is_open())
            self->accept();
    });
}

void LocalAgentApi::State::shutdown()
{
    boost::system::error_code ignored;
    acceptor.close(ignored);
    while (!sessions.empty()) {
        auto session = *sessions.begin();
        session->close();
    }
    handler = {};
}

LocalAgentApi::~LocalAgentApi() { stop(); }

bool LocalAgentApi::start(Handler handler)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_state || !handler)
        return false;
    const char *token = std::getenv("ORCA_AGENT_API_TOKEN");
    if (!token)
        return false;
    const std::string secret(token);
    if (secret.size() < 32 || secret.size() > 4096 ||
        !std::all_of(secret.begin(), secret.end(), [](unsigned char c) { return c >= 0x21 && c <= 0x7e; }))
        return false;
    unsigned port = 13619;
    if (const char *configured = std::getenv("ORCA_AGENT_API_PORT")) {
        port = 0;
        if (!*configured)
            return false;
        for (const char *p = configured; *p; ++p) {
            if (*p < '0' || *p > '9' || port > 6553)
                return false;
            port = port * 10 + static_cast<unsigned>(*p - '0');
        }
        if (port == 0 || port > 65535)
            return false;
    }
    auto state = std::make_shared<State>();
    state->authorization = "Bearer " + secret;
    state->handler = std::move(handler);
    boost::system::error_code error;
    state->acceptor.open(tcp::v4(), error);
    if (error)
        return false;
    state->acceptor.bind({asio::ip::make_address_v4("127.0.0.1"), static_cast<unsigned short>(port)}, error);
    if (error)
        return false;
    state->acceptor.listen(16, error);
    if (error)
        return false;
    state->accept();
    try {
        m_thread = std::thread([state] { state->io.run(); });
    } catch (...) {
        state->shutdown();
        state->io.poll();
        return false;
    }
    m_state = std::move(state);
    return true;
}

void LocalAgentApi::stop()
{
    std::thread thread;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_state)
            return;
        auto state = std::move(m_state);
        {
            std::lock_guard<std::mutex> gate(state->gate);
            state->running = false;
            asio::post(state->io, [state] { state->shutdown(); });
        }
        thread = std::move(m_thread);
    }
    if (thread.get_id() == std::this_thread::get_id())
        thread.detach();
    else
        thread.join();
}

} // namespace Slic3r::GUI
