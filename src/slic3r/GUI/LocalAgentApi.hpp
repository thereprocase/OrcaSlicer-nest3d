#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <nlohmann/json_fwd.hpp>

namespace Slic3r::GUI {

// Opt-in, authenticated loopback transport. Handler runs on the IO thread;
// dispatch GUI work with CallAfter. Reply is safe to retain across shutdown.
class LocalAgentApi
{
public:
    using Reply = std::function<void(int, const nlohmann::json &)>;
    using Handler = std::function<void(const std::string &, const std::string &, Reply)>;

    LocalAgentApi() = default;
    ~LocalAgentApi();
    LocalAgentApi(const LocalAgentApi &) = delete;
    LocalAgentApi &operator=(const LocalAgentApi &) = delete;

    bool start(Handler handler);
    void stop();

private:
    struct State;
    std::mutex             m_mutex;
    std::shared_ptr<State> m_state;
    std::thread            m_thread;
};

} // namespace Slic3r::GUI
