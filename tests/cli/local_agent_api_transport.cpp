// Standalone transport harness: deliberately has no wxWidgets or slicer linkage.
#include "GUI/LocalAgentApi.hpp"

#include <chrono>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

int main()
{
    using Slic3r::GUI::LocalAgentApi;
    LocalAgentApi api;
    std::mutex mutex;
    LocalAgentApi::Reply deferred;
    std::vector<std::thread> workers;
    const bool started = api.start([&](const std::string &method, const std::string &target, LocalAgentApi::Reply reply) {
        if (target == "/defer") {
            std::lock_guard<std::mutex> lock(mutex);
            deferred = std::move(reply);
            return;
        }
        if (target == "/async") {
            std::lock_guard<std::mutex> lock(mutex);
            workers.emplace_back([reply] {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                reply(200, {{"async", true}});
            });
            return;
        }
        if (target == "/throw")
            throw std::runtime_error("test handler failure");
        if (target == "/large") {
            reply(200, {{"large", std::string(1024 * 1024, 'x')}});
            return;
        }
        reply(200, {{"method", method}, {"target", target}});
        if (target == "/twice")
            reply(201, {{"second", true}});
    });
    std::cout << (started ? "READY" : "DISABLED") << std::endl;
    if (!started)
        return 0;
    std::string input;
    std::getline(std::cin, input);
    api.stop();
    api.stop();
    // CallAfter-style retained callbacks must be harmless after stop, and even
    // after the transport's IO context has gone away.
    if (deferred)
        deferred(200, {{"late", true}});
    for (auto &worker : workers)
        worker.join();
    std::cout << "STOPPED" << std::endl;
}
