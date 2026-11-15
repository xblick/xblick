// Every program declares itself (Python's declaration.py, protocol.hpp kSessions): a session opened by open_session
// declares the liveliness token xblick/sessions/<zid>/program and a queryable on that key that answers the program
// record. Both live as long as the session: the queryable is a background one (undeclared when the session closes),
// and the token is held by the queryable's drop callback, which releases it then.
#include <unistd.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>

#include "xblick/camera.hpp"
#include "xblick/protocol.hpp"
#include "xblick/version.hpp"
#include "json.hpp"

namespace xblick
{
    namespace p = protocol;

    namespace
    {
        struct Declared
        {
            std::mutex mutex;
            std::vector<std::pair<std::string, json::Value>> fields;  // the record but its names
            std::vector<std::string> names;

            std::string payload()
            {
                std::lock_guard lock(mutex);
                std::vector<std::pair<std::string, json::Value>> record = fields;
                record.emplace_back("names", names);
                return json::write(record);
            }
        };

        std::mutex registry_mutex;
        std::map<std::string, std::weak_ptr<Declared>> registry;  // zid -> its declaration, while its session is open

        std::vector<std::string> command_line()
        {
            std::ifstream in("/proc/self/cmdline", std::ios::binary);
            std::vector<std::string> out;
            std::string arg;
            while (std::getline(in, arg, '\0'))
            {
                out.push_back(arg);
            }
            return out;
        }

        // The process's start in ns since the epoch (/proc: its start in clock ticks after the boot); null where the
        // system does not say.
        json::Value started_ns()
        {
            std::ifstream stat("/proc/self/stat");
            std::string line;
            if (not std::getline(stat, line) or line.rfind(')') == std::string::npos)
            {
                return nullptr;
            }
            std::istringstream after(line.substr(line.rfind(')') + 1));  // the command's name may hold spaces
            std::string field;
            long long ticks = -1;
            for (int n = 3; n <= 22 and after >> field; ++n)
            {
                if (n == 22)
                {
                    ticks = std::stoll(field);
                }
            }
            std::ifstream system("/proc/stat");
            long long boot = -1;
            while (std::getline(system, line))
            {
                if (line.rfind("btime ", 0) == 0)
                {
                    boot = std::stoll(line.substr(6));
                }
            }
            const long tick = ::sysconf(_SC_CLK_TCK);
            if (ticks < 0 or boot < 0 or tick <= 0)
            {
                return nullptr;
            }
            return static_cast<std::int64_t>(boot * 1'000'000'000LL + ticks * 1'000'000'000LL / tick);
        }

        std::string hostname()
        {
            char name[256] = {};
            if (::gethostname(name, sizeof(name) - 1) != 0)
            {
                return "";
            }
            return name;
        }
    }  // namespace

    std::string declaration_key(const std::string& zid)
    {
        return std::string(p::kSessions) + "/" + zid + "/" + std::string(p::kProgram);
    }

    namespace detail
    {
        void declare_program(zenoh::Session& session, std::string program, const std::string& version)
        {
            const std::vector<std::string> command = command_line();
            if (program.empty())
            {
                // the name it was started by, as Python's program_name()
                program = command.empty() ? "c++" : command.front().substr(command.front().rfind('/') + 1);
            }
            const std::string zid = session.get_zid().to_string();
            const std::string key = declaration_key(zid);
            auto declared = std::make_shared<Declared>();
            declared->fields = {{"v", static_cast<std::int64_t>(p::kProgramVersion)},
                                {"kind", std::string("program")},
                                {"zid", zid},
                                {"program", program},
                                {"version", version.empty() ? json::Value(nullptr) : json::Value(version)},
                                {"xblick", std::string(XBLICK_VERSION)},
                                {"language", std::string("c++")},
                                {"host", hostname()},
                                {"command", command},
                                {"pid", static_cast<std::int64_t>(::getpid())},
                                {"started_ns", started_ns()}};
            auto token = std::make_shared<std::optional<zenoh::LivelinessToken>>();
            // answered at once; the query is dropped when the callback returns (the receiver rule, AGENTS.md)
            session.declare_background_queryable(
                zenoh::KeyExpr(key),
                [declared, key](const zenoh::Query& query) { query.reply(zenoh::KeyExpr(key), declared->payload()); },
                [token, zid]() {
                    token->reset();  // the session closes: the token goes with the queryable
                    std::lock_guard lock(registry_mutex);
                    registry.erase(zid);
                });
            token->emplace(session.liveliness_declare_token(zenoh::KeyExpr(key)));
            std::lock_guard lock(registry_mutex);
            registry[zid] = declared;
        }
    }  // namespace detail

    bool add_name(const zenoh::Session& session, std::string name)
    {
        name.erase(0, name.find_first_not_of('/'));
        name.erase(name.find_last_not_of('/') + 1);
        std::shared_ptr<Declared> declared;
        {
            std::lock_guard lock(registry_mutex);
            const auto found = registry.find(session.get_zid().to_string());
            if (found == registry.end() or not(declared = found->second.lock()))
            {
                return false;
            }
        }
        if (not name.empty())
        {
            std::lock_guard lock(declared->mutex);
            if (std::find(declared->names.begin(), declared->names.end(), name) == declared->names.end())
            {
                declared->names.push_back(name);
            }
        }
        return true;
    }
}  // namespace xblick
