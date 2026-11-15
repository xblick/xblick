// A command's node and deadline: the attachment parser and the receiver's checks (command.hpp).
#include "xblick/command.hpp"

#include <cstdio>
#include <map>
#include <stdexcept>
#include <variant>

#include "xblick/protocol.hpp"

namespace xblick
{
    namespace p = protocol;

    namespace
    {
        struct Other  // a value of an unknown kind, or a known kind out of range: only skipped
        {
        };
        using Value = std::variant<Other, std::int64_t, std::string, std::vector<std::string>>;

        struct Bad : std::runtime_error
        {
            using std::runtime_error::runtime_error;
        };

        // UTF-8 as Python's strict decoder takes it: no overlong forms, no surrogates, nothing above U+10FFFF
        bool valid_utf8(std::string_view s)
        {
            std::size_t i = 0;
            const auto byte = [&](std::size_t k) { return static_cast<unsigned char>(s[k]); };
            const auto cont = [&](std::size_t k, unsigned lo, unsigned hi) {
                return k < s.size() and byte(k) >= lo and byte(k) <= hi;
            };
            while (i < s.size())
            {
                const unsigned c = byte(i);
                if (c < 0x80)
                {
                    i += 1;
                }
                else if (c >= 0xC2 and c <= 0xDF and cont(i + 1, 0x80, 0xBF))
                {
                    i += 2;
                }
                else if ((c == 0xE0 and cont(i + 1, 0xA0, 0xBF)) or
                         (c >= 0xE1 and c <= 0xEC and cont(i + 1, 0x80, 0xBF)) or
                         (c == 0xED and cont(i + 1, 0x80, 0x9F)) or
                         (c >= 0xEE and c <= 0xEF and cont(i + 1, 0x80, 0xBF)))
                {
                    if (not cont(i + 2, 0x80, 0xBF))
                    {
                        return false;
                    }
                    i += 3;
                }
                else if ((c == 0xF0 and cont(i + 1, 0x90, 0xBF)) or
                         (c >= 0xF1 and c <= 0xF3 and cont(i + 1, 0x80, 0xBF)) or
                         (c == 0xF4 and cont(i + 1, 0x80, 0x8F)))
                {
                    if (not cont(i + 2, 0x80, 0xBF) or not cont(i + 3, 0x80, 0xBF))
                    {
                        return false;
                    }
                    i += 4;
                }
                else
                {
                    return false;
                }
            }
            return true;
        }

        // JSON (RFC 8259) as Python's json module reads it with strict=False: control characters allowed in strings,
        // no NaN or Infinity. Nesting is limited (kCommandMaxDepth), so the recursion is bounded.
        class Parser
        {
        public:
            explicit Parser(std::string_view s) : s_(s) {}

            std::map<std::string, Value> top()
            {
                ws();
                if (peek() != '{')
                {
                    throw Bad("is no JSON object");
                }
                std::map<std::string, Value> out;
                object(1, &out);
                ws();
                if (i_ != s_.size())
                {
                    throw Bad("has data after its JSON object");
                }
                return out;
            }

        private:
            char peek() const
            {
                if (i_ >= s_.size())
                {
                    throw Bad("is no complete JSON");
                }
                return s_[i_];
            }
            void ws()
            {
                while (i_ < s_.size() and (s_[i_] == ' ' or s_[i_] == '\t' or s_[i_] == '\n' or s_[i_] == '\r'))
                {
                    ++i_;
                }
            }
            void expect(char c)
            {
                if (peek() != c)
                {
                    throw Bad("is no valid JSON");
                }
                ++i_;
            }
            static void depth_ok(int depth)
            {
                if (depth > static_cast<int>(p::kCommandMaxDepth))
                {
                    throw Bad("is nested deeper than " + std::to_string(p::kCommandMaxDepth));
                }
            }
            // an object at `depth`; its top-level members into `out` when given
            void object(int depth, std::map<std::string, Value>* out)
            {
                depth_ok(depth);
                expect('{');
                ws();
                if (peek() == '}')
                {
                    ++i_;
                    return;
                }
                while (true)
                {
                    ws();
                    std::string key = string();
                    ws();
                    expect(':');
                    ws();
                    Value v = value(depth);
                    if (out)
                    {
                        (*out)[key] = std::move(v);
                    }
                    ws();
                    if (peek() == ',')
                    {
                        ++i_;
                        continue;
                    }
                    expect('}');
                    return;
                }
            }
            Value array(int depth)
            {
                depth_ok(depth);
                expect('[');
                std::vector<std::string> strings;
                bool all_strings = true;
                ws();
                if (peek() == ']')
                {
                    ++i_;
                    return strings;
                }
                while (true)
                {
                    ws();
                    Value v = value(depth);
                    if (auto* text = std::get_if<std::string>(&v))
                    {
                        strings.push_back(std::move(*text));
                    }
                    else
                    {
                        all_strings = false;
                    }
                    ws();
                    if (peek() == ',')
                    {
                        ++i_;
                        continue;
                    }
                    expect(']');
                    return all_strings ? Value(std::move(strings)) : Value(Other{});
                }
            }
            // a value inside a container at `depth`
            Value value(int depth)
            {
                const char c = peek();
                if (c == '{')
                {
                    object(depth + 1, nullptr);
                    return Other{};
                }
                if (c == '[')
                {
                    return array(depth + 1);
                }
                if (c == '"')
                {
                    return string();
                }
                for (const char* word : {"true", "false", "null"})
                {
                    const std::string_view w(word);
                    if (s_.substr(i_, w.size()) == w)
                    {
                        i_ += w.size();
                        return Other{};
                    }
                }
                return number();
            }
            Value number()
            {
                const std::size_t start = i_;
                if (i_ < s_.size() and s_[i_] == '-')
                {
                    ++i_;
                }
                const auto digits = [&] {
                    const std::size_t from = i_;
                    while (i_ < s_.size() and s_[i_] >= '0' and s_[i_] <= '9')
                    {
                        ++i_;
                    }
                    return i_ - from;
                };
                const std::size_t first = i_;
                const std::size_t n = digits();
                if (n == 0 or (n > 1 and s_[first] == '0'))
                {
                    throw Bad("is no valid JSON");
                }
                bool integer = true;
                if (i_ < s_.size() and s_[i_] == '.')
                {
                    ++i_;
                    if (digits() == 0)
                    {
                        throw Bad("is no valid JSON");
                    }
                    integer = false;
                }
                if (i_ < s_.size() and (s_[i_] == 'e' or s_[i_] == 'E'))
                {
                    ++i_;
                    if (i_ < s_.size() and (s_[i_] == '+' or s_[i_] == '-'))
                    {
                        ++i_;
                    }
                    if (digits() == 0)
                    {
                        throw Bad("is no valid JSON");
                    }
                    integer = false;
                }
                if (not integer or n > 19)
                {
                    return Other{};
                }
                const std::string text(s_.substr(start, i_ - start));
                try
                {
                    std::size_t used = 0;
                    const long long v = std::stoll(text, &used);
                    return static_cast<std::int64_t>(v);
                }
                catch (const std::out_of_range&)
                {
                    return Other{};
                }
            }
            static int hex(char c)
            {
                if (c >= '0' and c <= '9')
                {
                    return c - '0';
                }
                if (c >= 'a' and c <= 'f')
                {
                    return c - 'a' + 10;
                }
                if (c >= 'A' and c <= 'F')
                {
                    return c - 'A' + 10;
                }
                return -1;
            }
            std::string string()
            {
                expect('"');
                std::string out;
                while (true)
                {
                    const char c = peek();
                    ++i_;
                    if (c == '"')
                    {
                        return out;
                    }
                    if (c != '\\')
                    {
                        out += c;
                        continue;
                    }
                    const char e = peek();
                    ++i_;
                    switch (e)
                    {
                        case '"': out += '"'; break;
                        case '\\': out += '\\'; break;
                        case '/': out += '/'; break;
                        case 'b': out += '\b'; break;
                        case 'f': out += '\f'; break;
                        case 'n': out += '\n'; break;
                        case 'r': out += '\r'; break;
                        case 't': out += '\t'; break;
                        case 'u':
                        {
                            unsigned code = 0;
                            for (int k = 0; k < 4; ++k)
                            {
                                const int h = hex(peek());
                                if (h < 0)
                                {
                                    throw Bad("is no valid JSON");
                                }
                                code = code * 16 + static_cast<unsigned>(h);
                                ++i_;
                            }
                            // as UTF-8 (a surrogate as its 3 bytes: such names are compared, never shown)
                            if (code < 0x80)
                            {
                                out += static_cast<char>(code);
                            }
                            else if (code < 0x800)
                            {
                                out += static_cast<char>(0xC0 | (code >> 6));
                                out += static_cast<char>(0x80 | (code & 0x3F));
                            }
                            else
                            {
                                out += static_cast<char>(0xE0 | (code >> 12));
                                out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
                                out += static_cast<char>(0x80 | (code & 0x3F));
                            }
                            break;
                        }
                        default: throw Bad("is no valid JSON");
                    }
                }
            }

            std::string_view s_;
            std::size_t i_ = 0;
        };

        template <typename T>
        std::optional<T> field(const std::map<std::string, Value>& o, const char* key)
        {
            const auto it = o.find(key);
            if (it == o.end())
            {
                return std::nullopt;
            }
            if (const T* v = std::get_if<T>(&it->second))
            {
                return *v;
            }
            return std::nullopt;
        }
    }  // namespace

    CommandAttachment parse_command_attachment(std::string_view bytes)
    {
        CommandAttachment a;
        const std::string marker = "\"" + std::string(p::kCommandField) + "\"";
        if (bytes.find(marker) == std::string_view::npos)
        {
            return a;  // another program's attachment
        }
        a.kind = CommandAttachment::Kind::invalid;
        const std::string what = "its " + std::string(p::kCommandField) + " attachment ";
        if (bytes.size() > p::kCommandMaxBytes)
        {
            a.problem = what + "is longer than " + std::to_string(p::kCommandMaxBytes) + " bytes";
            return a;
        }
        if (not valid_utf8(bytes))
        {
            a.problem = what + "is no UTF-8";
            return a;
        }
        try
        {
            const std::map<std::string, Value> o = Parser(bytes).top();
            if (not o.count(std::string(p::kCommandField)))
            {
                a.problem = what + "names " + std::string(p::kCommandField) + " below its top level";
                return a;
            }
            a.kind = CommandAttachment::Kind::ours;
            a.version = field<std::int64_t>(o, p::kCommandField.data());
            a.deadline_ns = field<std::int64_t>(o, "deadline_ns");
            a.clock_ns = field<std::int64_t>(o, "clock_ns");
            a.node = field<std::string>(o, "node");
            a.zid = field<std::string>(o, "zid");
            a.checked = field<std::vector<std::string>>(o, "checked");
        }
        catch (const std::exception& e)  // Bad, or anything a malformed input may cause: refused, never thrown on
        {
            a.kind = CommandAttachment::Kind::invalid;
            a.problem = what + e.what();
        }
        return a;
    }

    std::optional<CommandRefusal> refuse_on_receipt(const CommandAttachment& a, const std::string& zid)
    {
        const std::string what = "its " + std::string(p::kCommandField) + " attachment ";
        if (a.kind == CommandAttachment::Kind::none)
        {
            return std::nullopt;
        }
        if (a.kind == CommandAttachment::Kind::invalid)
        {
            return CommandRefusal{std::string(p::kInvalidCommand), a.problem};
        }
        if (a.version != static_cast<std::int64_t>(p::kCommandVersion))
        {
            return CommandRefusal{std::string(p::kInvalidCommand),
                                  what + "is not of version " + std::to_string(p::kCommandVersion) +
                                      ", the version this node reads"};
        }
        if (not a.node or a.node->empty() or not a.deadline_ns or *a.deadline_ns < 0)
        {
            return CommandRefusal{std::string(p::kInvalidCommand), what + "has no node and deadline_ns"};
        }
        if (*a.node != zid)
        {
            return CommandRefusal{std::string(p::kNotTarget),
                                  "it is meant for the node of session " + *a.node + ", this is " + zid};
        }
        return std::nullopt;
    }

    std::optional<CommandRefusal> refuse_at_effect(const CommandAttachment& a, std::int64_t now_ns)
    {
        if (a.kind != CommandAttachment::Kind::ours or not a.deadline_ns)
        {
            return std::nullopt;
        }
        const std::int64_t deadline = *a.deadline_ns;  // >= 0 (refuse_on_receipt), now_ns >= 0: no overflow below
        if (now_ns >= deadline)
        {
            char seconds[64];
            std::snprintf(seconds, sizeof seconds, "%.3f",
                          (static_cast<double>(now_ns) - static_cast<double>(deadline)) / 1e9);
            return CommandRefusal{std::string(p::kExpired), std::string("its deadline had passed ") + seconds +
                                                                " s before this node came to execute it"};
        }
        if (deadline - now_ns > p::kCommandMaxAheadNs)
        {
            return CommandRefusal{std::string(p::kInvalidCommand),
                                  "its deadline lies more than " + std::to_string(p::kCommandMaxAheadNs / 1000000000) +
                                      " s ahead"};
        }
        return std::nullopt;
    }
}  // namespace xblick
