// A command's node and deadline (protocol.hpp, kCommandField): the attachment parser and the receiver's two checks,
// without Zenoh, so that Python (xblick._core) and the tests run the same rules. camera.hpp's CommandQuery and
// target_of use them. Python's session module implements the same rules; tests/test_queries.py runs one table of
// inputs through both.
//
// Rules of an attachment (version 1):
// - It is a command's attachment when its bytes contain "xblick_command" (with the quotes). Any other attachment is
//   another program's and is ignored (kind none).
// - A command's attachment is a JSON object in UTF-8, nested at most kCommandMaxDepth deep, with nothing but JSON
//   whitespace after it, whose top level has the field xblick_command. Anything else is invalid.
// - Known fields have fixed types: xblick_command, deadline_ns and clock_ns integers (no fraction, no exponent, within
//   int64; true is no integer), node and zid strings, checked a list of strings. A known field of another type counts
//   as missing. Unknown fields of any JSON type are ignored, so that version 1 can grow.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace xblick
{
    struct CommandAttachment
    {
        enum class Kind
        {
            none,     // no command's attachment: another program's, or none at all
            invalid,  // names xblick_command, but breaks the rules above (problem says how)
            ours,     // a JSON object with xblick_command at its top level
        };
        Kind kind = Kind::none;
        std::string problem;
        std::optional<std::int64_t> version, deadline_ns, clock_ns;
        std::optional<std::string> node, zid;
        std::optional<std::vector<std::string>> checked;
    };

    // The attachment's bytes as the rules above read them. Never throws.
    CommandAttachment parse_command_attachment(std::string_view bytes);

    // A refusal before a command's effect: schema (protocol.hpp kExpired, kNotTarget, kInvalidCommand) and why.
    struct CommandRefusal
    {
        std::string schema;
        std::string reason;
    };

    // The first check, on receipt, before anything else: an attachment it cannot read (invalid, version other than 1,
    // node or deadline_ns missing, a negative deadline) or a command meant for another node than `zid`. Empty: go on
    // (also for an attachment of kind none: nothing to check).
    std::optional<CommandRefusal> refuse_on_receipt(const CommandAttachment& a, const std::string& zid);

    // The second check, just before the effect, on this PC's command clock `now_ns`: the deadline reached, or one more
    // than kCommandMaxAheadNs ahead (no sender's). Empty: go on. Call it only after refuse_on_receipt gave none.
    std::optional<CommandRefusal> refuse_at_effect(const CommandAttachment& a, std::int64_t now_ns);
}  // namespace xblick
