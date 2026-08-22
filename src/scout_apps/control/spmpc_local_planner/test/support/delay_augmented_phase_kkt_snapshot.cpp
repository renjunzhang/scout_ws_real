#include "support/delay_augmented_phase_kkt_snapshot.h"

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <utility>

namespace spmpc_local_planner {
namespace test_support {
namespace {

enum class Kind {
    Object, Array, Number, String, True, False, Null,
};

struct JsonNode {
    Kind kind = Kind::Null;
    double number = 0.0;
    std::int64_t integer = 0;
    std::string text;
    std::vector<std::pair<std::string, std::size_t>> members;
    std::vector<std::size_t> children;
};

struct Parser {
    const std::string& raw;
    std::vector<JsonNode>& nodes;
    std::size_t pos = 0;
    std::string error;

    void skipWs() {
        while (pos < raw.size() &&
               std::isspace(static_cast<unsigned char>(raw[pos]))) {
            ++pos;
        }
    }

    std::size_t addNode(Kind kind) {
        JsonNode node;
        node.kind = kind;
        nodes.push_back(std::move(node));
        return nodes.size() - 1;
    }

    bool parseValue(std::size_t& out_index) {
        skipWs();
        if (pos >= raw.size()) {
            error = "unexpected end of document";
            return false;
        }
        switch (raw[pos]) {
            case '{': return parseObject(out_index);
            case '[': return parseArray(out_index);
            case '"': return parseString(out_index);
            case 't': case 'f': return parseBool(out_index);
            case 'n': return parseNull(out_index);
            default: return parseNumber(out_index);
        }
    }

    bool parseObject(std::size_t& out_index) {
        out_index = addNode(Kind::Object);
        ++pos;  // '{'
        skipWs();
        if (pos < raw.size() && raw[pos] == '}') { ++pos; return true; }
        while (true) {
            skipWs();
            if (pos >= raw.size() || raw[pos] != '"') {
                error = "object key must be a string";
                return false;
            }
            std::size_t key_index = 0;
            if (!parseString(key_index)) return false;
            const std::string key = nodes[key_index].text;  // copied, not a ref
            skipWs();
            if (pos >= raw.size() || raw[pos] != ':') {
                error = "object missing ':'";
                return false;
            }
            ++pos;
            std::size_t value_index = 0;
            if (!parseValue(value_index)) return false;
            // Re-fetch by index: addNode() inside the recursive parseValue may
            // have reallocated `nodes`, so no reference may be held here.
            nodes[out_index].members.emplace_back(key, value_index);
            skipWs();
            if (pos >= raw.size()) {
                error = "unterminated object";
                return false;
            }
            if (raw[pos] == ',') { ++pos; continue; }
            if (raw[pos] == '}') { ++pos; return true; }
            error = "object missing ',' or '}'";
            return false;
        }
    }

    bool parseArray(std::size_t& out_index) {
        out_index = addNode(Kind::Array);
        ++pos;  // '['
        skipWs();
        if (pos < raw.size() && raw[pos] == ']') { ++pos; return true; }
        while (true) {
            std::size_t element_index = 0;
            if (!parseValue(element_index)) return false;
            // Re-fetch by index (see parseObject for reallocation rationale).
            nodes[out_index].children.push_back(element_index);
            skipWs();
            if (pos >= raw.size()) {
                error = "unterminated array";
                return false;
            }
            if (raw[pos] == ',') { ++pos; continue; }
            if (raw[pos] == ']') { ++pos; return true; }
            error = "array missing ',' or ']'";
            return false;
        }
    }

    static void appendUtf8(std::string& out, std::uint32_t code) {
        if (code < 0x80u) {
            out.push_back(static_cast<char>(code));
        } else if (code < 0x800u) {
            out.push_back(static_cast<char>(0xC0u | (code >> 6)));
            out.push_back(static_cast<char>(0x80u | (code & 0x3Fu)));
        } else if (code < 0x10000u) {
            out.push_back(static_cast<char>(0xE0u | (code >> 12)));
            out.push_back(static_cast<char>(0x80u | ((code >> 6) & 0x3Fu)));
            out.push_back(static_cast<char>(0x80u | (code & 0x3Fu)));
        } else {
            out.push_back(static_cast<char>(0xF0u | (code >> 18)));
            out.push_back(static_cast<char>(0x80u | ((code >> 12) & 0x3Fu)));
            out.push_back(static_cast<char>(0x80u | ((code >> 6) & 0x3Fu)));
            out.push_back(static_cast<char>(0x80u | (code & 0x3Fu)));
        }
    }

    bool parseString(std::size_t& out_index) {
        out_index = addNode(Kind::String);
        JsonNode& node = nodes[out_index];
        ++pos;  // opening quote
        std::string& out = node.text;
        while (pos < raw.size()) {
            const char c = raw[pos];
            if (c == '"') { ++pos; return true; }
            if (c == '\\') {
                ++pos;
                if (pos >= raw.size()) { error = "bad escape"; return false; }
                const char e = raw[pos];
                switch (e) {
                    case 'n': out.push_back('\n'); ++pos; break;
                    case 't': out.push_back('\t'); ++pos; break;
                    case 'r': out.push_back('\r'); ++pos; break;
                    case 'b': out.push_back('\b'); ++pos; break;
                    case 'f': out.push_back('\f'); ++pos; break;
                    case '\\': out.push_back('\\'); ++pos; break;
                    case '/': out.push_back('/'); ++pos; break;
                    case '"': out.push_back('"'); ++pos; break;
                    case 'u': {
                        if (pos + 4 >= raw.size()) {
                            error = "bad unicode escape";
                            return false;
                        }
                        std::uint32_t code = 0;
                        for (int k = 1; k <= 4; ++k) {
                            const char h = raw[pos + k];
                            code *= 16u;
                            if (h >= '0' && h <= '9') code += static_cast<std::uint32_t>(h - '0');
                            else if (h >= 'a' && h <= 'f') code += static_cast<std::uint32_t>(h - 'a' + 10);
                            else if (h >= 'A' && h <= 'F') code += static_cast<std::uint32_t>(h - 'A' + 10);
                            else { error = "bad unicode escape"; return false; }
                        }
                        appendUtf8(out, code);
                        pos += 5;
                        break;
                    }
                    default:
                        error = "bad escape";
                        return false;
                }
                continue;
            }
            out.push_back(c);
            ++pos;
        }
        error = "unterminated string";
        return false;
    }

    bool parseBool(std::size_t& out_index) {
        if (raw.compare(pos, 4, "true") == 0) {
            out_index = addNode(Kind::True);
            pos += 4;
            return true;
        }
        if (raw.compare(pos, 5, "false") == 0) {
            out_index = addNode(Kind::False);
            pos += 5;
            return true;
        }
        error = "invalid literal";
        return false;
    }

    bool parseNull(std::size_t& out_index) {
        if (raw.compare(pos, 4, "null") == 0) {
            out_index = addNode(Kind::Null);
            pos += 4;
            return true;
        }
        error = "invalid literal";
        return false;
    }

    bool parseNumber(std::size_t& out_index) {
        const std::size_t start = pos;
        while (pos < raw.size() &&
               std::strchr("-+.eE0123456789", raw[pos]) != nullptr) {
            ++pos;
        }
        if (pos == start) {
            error = "invalid number";
            return false;
        }
        out_index = addNode(Kind::Number);
        JsonNode& node = nodes[out_index];
        node.text.assign(raw, start, pos - start);
        const char* begin = raw.c_str() + start;
        char* end = nullptr;
        node.number = std::strtod(begin, &end);
        node.integer = static_cast<std::int64_t>(std::strtoll(begin, &end, 10));
        return true;
    }
};

}  // namespace

struct SnapshotJson::Store {
    std::vector<JsonNode> nodes;
    std::size_t root = 0;
};

SnapshotJson::SnapshotJson() = default;

SnapshotJson::SnapshotJson(std::shared_ptr<const Store> store, std::size_t node)
    : store_(std::move(store)), node_(node) {}

bool SnapshotJson::parse(const std::string& text, SnapshotJson& out,
                        std::string& error) {
    auto store = std::make_shared<Store>();
    Parser parser{text, store->nodes};
    std::size_t root = 0;
    if (!parser.parseValue(root)) {
        error = parser.error;
        return false;
    }
    parser.skipWs();
    if (parser.pos != text.size()) {
        error = "trailing content after JSON document";
        return false;
    }
    store->root = root;
    out = SnapshotJson(std::move(store), root);
    return true;
}

// Accessor methods below are member functions of SnapshotJson and therefore
// have direct access to the private store_/node_ fields and the JsonNode type
// (from the encompassing anonymous namespace).

bool SnapshotJson::isNull() const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    return node == nullptr || node->kind == Kind::Null;
}
bool SnapshotJson::isObject() const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    return node != nullptr && node->kind == Kind::Object;
}
bool SnapshotJson::isArray() const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    return node != nullptr && node->kind == Kind::Array;
}
bool SnapshotJson::isNumber() const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    return node != nullptr && node->kind == Kind::Number;
}
bool SnapshotJson::isString() const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    return node != nullptr && node->kind == Kind::String;
}
bool SnapshotJson::isBool() const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    return node != nullptr &&
        (node->kind == Kind::True || node->kind == Kind::False);
}

SnapshotJson SnapshotJson::find(const std::string& key) const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    if (node == nullptr || node->kind != Kind::Object) return SnapshotJson();
    for (const auto& member : node->members) {
        if (member.first == key) return SnapshotJson(store_, member.second);
    }
    return SnapshotJson();
}

SnapshotJson SnapshotJson::at(std::size_t index) const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    if (node == nullptr || node->kind != Kind::Array ||
        index >= node->children.size()) {
        return SnapshotJson();
    }
    return SnapshotJson(store_, node->children[index]);
}

std::size_t SnapshotJson::size() const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    if (node == nullptr || node->kind != Kind::Array) return 0;
    return node->children.size();
}

bool SnapshotJson::numberArray(std::vector<double>& out) const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    if (node == nullptr || node->kind != Kind::Array) return false;
    out.clear();
    out.reserve(node->children.size());
    for (const std::size_t child : node->children) {
        if (child >= store_->nodes.size()) return false;
        const JsonNode& element = store_->nodes[child];
        if (element.kind != Kind::Number) return false;
        out.push_back(element.number);
    }
    return true;
}

bool SnapshotJson::stringArray(std::vector<std::string>& out) const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    if (node == nullptr || node->kind != Kind::Array) return false;
    out.clear();
    out.reserve(node->children.size());
    for (const std::size_t child : node->children) {
        if (child >= store_->nodes.size()) return false;
        const JsonNode& element = store_->nodes[child];
        if (element.kind != Kind::String) return false;
        out.push_back(element.text);
    }
    return true;
}

bool SnapshotJson::number(double& out) const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    if (node == nullptr || node->kind != Kind::Number) return false;
    out = node->number;
    return true;
}

bool SnapshotJson::integer(std::int64_t& out) const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    if (node == nullptr || node->kind != Kind::Number) return false;
    out = node->integer;
    return true;
}

bool SnapshotJson::boolean(bool& out) const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    if (node == nullptr) return false;
    if (node->kind == Kind::True) { out = true; return true; }
    if (node->kind == Kind::False) { out = false; return true; }
    return false;
}

bool SnapshotJson::stringValue(std::string& out) const {
    const JsonNode* node = (store_ && node_ != kNullNode && node_ < store_->nodes.size())
        ? &store_->nodes[node_] : nullptr;
    if (node == nullptr || node->kind != Kind::String) return false;
    out = node->text;
    return true;
}

}  // namespace test_support
}  // namespace spmpc_local_planner
