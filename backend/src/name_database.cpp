#include "multi_radio/name_database.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>

namespace multi_radio {

namespace {

std::string JsonEscape(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    if (c == '"')  { out += "\\\""; continue; }
    if (c == '\\') { out += "\\\\"; continue; }
    if (c == '\n') { out += "\\n";  continue; }
    if (c == '\r') { out += "\\r";  continue; }
    out += c;
  }
  return out;
}

// Minimal flat JSON object parser: {"key":"value",...}
std::unordered_map<std::string, std::string> ParseFlatJson(const std::string& src) {
  std::unordered_map<std::string, std::string> out;
  const char* p = src.c_str();
  while (*p && *p != '{') ++p;
  if (*p != '{') return out;
  ++p;
  while (*p && *p != '}') {
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r' || *p == ',') ++p;
    if (*p != '"') break;
    ++p;
    std::string key;
    while (*p && *p != '"') {
      if (*p == '\\' && *(p+1)) { ++p; }
      key += *p++;
    }
    if (*p == '"') ++p;
    while (*p == ' ' || *p == ':') ++p;
    if (*p != '"') { while (*p && *p != ',' && *p != '}') ++p; continue; }
    ++p;
    std::string val;
    while (*p && *p != '"') {
      if (*p == '\\' && *(p+1)) { ++p; }
      val += *p++;
    }
    if (*p == '"') ++p;
    if (!key.empty()) out[key] = val;
  }
  return out;
}

}  // namespace

NameDatabase::NameDatabase(std::filesystem::path path) : path_(std::move(path)) {
  Load();
}

NameDatabase::~NameDatabase() = default;

void NameDatabase::Learn(const std::string& key, const std::string& name) {
  if (key.empty() || name.empty()) return;
  std::lock_guard<std::mutex> lock(mu_);
  auto it = db_.find(key);
  if (it != db_.end() && it->second == name) return;
  db_[key] = name;
  Save();
}

std::string NameDatabase::Lookup(const std::string& key) const {
  if (key.empty()) return {};
  std::lock_guard<std::mutex> lock(mu_);
  auto it = db_.find(key);
  return it != db_.end() ? it->second : std::string{};
}

size_t NameDatabase::Size() const {
  std::lock_guard<std::mutex> lock(mu_);
  return db_.size();
}

void NameDatabase::Load() {
  std::ifstream f(path_);
  if (!f.is_open()) return;
  std::ostringstream ss;
  ss << f.rdbuf();
  db_ = ParseFlatJson(ss.str());
}

void NameDatabase::Save() const {
  std::ofstream f(path_, std::ios::trunc);
  if (!f.is_open()) return;
  f << '{';
  bool first = true;
  for (const auto& [k, v] : db_) {
    if (!first) f << ',';
    f << '"' << JsonEscape(k) << "\":\"" << JsonEscape(v) << '"';
    first = false;
  }
  f << '}';
}

}  // namespace multi_radio
