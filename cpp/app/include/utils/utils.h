#pragma once

#include <string>

#include <json/value.h>

namespace project
{
namespace utils
{

void WaitForSignal();

Json::Value LoadJsonConfig(const std::string& config_path);

void crashHandler();

}  // namespace utils
}  // namespace project
